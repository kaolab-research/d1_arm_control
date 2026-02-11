import pybullet as p
import pybullet_data
import socket
import numpy as np
import struct
import math
import signal 
import sys
import time
import csv
from scipy.spatial.transform import Rotation as R

from oculus_reader_repo.oculus_reader.reader import OculusReader

NORMAL = 0 
DEBUG = 1
MODE = NORMAL

MSG_REQUEST_ANGLES = 0
MSG_COMMAND_ANGLES = 1
MSG_PING = 3
MSG_HOME_ARM = 4
MSG_COMMAND_GRIPPER = 5

class ArmClient: 
    """ Handle Communication to C++ Arm Control Interface """

    def __init__(self, host='192.168.123.18', port=5555):
        self.host = host
        self.port = port
        self.sock = None 
    
    def connect(self):
        """ Connect to C++ D1 Arm Controller"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.connect((self.host, self.port))
        print(f"Connected to D1 Arm Control at {self.host}:{self.port}")

    def disconnect(self): 
        if self.sock:
            self.sock.close()
            self.sock = None
    
    def ping(self):
        try: 
            self.sock.send(struct.pack('B', MSG_PING))
            response = self.sock.recv(2)
            return response == b'OK'
        except Exception as e: 
            print(f"Ping failed: {e}")
            return False
    
    def request_current_angles(self): 
        try:
            self.sock.send(struct.pack('B', MSG_REQUEST_ANGLES))

            num_joints_data = self.sock.recv(1) 
            if not num_joints_data: 
                print("Error: No response from arm controller")
                return None
            
            num_joints = struct.unpack('B', num_joints_data)[0]

            if num_joints == 0: 
                print("Error: D1 Arm Controller returned 0 joints")
                return None 
            
            # Receive joint angles 
            angles_size = num_joints * 4
            angles_data = b''

            while len(angles_data) < angles_size: 
                data = self.sock.recv(angles_size - len(angles_data))
                if not data:
                    print("Error: Connection closed while receiving angles")
                    return None 
                angles_data += data

            angles = struct.unpack(f'{num_joints}f', angles_data)
            print(f"Received angles: {[f'{a:.3f}' for a in angles]}")
            return list(angles)
        except Exception as e: 
            print(f"Error requesting current angles: {e}")
            return None 
    
    def command_angles(self, joint_angles, gripper_width):
        try:
            if len(joint_angles) > 7: 
                print(f"Error: Too many joint angles provided for Arm Control")
                return False
            
            result_angles = struct.pack('B', MSG_COMMAND_ANGLES)
            result_angles += struct.pack('B', len(joint_angles))
            result_angles += struct.pack(f'{len(joint_angles)}f', *joint_angles)
            result_angles += struct.pack('f', gripper_width)
            self.sock.send(result_angles)

            # Potentially Add an ack here # 
            return True

        except Exception as e: 
            print(f"Error commanding angles: {e}")
            return False
    
    def command_gripper(self, gripper_width):
        try:
            result_angles = struct.pack('B', MSG_COMMAND_GRIPPER)
            result_angles += struct.pack('f', gripper_width)
            self.sock.send(result_angles)

            return True

        except Exception as e: 
            print(f"Error commanding gripper: {e}")
            return False
        
    def home_arm(self):
        try: 
            self.sock.send(struct.pack('B', MSG_HOME_ARM))
            return True
        except Exception as e: 
            print(f"Error homing robot arm: {e}")
            return False

class TeleopController:
    def __init__(self, ik_server, arm_client):
        self.ik_server = ik_server
        self.arm_client = arm_client

        # Offset Tracking
        self.position_offset = None
        self.orientation_offset = None

        # Current Pose Tracking 
        self.current_angles_deg = None
        self.gripper_width = 0.0

        # Button State Tracking
        self.prev_button_state = False

        # Coordinate frame transformation
        self.setup_coordinate_transform()

        self.GRIPPER_STEP = 0.02

    def setup_coordinate_transform(self):
        """
        Transform from Quest frame to D1 Arm Frame

        Quest Controller: +X=Left, +Y=Up, +Z=Forward
        D1 Robot (dog): +X=Forward, +Y=Left, +Z=Up

        Mapping:
        - Robot X (forward) = Quest Z
        - Robot Y (left) = Quest X
        - Robot Z (up) = Quest Y
        """

        self.transform_matrix = np.array([
            [0, 0, 1],
            [1, 0, 0],
            [0, 1, 0],
        ])
        self.position_scale = 1.0

        self.frame_rotation = R.from_matrix(self.transform_matrix)
    
    def quaternion_multiply(self, q1, q2):
        """ Multiply 2 Quaternions [x, y, z, w]"""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2

        return np.array([
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2
        ])
    
    def quaternion_inverse(self, q): 
        """Inverse of quaternion [x, y, z, w]"""
        x, y, z, w = q 
        return np.array([-x, -y, -z, w])
    
    def transform_controller_to_robot(self, controller_pos):
        """ Transform controller position to robot coordinate frame """
        robot_pos = self.transform_matrix @ np.array(controller_pos)
        robot_pos = robot_pos * self.position_scale
        return robot_pos
    
    def transform_orientation_to_robot(self, controller_quat):
        """ 
        Transform controller orientation to robot coordinate frame 
        
        Args: 
            controller_quat: [x, y, z, w] in Quest frame 
        
        Returns: 
            [x, y, z, w] in Robot frame 
        """
        controller_rot = R.from_quat(controller_quat)
        robot_rot = self.frame_rotation * controller_rot
        return robot_rot.as_quat()

    def calculate_orientation_offset(self, controller_q, arm_q):
        """ Calculate relative orientation between controller and robotic arm. 
            offset = robot * inverse(controller) 
        """
        controller_inv = self.quaternion_inverse(controller_q)
        offset = self.quaternion_multiply(arm_q, controller_inv)
        return offset
    
    def apply_orientation_offset(self, controller_q):
        if self.orientation_offset is None: 
            return controller_q
        return self.quaternion_multiply(self.orientation_offset, controller_q)
    
    def handle_button_press(self, controller_pos, controller_q): 
        """ When controller button is first pressed """
        print('\n' + "="*50)
        print("BUTTON PRESSED - Initializing Teleop")
        print("="*50) 

        # Request Current Angles from C++ Arm Controller 
        print("Requesting current arm joint angles...")
        current_angles = self.arm_client.request_current_angles()

        if current_angles is None: 
            print("Error: Failed to get current joint angles")
            return False
        self.current_angles_deg = current_angles
        
        print(f"Current arm angles: {[f'{a:.3f}' for a in current_angles]}")

        current_pos, current_q = self.ik_server.forward_kinematics(current_angles)
        
        # Transform controller pos to robot frame
        controller_pos_robot = self.transform_controller_to_robot(controller_pos)
        controller_q_robot = self.transform_orientation_to_robot(controller_q)

        self.position_offset = np.array(current_pos) - controller_pos_robot
        self.orientation_offset = self.calculate_orientation_offset(controller_q_robot, current_q) # switch back to original for easier debug

        print(f"\nPosition offset: [{self.position_offset[0]:.3f}, {self.position_offset[1]:.3f}, {self.position_offset[2]:.3f}]")
        print(f"Orientation offset: [{self.orientation_offset[0]:.3f}, {self.orientation_offset[1]:.3f}, {self.orientation_offset[2]:.3f}, {self.orientation_offset[3]:.3f}]")

        return True
    
    def update(self, controller_pos, controller_q, button_pressed, gripper_close_cmd, gripper_open_cmd, home_pressed):
        """ Main update function - called repeatedly """

        if home_pressed: 
            success = self.arm_client.home_arm()
            if success: 
                print("Homing Robot Arm")
                return True
            else: 
                print("Failed to Home")
                return False

        button_just_pressed = button_pressed and not self.prev_button_state

        if button_just_pressed: 
            success = self.handle_button_press(controller_pos, controller_q)
            if not success:
                print("Failed to initialize teleoperation")
                self.prev_button_state = button_pressed
                return False
            
        if button_pressed and self.position_offset is not None: 
            controller_pos_robot = self.transform_controller_to_robot(controller_pos)
            controller_q_robot = self.transform_orientation_to_robot(controller_q)

            target_pos = controller_pos_robot + self.position_offset
            target_quat = self.apply_orientation_offset(controller_q_robot) # switch back rotation transformation 

            joint_angles = self.ik_server.solve_ik(self.current_angles_deg, target_pos, target_quat)

            if joint_angles is None: 
                print("IK solution failed")
                self.prev_button_state = button_pressed
                return False
            
            if gripper_close_cmd == 1.0: 
                self.gripper_width -= self.GRIPPER_STEP
            if gripper_open_cmd == 1.0:
                self.gripper_width += self.GRIPPER_STEP
            self.gripper_width = np.clip(self.gripper_width, 0.0, 1.0)

            success = self.arm_client.command_angles(joint_angles, self.gripper_width)

            if success:
                print("Commanded joint angles")
            else:
                print("Failed to command angles")

            self.prev_button_state = button_pressed
            return success
        
        # If we want to move gripper without moving arm
        if gripper_close_cmd == 1.0 or gripper_open_cmd == 1.0:
            if gripper_close_cmd == 1.0: 
                self.gripper_width -= self.GRIPPER_STEP
            if gripper_open_cmd == 1.0:
                self.gripper_width += self.GRIPPER_STEP
            self.gripper_width = np.clip(self.gripper_width, 0.0, 1.0)
            success = self.arm_client.command_gripper(self.gripper_width)
            print(f"Commanded Gripper to: {self.gripper_width}")
            
        if not button_pressed and self.prev_button_state:
            print("Button released - pausing teleoperation")
            self.position_offset = None 
            self.orientation_offset = None 

        self.prev_button_state = button_pressed
        return True


class IKServer:
    def __init__(self, urdf_path, verbose=False):
        self.verbose = verbose
        
        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.d1_arm_id  = p.loadURDF(urdf_path)
        self.num_joints = p.getNumJoints(self.d1_arm_id) - 2
        if self.num_joints != 6: 
            print(f"Error: Incorrect number of joints reported. {self.num_joints} instead of 6")
        
        self.joint_indices = []
        for i in range(self.num_joints):
            info = p.getJointInfo(self.d1_arm_id, i)
            self.joint_indices.append(i)
            
            if self.verbose:
                joint_name = info[1].decode('utf-8')
                joint_type = info[2]
                link_name = info[12].decode('utf-8')
                print(f"Joint {i}: {joint_name} (type {joint_type}) (link {link_name})")
        
        self.end_effector_index = self.num_joints - 1

        self.zones = {
            'over_dog': {
                'x_min': -0.20,
                'x_max': 0.30,
                'z_min': 0.15
            }, 
            'front': {
                'x_min': 0.30,
                'x_max': 0.52, 
                'z_min': -0.07
            },
            'back': {
                'x_min': -0.52, 
                'x_max': -0.20,
                'z_min': -0.07
            },
            'sides': {
                'y_threshold': 0.1,
                'z_min': -0.07
            }
        }

        self.global_limits = {
            'x_min': -0.52, 
            'x_max': 0.52,
            'y_min': -0.52,
            'y_max': 0.52,
            'z_max': 0.67
        }

        self.max_reach = 0.7
    
    def get_z_min_for_pos(self, x, y):
        """
        Calculate minimum z position for arm based on current zone
        """
        if x < self.zones['over_dog']['x_max'] and x > self.zones['over_dog']['x_min'] and abs(y) < self.zones['sides']['y_threshold']: 
            return self.zones['over_dog']['z_min']
        else: 
            return self.zones['front']['z_min']
        
    def is_in_workspace(self, position, verbose=True):
        x, y, z = position

        if x < self.global_limits['x_min'] or x > self.global_limits['x_max']: 
            if verbose: print(f"WARNING: Commanded X out of bounds - {x:.3f}")
            return False
        
        if y < self.global_limits['y_min'] or y > self.global_limits['y_max']: 
            if verbose: print(f"WARNING: Commanded Y out of bounds - {y:.3f}")
            return False
        
        min_z = self.get_z_min_for_pos(x, y)
        if z < min_z or z > self.global_limits['z_max']: 
            if verbose: print(f"WARNING: Commanded Z out of bounds - {z:.3f}")
            return False

        distance = np.sqrt(x**2 + y**2 + z**2)
        if distance > self.max_reach: 
            if verbose: print(f"WARNING: Commanded position overextends arm - {distance:.3f} ")
            return False
    
        return True

    def forward_kinematics(self, joint_angles):
        """ Compute end-effector pose from joint angles 

            Returns: 
                position: [x, y, z]
                orientation: [x, y, z, w] 
        """

        for i, angle in enumerate(joint_angles): 
            if i < len(self.joint_indices):
                p.resetJointState(self.d1_arm_id, self.joint_indices[i], math.radians(angle))


        link_state = p.getLinkState(self.d1_arm_id, self.end_effector_index)

        position = link_state[4] 
        orientation = link_state[5] 

        return position, orientation
    
    
    def solve_ik(self, current_angles_deg, target_position, target_orientation): 
        """ Calculate joint angles needed for specific position and orientation """
        lower_limits = np.radians([-135, -90, -90, -135, -90, -135])
        upper_limits = np.radians([135, 90, 90, 135, 90, 135])
        rest_poses = np.radians([0, -90, 90, 90, 0, 90, 0])

        if not self.is_in_workspace(target_position):
            return None

        joint_angles = p.calculateInverseKinematics(
            self.d1_arm_id,
            self.end_effector_index,
            target_position,
            target_orientation,
            lowerLimits=lower_limits, 
            upperLimits=upper_limits,
            jointRanges=[u - l for u, l in zip(upper_limits, lower_limits)],
            restPoses=np.radians(current_angles_deg),
            maxNumIterations=100, 
            residualThreshold=1e-5
        )
        
        return [math.degrees(joint_angles[i]) for i in range(self.num_joints)]

def run_oculus(ik_server, oculus_reader, arm_client, hz=100, verbose=False):
    """ Main operarion loop """

    teleop = TeleopController(ik_server, arm_client)

    print("Starting teleop loop...")
    print("Press button A on controller to start controlling the arm")

    while True: 
        time.sleep(1/hz)

        # Get controller data 
        poses, buttons = oculus_reader.get_transformations_and_buttons()
        
        
        if 'r' in poses:
            pose_matrix = poses['r']
        else:
            continue

        # Get position and orientation from pose matrix
        controller_pos, controller_quat = convert_pose_to_pos_quat(pose_matrix)
        
        # Get button state
        button_pressed = buttons.get('A', False)
        home_button = buttons.get('B', False)
        gripper_close_command = buttons.get('rightTrig')[0]
        gripper_open_command = buttons.get('rightGrip')[0]

        teleop.update(controller_pos, controller_quat, button_pressed, gripper_close_command, gripper_open_command, home_button)

        if verbose:
            print(f"Controller pos: {controller_pos}, Button A: {button_pressed}")

    
def convert_pose_to_pos_quat(T):
    """
    T: 4x4 numpy transform matrix
    returns:
        position: [x, y, z]
        quaternion: [qx, qy, qz, qw]
    """
    # Translation is last column (first 3 rows)
    pos = T[:3, 3]

    # Rotation is upper-left 3×3
    rot_mat = T[:3, :3]

    # Convert to quaternion
    quat = R.from_matrix(rot_mat).as_quat()  
    # SciPy returns quaternions as [x, y, z, w]

    return pos.tolist(), quat.tolist()

if __name__ == "__main__":

    if MODE == NORMAL: 
        print("Initializing IK Server")
        ik_server = IKServer("d1_550_description/urdf/d1_550_description.urdf")

        print("Connecting to C++ Arm Controller")
        arm_client = ArmClient()
        arm_client.connect() 
        
        print("Testing Connection...")
        if not arm_client.ping():
            print("Connection FAILED")
            exit(1)
        print("Connection Successful")

        if not arm_client.home_arm(): 
            print("Failed to Home")
            exit(1) 
        print("Arm Homed")

        oculus_reader = OculusReader()

        try: 
            run_oculus(ik_server, oculus_reader, arm_client)
        except KeyboardInterrupt:
            print("\nShutting down...")
            arm_client.disconnect() 

    elif MODE == DEBUG:
        print('\n' + "="*50)
        print("DEBUG MODE - Quest Coordinate Frame Test")
        print("="*50)
        
        oculus_reader = OculusReader()
        
        print("\nHold the RIGHT controller in front of you, pointing forward")
        print("Press Enter when ready...")
        input()
        
        # for i in range(5):
        while True:
            poses, buttons = oculus_reader.get_transformations_and_buttons()
            if 'r' in poses:
                print(f"Buttons = [{buttons}]")
            time.sleep(0.2)

    elif MODE == 8:
        print("="*50)
        print("PHASE 8: Orientation Control Test")
        print("="*50)
        
        ik_server = IKServer("d1_550_description/urdf/d1_550_description.urdf")
        arm_client = ArmClient()
        arm_client.connect()
        
        # Get current state
        print("\n1. Getting current arm state...")
        current_angles = arm_client.request_current_angles()
        current_pos, current_quat = ik_server.forward_kinematics(current_angles)
        
        print(f"Current position: {current_pos}")
        print(f"Current quaternion: {current_quat}")
        print(f"Current angles: {[f'{a:.1f}' for a in current_angles]}")
        
        for value in [0.0, 0.01, 0.03, 0.065, 0.1, 1.0, 10.0, 65.0]:
            print(f"Sending gripper value: {value}")
            arm_client.command_angles(current_angles[:6], value)
            time.sleep(5)

    elif MODE == 9:
        print("="*50)
        print("PHASE 9: Check URDF Joint Axes")
        print("="*50)
        
        ik_server = IKServer("d1_550_description/urdf/d1_550_description.urdf")
        
        print("\nJoint Axes in URDF:")
        for i in range(6):  # Only arm joints
            info = p.getJointInfo(ik_server.d1_arm_id, i)
            joint_name = info[1].decode('utf-8')
            joint_axis = info[13]  # Joint axis in local frame
            joint_type = info[2]
            
            print(f"Joint {i} ({joint_name:10s}): axis = {joint_axis}")
        
        print("\n" + "="*50)
        print("Expected for a typical 6-DOF arm:")
        print("  Joint 0: (0, 0, 1) - Base rotation (Z-axis)")
        print("  Joint 1: (0, 1, 0) - Shoulder pitch (Y-axis)")
        print("  Joint 2: (0, 1, 0) - Elbow pitch (Y-axis)")
        print("  Joint 3: (0, 1, 0) - Wrist pitch (Y-axis)")
        print("  Joint 4: (0, 0, 1) - Wrist roll (Z-axis)")
        print("  Joint 5: (0, 1, 0) - Wrist yaw (Y-axis)")
        print("OR")
        print("  Joint 5: (0, 0, 1) - Wrist twist (Z-axis) ← This is what we want!")
        print("="*50)