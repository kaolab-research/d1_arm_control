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
import threading

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
        

    "{\"seq\":4,\"address\":1,\"funcode\":1,\"data\":{\"id\":5,\"angle\":60,\"delay_ms\":0}}"

    def home_arm(self):
        try: 
            self.sock.send(struct.pack('B', MSG_HOME_ARM))
            time.sleep(1) # Allow time to home the arm
            self.command_gripper(1)
            time.sleep(1)
            self.command_gripper(0)
            time.sleep(1)
            self.command_gripper(1)

            self.gripper_width = 1.0
            return True
        except Exception as e: 
            print(f"Error homing robot arm: {e}")
            return False

class TeleopController:
    def __init__(self, ik_server, arm_client, oculus_reader):
        self.ik_server = ik_server
        self.arm_client = arm_client
        self.oculus_reader = oculus_reader

        # Offset Tracking
        self.position_offset = None
        self.orientation_offset = None
        self.controller_q_init = None
        self.arm_q_init = None

        # Current Pose Tracking 
        self.current_angles_deg = None
        self.gripper_width = 0.0

        # Threshold to Send Command 
        self.pos_threshold = 0.005
        self.rot_threshold = 1.0

        # Button State Tracking
        self.prev_button_state = False
        
        # Shared controller state 
        # Write in thread, Read in loop
        self._state = {
            "poses": {},
            "buttons": {},
            "controller_on": False,
        }
        self._lock = threading.Lock()

        # Coordinate frame transformation
        self.setup_coordinate_transform()
        
        t = threading.Thread(target=self._update_internal_state, daemon=True)
        t.start()

    def _update_internal_state(self, hz=50, timeout_sec=5):
        last_read_time = time.time()
        while True:
            time.sleep(1/hz) 
            poses, buttons = self.oculus_reader.get_transformations_and_buttons()

            with self._lock:
                self._state["controller_on"] = (time.time() - last_read_time) < timeout_sec
            
            if not poses:
                continue

            last_read_time = time.time()
            with self._lock: 
                self._state["poses"] = poses 
                self._state["buttons"] = buttons 
                self._state["controller_on"] = True

    def _get_state(self):
        with self._lock:
            return {
                "poses": dict(self._state["poses"]),
                "buttons": dict(self._state["buttons"]),
                "controller_on": self._state["controller_on"],
            }

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
        self.position_scale = 0.5
        self.rotation_scale = 0.5

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
        
        current_pos, current_q = self.ik_server.forward_kinematics(current_angles)
    
        # Transform controller pos to robot frame
        controller_pos_robot = self.transform_controller_to_robot(controller_pos)
        controller_q_robot = self.transform_orientation_to_robot(controller_q)

        self.controller_q_init = np.array(controller_q_robot)
        self.arm_q_init = np.array(current_q)

        self.position_offset = np.array(current_pos) - controller_pos_robot
        self.orientation_offset = self.calculate_orientation_offset(controller_q_robot, current_q) # switch back to original for easier debug

        print(f"\nPosition offset: [{self.position_offset[0]:.3f}, {self.position_offset[1]:.3f}, {self.position_offset[2]:.3f}]")
        print(f"Orientation offset: [{self.orientation_offset[0]:.3f}, {self.orientation_offset[1]:.3f}, {self.orientation_offset[2]:.3f}, {self.orientation_offset[3]:.3f}]")

        return True
    
    def update(self, controller_pos, controller_q, button_pressed, gripper_close_cmd, home_pressed):
        """ Main update function - called repeatedly """

        # Home arm on request (press button B)
        if home_pressed: 
            success = self.arm_client.home_arm()
            if success: 
                print("Homing Robot Arm")
                return True
            else: 
                print("Failed to Home")
                return False

        # Reset arm state on initial button press 
        button_just_pressed = button_pressed and not self.prev_button_state
        if button_just_pressed: 
            success = self.handle_button_press(controller_pos, controller_q)
            if not success:
                print("Failed to initialize teleoperation")
                self.prev_button_state = button_pressed
                return False
            
        # Button pressed regular operations
        if button_pressed and self.position_offset is not None: 
            current_pos_fk, _ = self.ik_server.forward_kinematics(self.current_angles_deg)
            controller_pos_robot = self.transform_controller_to_robot(controller_pos)
            controller_q_robot = self.transform_orientation_to_robot(controller_q)

            # Calculate target position
            target_pos = controller_pos_robot + self.position_offset
            
            # Compute delta rotation from controller init
            controller_init_inv = R.from_quat(self.controller_q_init).inv()
            delta_rot = R.from_quat(controller_q_robot) * controller_init_inv
            # Extract delta as euler, zero out roll, only apply pitch and yaw
            delta_euler = delta_rot.as_euler('xyz', degrees=True)
            delta_euler[0] = 0 # ignore roll changes
            delta_euler[1] *= self.rotation_scale
            delta_euler[2] *= -self.rotation_scale # flip yaw direction
            delta_rot_filtered = R.from_euler('xyz', delta_euler, degrees=True)

            # Apply filtered delta starting orientation of arm 
            target_rot = delta_rot_filtered * R.from_quat(self.arm_q_init)
            target_quat = target_rot.as_quat()
            
            # Skip command if no significant movement is made
            pos_delta = np.linalg.norm(target_pos - np.array(current_pos_fk))
            rot_delta = np.abs(delta_euler[1]) + np.abs(delta_euler[2])
            if pos_delta < self.pos_threshold and rot_delta < self.rot_threshold:
                self.prev_button_state = button_pressed
                return True

            # Inverse kinematics to get joint angles from position command 
            joint_angles = self.ik_server.solve_ik(self.current_angles_deg, target_pos, target_quat)
            if joint_angles is None: 
                print("IK solution failed")
                self.prev_button_state = button_pressed
                return False

            success = self.arm_client.command_angles(joint_angles, self.gripper_width)
            if success:
                self.current_angles_deg = joint_angles
            else:
                print("Failed to command joint angles")

            self.prev_button_state = button_pressed
            return success
        else:
            # If we want to move gripper without moving arm
            grip_delta = np.abs(gripper_close_cmd - self.gripper_width)
            if grip_delta > 0.1:
                norm_gripper_cmd = np.abs(1 - gripper_close_cmd)
                success = self.arm_client.command_gripper(norm_gripper_cmd)
                if success:
                    self.gripper_width = norm_gripper_cmd
        
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

    teleop = TeleopController(ik_server, arm_client, oculus_reader)

    print("Starting teleop loop...")
    print("Press button A on controller to start controlling the arm")

    while True: 
        time.sleep(1/hz)

        # Get controller data 
        state = teleop._get_state()

        if not state["controller_on"]: 
            print("Controller disconnected, skip ...")
            continue
        
        if 'r' not in state["poses"]:
            continue
            
        pose_matrix = state["poses"]['r']
        buttons = state["buttons"]

        # Get position and orientation from pose matrix
        controller_pos, controller_quat = convert_pose_to_pos_quat(pose_matrix)
        
        # Get button state
        teleop_button = buttons.get('A', False)
        home_button = buttons.get('B', False)
        gripper_close_cmd = buttons.get('rightTrig', 0.0)[0]

        teleop.update(controller_pos, controller_quat, teleop_button, gripper_close_cmd, home_button)

        if verbose:
            print(f"Controller pos: {controller_pos}, Button A: {teleop_button}, Gripper: {gripper_close_cmd}, Home: {home_button}")

    
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

    