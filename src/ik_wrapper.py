import pybullet as p
import pybullet_data
import socket
import numpy as np
import struct
import math
import signal 
import sys


class IKServer:
    def __init__(self, urdf_path, host='0.0.0.0', port=5555, verbose=True):
        self.host = host
        self.port = port
        self.running = True
        self.verbose = verbose
    
        self.p_id = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        print("Pybullet Initialized")

        self.d1_arm  = p.loadURDF(urdf_path)
        self.num_joints = p.getNumJoints(self.d1_arm) - 2
        if self.num_joints is not 6: 
            print(f"Error: Incorrect number of joints reported. {self.num_joints} instead of 8")
        
        self.joint_indices = []
        for i in range(self.num_joints):
            info = p.getJointInfo(self.d1_arm, i)
            joint_name = info[1].decode('utf-8')
            joint_type = info[2]
            link_name = info[12].decode('utf-8')
            if self.verbose:
                print(f"Joint {i}: {joint_name} (type {joint_type}) (link {link_name})")
        
        self.end_effector_index = self.num_joints - 1
        
        if self.verbose:
            print("IK Server Successfully Initialized")
    
    def solve_ik(self, target_position, target_orientation): 
        if target_orientation is None: 
            joint_angles = p.calculateInverseKinematics(
                self.d1_arm,
                self.end_effector_index,
                target_position,
                maxNumIterations=100, 
                residualThreshold=1e-5
            )
        else:
            joint_angles = p.calculateInverseKinematics(
                self.d1_arm,
                self.end_effector_index,
                target_position,
                target_orientation,
                maxNumIterations=100, 
                residualThreshold=1e-5
            )
        
        return [math.degrees(joint_angles[i]) for i in range(self.num_joints)]

    def handle_client(self, conn, addr): 
        print(f"Connected: {addr}")
        try:
            while self.running:
                conn.settimeout(1.0)
                try:
                    data = conn.recv(1024)
                except socket.timeout:
                    continue

                if not data:
                    break

                command = data[0] 

                if command == 1: 
                    # Send 3 floats for position
                    if len(data) < 13: 
                        print("Error: Not enough data for position ik")
                        continue

                    target_pos = struct.unpack('fff', data[1:13])
                    print(f"IK Request (position): {target_pos}")

                    joint_angles = self.solve_ik(target_pos, None)

                    result_angles = struct.pack('B', len(joint_angles))
                    result_angles += struct.pack(f'{len(joint_angles)}f', *joint_angles)
                    conn.send(result_angles)
                    print(f"Calculated Angles: {[f'{a:.3f}' for a in joint_angles]}")
                
                elif command == 2: 
                    # Send 7 floats for position and orientation
                    if len(data) < 29: 
                        print("Error: Not enough data for position + orientation ik")
                        continue

                    pose_values = struct.unpack('fff', data[1:29])
                    target_pos = pose_values[:3]
                    target_orientation = pose_values[3:7]
                    print(f"IK Request (position): pos={target_pos}, orientation={target_orientation}")

                    joint_angles = self.solve_ik(target_pos, target_orientation)

                    result_pose = struct.pack('B', len(joint_angles))
                    result_angles += struct.pack(f'{len(joint_angles)}f', *joint_angles)
                    conn.send(result_angles)
                    print(f"Calculated Angles: {[f'{a:.3f}' for a in joint_angles]}")
                
                elif command == 3: 
                    conn.send(b"OK")
                    print("Ping received")
                
                else: 
                    print(f"Unknown Command: {command}")

        except Exception as e: 
            print(f"Error handling client: {e}")
        
        finally: 
            conn.close()
            print(f"Disconnected: {addr}")


    def run(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Allows the same port to be quickly reused
       
        try:
            server.bind((self.host, self.port))
            server.listen(5)
            server.settimeout(1.0)

            if self.verbose:
                print(f"\n{'='*50}")
                print(f"IK Server Running on {self.host}:{self.port}")
                print(f"\n{'='*50}")
                print("Waiting for messages...")

            while self.running:
                try:
                    conn, addr = server.accept()
                    self.handle_client(conn, addr)
                except socket.timeout:
                    continue
                except Exception as e: 
                    if self.running: 
                        print(f"Error connecting: {e}")
        except Exception as e: 
            print(f"Server error: {e}")
        finally: 
            server.close()
            p.disconnect(self.p_id) 
            print(f"Server Shut Down")
    
    def shutdown(self, signum, frame):
        print("/n/nShutting Down Server...")
        self.running = False

    
ik_server = IKServer("urdf/d1_550_description.urdf")
ik_server.run()
