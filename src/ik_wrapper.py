import pybullet as p
import pybullet_data
import socket
import numpy as np


class IKServer:
    def __init__(self, urdf_path, host='127.0.0.1', port=5555, verbose=True):
        self.host = host
        self.port = port
        self.running = True
        self.verbose = verbose
    
        self.p_id = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        print("Pybullet Initialized")

        self.d1_arm  = p.loadURDF(urdf_path)
        self.num_joints = p.getNumJoints(self.d1_arm)
        if self.num_joints is not 8: 
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
    
    def solve_ik(): 
        print("Solve Inverse Kinematics for D1 Arm")
    
    def handle_client(): 
        print(f"Handle Client Calls")

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
        
                
target_pos = [3, 1, 10]
ik_server = IKServer("urdf/d1_550_description.urdf")

joint_angles = p.calculateInverseKinematics(
    ik_server.d1_arm,
    ik_server.end_effector_index,
    target_pos,
    maxNumIterations=100, 
    residualThreshold=1e-5
)

ik_server.run()

print(f"Joint angles: {joint_angles[:7]}")