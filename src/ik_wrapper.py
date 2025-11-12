import pybullet as p
import pybullet_data

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

d1_arm  = p.loadURDF("urdf/d1_550_description.urdf")
end_effector_index = 6

target_pos = [0.3, 1, 0.5]

joint_angles = p.calculateInverseKinematics(
    d1_arm,
    end_effector_index,
    target_pos,
    maxNumIterations=100, 
    residualThreshold=1e-5
)

print(f"Joint angles: {joint_angles[:7]}")