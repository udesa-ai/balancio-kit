import pybullet
import pybullet_data
from pybullet_utils import bullet_client
import time
import os

physicsClient = pybullet.connect(pybullet.GUI)  # or pybullet.DIRECT for non-graphical version
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
pybullet.setGravity(0, 0, -9.81)
planeId = pybullet.loadURDF("plane.urdf")
StartPos = [0, 0, 0.6]
StartOrientation = pybullet.getQuaternionFromEuler([0,0,0])
balancioId = pybullet.loadURDF("/home/agus/Documents/UdeSA/Balancio_V0/Code/Simulation/urdf/balancio_v0.urdf", StartPos, StartOrientation)

# PbClient = bullet_client.BulletClient(connection_mode=pybullet.GUI)

# Get number of robot joints
num_joints = pybullet.getNumJoints(balancioId)
# Revolute joints (a mano). Name to joint index.
joint_name2idx = {'left_wheel': 0,
                  'right_wheel': 1}

Kp = -65
Ki = -2.8
Kd = -2.0
tita_target = 0.0
delta_tita = 0
previous_delta_tita = 0
pid_p = 0
pid_i = 0
pid_d = 0


while True:
    time.sleep(0.005)

    [_, orientation_quat] = pybullet.getBasePositionAndOrientation(bodyUniqueId=balancioId)
    orientation_euler = pybullet.getEulerFromQuaternion(orientation_quat)
    tita = orientation_euler[1]
    delta_tita = tita_target - tita

    pid_p = Kp * delta_tita
    pid_i += Ki * delta_tita
    pid_d = Kd * ((delta_tita - previous_delta_tita) / 0.005)
    omega = pid_p + pid_i + pid_d

    pybullet.setJointMotorControlArray(bodyUniqueId=balancioId,
                                jointIndices=[joint_name2idx['left_wheel'], joint_name2idx['right_wheel']],
                                controlMode=pybullet.VELOCITY_CONTROL,
                                targetVelocities=[omega, omega])

    pybullet.stepSimulation()

# pybullet.disconnect()