import pybullet as p
import time
import pybullet_data
import math
import numpy as np

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)

# Load ground plane
planeId = p.loadURDF("plane.urdf")

# Load robot - replace with your URDF
initialOrientation = p.getQuaternionFromEuler([1 / 1000, 0.0, 0.0])
robotId = p.loadURDF("robot.urdf", [0, 0, 1], initialOrientation)

# Get wheel joint indices
leftWheelIndex = 0  # Adjust based on your URDF
rightWheelIndex = 1  # Adjust based on your URDF

# Control parameters
targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -100, 100, 0)
maxForceSlider = p.addUserDebugParameter("maxForce", 0, 20, 10)

# Main simulation loop
while True:
    # Read user inputs
    maxForce = p.readUserDebugParameter(maxForceSlider)
    targetVelocity = p.readUserDebugParameter(targetVelocitySlider)

    # Get robot state
    robotPos, robotOrn = p.getBasePositionAndOrientation(robotId)
    robotEuler = p.getEulerFromQuaternion(robotOrn)
    pitch = robotEuler[1]

    # Simple P controller for balance
    kp = 0  # Proportional gain
    balanceControl = kp * pitch
    print(f"pitch: {robotEuler[0]*180/math.pi:.2f} degrees")
    # Apply wheel controls
    p.setJointMotorControl2(
        robotId,
        leftWheelIndex,
        p.VELOCITY_CONTROL,
        targetVelocity=targetVelocity + balanceControl,
        force=maxForce,
    )
    p.setJointMotorControl2(
        robotId,
        rightWheelIndex,
        p.VELOCITY_CONTROL,
        targetVelocity=-targetVelocity + balanceControl,
        force=maxForce,
    )

    time.sleep(0.01)
