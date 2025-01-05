import pybullet as p
import time
import pybullet_data
import math
import numpy as np

dt = 0.01


def clamp(value, min, max):
    if value < min:
        return min
    elif value > max:
        return max
    return value


class PID:
    def __init__(self, k_p, k_i, k_d, dt):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.integral = 0
        self.dt = dt
        self.prev_error = 0

    def get_signal(self, value, target):
        error = value - target
        # Proportion
        p_out = self.k_p * error
        # Integral
        self.integral += error
        i_out = self.k_i * self.integral
        # Derivative
        derivative = error - self.prev_error
        d_out = self.k_d * derivative

        # Clamp output
        output = p_out + i_out + d_out
        output = clamp(output, -255, 255)

        self.prev_error = error

        return output


# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)

# Load ground plane
planeId = p.loadURDF("plane.urdf")

# Load robot - replace with your URDF
initialOrientation = p.getQuaternionFromEuler([1 / 100, 0.0, 0])
robotId = p.loadURDF("robot.urdf", [0, 0, 0.2], initialOrientation)

# Get wheel joint indices
leftWheelIndex = 0  # Adjust based on your URDF
rightWheelIndex = 1  # Adjust based on your URDF

# Control parameters
targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -100, 100, 0)
maxForceSlider = p.addUserDebugParameter("maxForce", 0, 20, 10)

controller = PID(1, 10, 0, dt)
# Main simulation loop
while True:
    # Read user inputs
    maxForce = p.readUserDebugParameter(maxForceSlider)
    targetVelocity = p.readUserDebugParameter(targetVelocitySlider)

    # Get robot state
    robotPos, robotOrn = p.getBasePositionAndOrientation(robotId)
    robotEuler = p.getEulerFromQuaternion(robotOrn)
    pitch = robotEuler[0] * 180 / math.pi

    # Simple P controller for balance
    balanceControl = controller.get_signal(pitch, 0.0)
    analogSignal = clamp(balanceControl, -255, 255)
    percentage = analogSignal / 255
    velocity = percentage * 33.16
    # Apply wheel controls
    p.setJointMotorControl2(
        robotId,
        leftWheelIndex,
        p.VELOCITY_CONTROL,
        targetVelocity=-velocity,
        force=maxForce,
    )
    p.setJointMotorControl2(
        robotId,
        rightWheelIndex,
        p.VELOCITY_CONTROL,
        targetVelocity=velocity,
        force=maxForce,
    )

    time.sleep(dt)
