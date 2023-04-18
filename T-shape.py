import pybullet as p
import time
import pandas as pd
import pybullet_data
from utils import State
from utils import read_plan_from_file

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

planeId = p.loadURDF("plane.urdf")
# Create a floor and walls
# floor = p.createCollisionShape(p.GEOM_PLANE)
# walls = p.createCollisionShape(p.GEOM_BOX, halfExtents=[2, 2, 0.1])
# p.createMultiBody(baseCollisionShapeIndex=walls, basePosition=[0, 0, 0.5])

# Drop car into environment
startPos = [0,0,0.5]
startOrientation = p.getQuaternionFromEuler([0,0,0])
car = p.loadURDF("car.urdf",startPos, startOrientation)

# Create a box object
# startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
# Middle Right Line
box = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 1.5, 0.2])
box_body_1 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box, 
                    basePosition=[-1, -2, 0.2])


# Middle Left Line
box_body_1 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box, 
                    basePosition=[1, -2, 0.2])

# Right down curve
box_curve1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.2])
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[1.1, -0.4, 0.2])
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[1.2, -0.2, 0.2])
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[1.3, 0.0, 0.2])
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[1.4, 0.2, 0.2])

# Left Down Curve
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[-1.1, -0.4, 0.2])
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[-1.2, -0.2, 0.2])
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[-1.3, 0.0, 0.2])
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[-1.4, 0.2, 0.2])

box2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1.5, 0.1, 0.2])
# Up right Line
box_body_1 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box2, 
                    basePosition=[3.0, 0.2, 0.2])


# Middle Left Line
box_body_1 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box2, 
                    basePosition=[-3.0, 0.2, 0.2])

box3 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[4.2, 0.1, 0.2])
box_body_1 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box3, 
                    basePosition=[0, 1.5, 0.2])


# box_body_2 = p.createMultiBody(
#                     baseMass=1,
#                     baseInertialFramePosition=[0, 0, 0],
#                     baseCollisionShapeIndex=box_curve1, 
#                     basePosition=[0.4, 2.2, 0.2])
# box_body_2 = p.createMultiBody(
#                     baseMass=1,
#                     baseInertialFramePosition=[0, 0, 0],
#                     baseCollisionShapeIndex=box_curve1, 
#                     basePosition=[0.2, 2.3, 0.2])

# # Left Right Curve
# box_body_2 = p.createMultiBody(
#                     baseMass=1,
#                     baseInertialFramePosition=[0, 0, 0],
#                     baseCollisionShapeIndex=box_curve1, 
#                     basePosition=[-0.2, 1, 0.2])
# box_body_2 = p.createMultiBody(
#                     baseMass=1,
#                     baseInertialFramePosition=[0, 0, 0],
#                     baseCollisionShapeIndex=box_curve1, 
#                     basePosition=[-0.4, 1.1, 0.2])

# # Up Left and Right Straight Lines
# box_horizontal = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1, 0.1, 0.2])
# box_body_1 = p.createMultiBody(
#                     baseMass=1,
#                     baseInertialFramePosition=[0, 0, 0],
#                     baseCollisionShapeIndex=box_horizontal, 
#                     basePosition=[-1.5, 1.1, 0.2])
# box_body_1 = p.createMultiBody(
#                     baseMass=1,
#                     baseInertialFramePosition=[0, 0, 0],
#                     baseCollisionShapeIndex=box_horizontal, 
#                     basePosition=[-0.9, 2.3, 0.2])
# # Right down curve
# box_body_2 = p.createMultiBody(
#                     baseMass=1,
#                     baseInertialFramePosition=[0, 0, 0],
#                     baseCollisionShapeIndex=box_curve1, 
#                     basePosition=[1.1, -0.1, 0.2])
# box_body_2 = p.createMultiBody(
#                     baseMass=1,
#                     baseInertialFramePosition=[0, 0, 0],
#                     baseCollisionShapeIndex=box_curve1, 
#                     basePosition=[1.2, -0.3, 0.2])
# box_body_2 = p.createMultiBody(
#                     baseMass=1,
#                     baseInertialFramePosition=[0, 0, 0],
#                     baseCollisionShapeIndex=box_curve1, 
#                     basePosition=[1.3, -0.5, 0.2])

# # Left down curve
# box_body_2 = p.createMultiBody(
#                     baseMass=1,
#                     baseInertialFramePosition=[0, 0, 0],
#                     baseCollisionShapeIndex=box_curve1, 
#                     basePosition=[0.1, -1.1, 0.2])
# box_body_2 = p.createMultiBody(
#                     baseMass=1,
#                     baseInertialFramePosition=[0, 0, 0],
#                     baseCollisionShapeIndex=box_curve1, 
#                     basePosition=[0.2, -1.3, 0.2])
# box_body_2 = p.createMultiBody(
#                     baseMass=1,
#                     baseInertialFramePosition=[0, 0, 0],
#                     baseCollisionShapeIndex=box_curve1, 
#                     basePosition=[0.3, -1.5, 0.2])

# # Down Left and Right Straight Lines
# box_horizontal = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1, 0.1, 0.2])
# box_horizontal_add = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1.5, 0.1, 0.2])
# box_body_1 = p.createMultiBody(
#                     baseMass=1,
#                     baseInertialFramePosition=[0, 0, 0],
#                     baseCollisionShapeIndex=box_horizontal_add, 
#                     basePosition=[1.9, -1.5, 0.2])
# box_body_1 = p.createMultiBody(
#                     baseMass=1,
#                     baseInertialFramePosition=[0, 0, 0],
#                     baseCollisionShapeIndex=box_horizontal, 
#                     basePosition=[2.4, -0.5, 0.2])

# Read input from Planner:
target_states = read_plan_from_file('testControl.txt')

# Run the simulation
while(True):
    for current_state in target_states:
        for i in range (p.getNumJoints(car)):
            jointInfo = p.getJointInfo(car, i)
            jointName = jointInfo[1]

            # Set steering angle
            if "bar_joint" in jointName.decode("utf-8"):
                p.setJointMotorControl2(car, i, p.POSITION_CONTROL,targetPosition=current_state.steering_angle)
            
            # Set velocity
            if "wheel_joint" in jointName.decode("utf-8"):
                p.setJointMotorControl2(car, i, p.VELOCITY_CONTROL, targetVelocity=current_state.velocity, force=100)

            # Don't think we can Set orientation
            #if "base_scan_joint" in jointName.decode("utf-8"):
                #if "steer_joint" in jointName.decode("utf-8"):
            #    p.setJointMotorControl2(car, i, p.POSITION_CONTROL,targetPosition=current_state.orientation)

            p.stepSimulation()
        #time.sleep(.5)

    # Disconnect from the simulation environment
    p.disconnect()
