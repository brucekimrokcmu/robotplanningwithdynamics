import pybullet as p
import time
import pybullet_data
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




# Run the simulation
while(1):
    # make car turn right
    p.setJointMotorControl2(car, 0, p.VELOCITY_CONTROL, targetVelocity=2, force=100)   
    # p.setJointMotorControl2(car, 1, p.TORQUE_CONTROL, force=0.5)
    # p.setJointMotorControl2(car, 4, p.VELOCITY_CONTROL, targetVelocity=0, force=100)
    # p.setJointMotorControl2(car, 6, p.VELOCITY_CONTROL, targetVelocity=, force=100)
    # p.setJointMotorControl2(car, 7, p.VELOCITY_CONTROL, targetVelocity=2, force=100)
    # p.setJointMotorControl2(car, 8, p.VELOCITY_CONTROL, targetVelocity=2, force=100)
    p.stepSimulation()
    # time.sleep(1./240.)
# Disconnect from the simulation environment
p.disconnect()