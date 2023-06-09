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

# Create a box object
# startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
# Middle Right Line
box = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 1, 0.2])
box_body_1 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box, 
                    basePosition=[1, 1, 0.2])


# Middle Left Line
box_body_1 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box, 
                    basePosition=[0, 0, 0.2])

# Right Up curve
box_curve1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.2])
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[0.8, 2, 0.2])
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[0.6, 2.1, 0.2])
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[0.4, 2.2, 0.2])
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[0.2, 2.3, 0.2])

# Left Right Curve
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[-0.2, 1, 0.2])
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[-0.4, 1.1, 0.2])

# Up Left and Right Straight Lines
box_horizontal = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1, 0.1, 0.2])
box_body_1 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_horizontal, 
                    basePosition=[-1.5, 1.1, 0.2])
box_body_1 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_horizontal, 
                    basePosition=[-0.9, 2.3, 0.2])
# Right down curve
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[1.1, -0.1, 0.2])
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[1.2, -0.3, 0.2])
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[1.3, -0.5, 0.2])

# Left down curve
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[0.1, -1.1, 0.2])
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[0.2, -1.3, 0.2])
box_body_2 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_curve1, 
                    basePosition=[0.3, -1.5, 0.2])

# Down Left and Right Straight Lines
box_horizontal = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1, 0.1, 0.2])
box_horizontal_add = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1.5, 0.1, 0.2])
box_body_1 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_horizontal_add, 
                    basePosition=[1.9, -1.5, 0.2])
box_body_1 = p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=box_horizontal, 
                    basePosition=[2.4, -0.5, 0.2])





# Run the simulation
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)
# Disconnect from the simulation environment
p.disconnect()