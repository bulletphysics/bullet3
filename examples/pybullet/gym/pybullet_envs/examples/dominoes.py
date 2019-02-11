import pybullet_data as pd
import pybullet_utils as pu
import pybullet
import pybullet_utils.bullet_client as bc
import time

p = bc.BulletClient(connection_mode=pybullet.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
p.loadURDF("plane_transparent.urdf", useMaximalCoordinates=True)
p#.setPhysicsEngineParameter(numSolverIterations=10, fixedTimeStep=0.01)


p.configureDebugVisualizer(p.COV_ENABLE_PLANAR_REFLECTION,1)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)

y2z = p.getQuaternionFromEuler([0,0,1.57])
meshScale = [1,1,1]
visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,fileName="domino/domino.obj", rgbaColor=[1,1,1,1], specularColor=[0.4,.4,0], visualFrameOrientation=y2z, meshScale=meshScale)

boxDimensions = [0.5*0.00635, 0.5*0.0254, 0.5*0.0508]
collisionShapeId = p.createCollisionShape(p.GEOM_BOX,halfExtents=boxDimensions)


for j in range (12):
  print("j=",j)
  for i in range (35):
    #p.loadURDF("domino/domino.urdf",[i*0.04,0, 0.06])
    p.createMultiBody(baseMass=0.025,baseCollisionShapeIndex = collisionShapeId,baseVisualShapeIndex = visualShapeId, basePosition = [i*0.04,j*0.05, 0.06], useMaximalCoordinates=True)
  
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)

p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(1)
while (1):
  p.setGravity(0,0,-9.8)
  #p.stepSimulation(1./100.)
  time.sleep(1./240.)