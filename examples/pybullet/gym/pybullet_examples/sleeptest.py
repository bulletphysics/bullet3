import pybullet as p
import pybullet_data as pd
import time
p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
objects=[]
useMaximalCoordinates=False

if 0:
  collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_SPHERE, radius = 0.1)
  batchPositions = []
  index = 1
  for x in range(2):
    for y in range(1):
      for z in range(1):
        batchPositions.append(arr[index])
        index=index+1
  bodyUids = p.createMultiBody(baseMass=10,
                              baseInertialFramePosition=[0, 0, 0],
                              baseCollisionShapeIndex=collisionShapeId,
                              baseVisualShapeIndex=-1,
                              basePosition=[0, 0, 2],
                              batchPositions=batchPositions,
                              useMaximalCoordinates=useMaximalCoordinates)
  for b in bodyUids:
    objects.append(b)

else:
  for i in range(2):
    for j in range(1):
      for k in range(1):
        ob = p.loadURDF("sphere_1cm.urdf", [0.210050 * i, 0.210050 * j, 1 + 0.210050 * k],globalScaling=20,
                        useMaximalCoordinates=useMaximalCoordinates)
        objects.append(ob)
        p.changeDynamics(ob, -1, activationState=p.ACTIVATION_STATE_ENABLE_SLEEPING, linearDamping=0, angularDamping=0, sleepThreshold=0.05)
        if (i==0):
          p.resetBaseVelocity(ob, [0,0,0], [0,0,0.22])
        
        
timeid = p.addUserDebugText("t=", [0,0,2])
lvelid = p.addUserDebugText("lvel", [0,0,1.8])
avelid = p.addUserDebugText("avel", [0,0,1.6])
t=0
dt=1./240.
while p.isConnected():
  p.stepSimulation()
  t+=dt
  txtid = p.addUserDebugText("t="+str(t), [0,0,2],replaceItemUniqueId=timeid)
  lin, ang = p.getBaseVelocity(ob)
  txtid = p.addUserDebugText("lvel="+"{:.4f}".format(lin[0])+","+"{:.4f}".format(lin[1])+","+"{:.4f}".format(lin[2]), [0,0,1.8],replaceItemUniqueId=lvelid)
  txtid = p.addUserDebugText("avel="+"{:.4f}".format(ang[0])+","+"{:.4f}".format(ang[1])+","+"{:.4f}".format(ang[2]), [0,0,1.6],replaceItemUniqueId=avelid)
  time.sleep(dt)
  