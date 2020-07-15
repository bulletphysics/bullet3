import pybullet as p
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useMaximalCoordinates = False
p.loadURDF("plane.urdf", useMaximalCoordinates=useMaximalCoordinates)
#p.loadURDF("sphere2.urdf",[0,0,1])
p.loadURDF("cube.urdf", [0, 0, 1], useMaximalCoordinates=useMaximalCoordinates)
p.setGravity(0, 3, -10)
while (1):
  p.stepSimulation()
  pts = p.getContactPoints()

  print("num pts=", len(pts))
  totalNormalForce = 0
  totalFrictionForce = [0, 0, 0]
  totalLateralFrictionForce = [0, 0, 0]
  for pt in pts:
    #print("pt.normal=",pt[7])
    #print("pt.normalForce=",pt[9])
    totalNormalForce += pt[9]
    #print("pt.lateralFrictionA=",pt[10])
    #print("pt.lateralFrictionADir=",pt[11])
    #print("pt.lateralFrictionB=",pt[12])
    #print("pt.lateralFrictionBDir=",pt[13])
    totalLateralFrictionForce[0] += pt[11][0] * pt[10] + pt[13][0] * pt[12]
    totalLateralFrictionForce[1] += pt[11][1] * pt[10] + pt[13][1] * pt[12]
    totalLateralFrictionForce[2] += pt[11][2] * pt[10] + pt[13][2] * pt[12]

  print("totalNormalForce=", totalNormalForce)
  print("totalLateralFrictionForce=", totalLateralFrictionForce)
