import pybullet as p
import time

p.connect(p.GUI)
p.loadURDF("plane.urdf",useMaximalCoordinates=True)
p.loadURDF("tray/traybox.urdf",useMaximalCoordinates=True)

gravXid = p.addUserDebugParameter("gravityX",-10,10,0)
gravYid = p.addUserDebugParameter("gravityY",-10,10,0)
gravZid = p.addUserDebugParameter("gravityZ",-10,10,-10)
p.setPhysicsEngineParameter(numSolverIterations=10)
p.setPhysicsEngineParameter(contactBreakingThreshold=0.001)
for i in range (10):
    for j in range (10):
        for k in range (5):
            ob = p.loadURDF("sphere_1cm.urdf",[0.02*i,0.02*j,0.2+0.02*k],useMaximalCoordinates=True)
p.setGravity(0,0,-10)
p.setRealTimeSimulation(1)
while True:
    gravX = p.readUserDebugParameter(gravXid)
    gravY = p.readUserDebugParameter(gravYid)
    gravZ = p.readUserDebugParameter(gravZid)
    p.setGravity(gravX,gravY,gravZ)
    time.sleep(0.01)

