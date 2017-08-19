import pybullet as p
import time
import numpy as np

conid = p.connect(p.SHARED_MEMORY)
if (conid<0):
	p.connect(p.GUI)

p.setInternalSimFlags(0)
p.resetSimulation()


def createPoseMarker(position=np.array([0,0,0]),
                     orientation=np.array([0,0,0,1]),
                     x_color=np.array([1,0,0]),
                     y_color=np.array([0,1,0]),
                     z_color=np.array([0,0,1]),
                     lineLength=0.1,
                     lineWidth=1,
                     lifeTime=0,
                     parentObjectUniqueId=0,
                     parentLinkIndex=0,
                     physicsClientId=0):
    '''Create a pose marker that identifies a position and orientation in space with 3 colored lines.
    '''
    pts = np.array([[0,0,0],[lineLength,0,0],[0,lineLength,0],[0,0,lineLength]])
    rotIdentity = np.array([0,0,0,1])
    po, _ = p.multiplyTransforms(position, orientation, pts[0,:], rotIdentity)
    px, _ = p.multiplyTransforms(position, orientation, pts[1,:], rotIdentity)
    py, _ = p.multiplyTransforms(position, orientation, pts[2,:], rotIdentity)
    pz, _ = p.multiplyTransforms(position, orientation, pts[3,:], rotIdentity)
    p.addUserDebugLine(po, px, x_color, lineWidth, lifeTime, parentObjectUniqueId, parentLinkIndex, physicsClientId)
    p.addUserDebugLine(po, py, y_color, lineWidth, lifeTime, parentObjectUniqueId, parentLinkIndex, physicsClientId)
    p.addUserDebugLine(po, pz, z_color, lineWidth, lifeTime, parentObjectUniqueId, parentLinkIndex, physicsClientId)

p.loadURDF("plane.urdf",useMaximalCoordinates=True)
p.loadURDF("tray/traybox.urdf",useMaximalCoordinates=True)

gravXid = p.addUserDebugParameter("gravityX",-10,10,0)
gravYid = p.addUserDebugParameter("gravityY",-10,10,0)
gravZid = p.addUserDebugParameter("gravityZ",-10,10,-10)
p.setPhysicsEngineParameter(numSolverIterations=10)
p.setPhysicsEngineParameter(contactBreakingThreshold=0.001)
for i in range (2):
    for j in range (2):
        for k in range (2):
            location = np.array([0.02*i,0.02*j,0.2+0.02*k])
            ob = p.loadURDF("sphere_1cm.urdf", location, useMaximalCoordinates=True)
            orientation = p.getQuaternionFromEuler(location * 200)
            createPoseMarker(location, orientation)
p.setGravity(0,0,-10)
p.setRealTimeSimulation(1)
while True:
    gravX = p.readUserDebugParameter(gravXid)
    gravY = p.readUserDebugParameter(gravYid)
    gravZ = p.readUserDebugParameter(gravZid)
    p.setGravity(gravX,gravY,gravZ)
    time.sleep(0.01)

