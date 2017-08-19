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
                     text="",
                     xColor=np.array([1,0,0]),
                     yColor=np.array([0,1,0]),
                     zColor=np.array([0,0,1]),
                     textColor=np.array([0,0,0]),
                     lineLength=0.1,
                     lineWidth=1,
                     textSize=1,
                     textPosition=np.array([0,0,0.1]),
                     textOrientation=None,
                     lifeTime=0,
                     parentObjectUniqueId=-1,
                     parentLinkIndex=-1,
                     physicsClientId=0):
    '''Create a pose marker that identifies a position and orientation in space with 3 colored lines.
    '''
    pts = np.array([[0,0,0],[lineLength,0,0],[0,lineLength,0],[0,0,lineLength]])
    rotIdentity = np.array([0,0,0,1])
    po, _ = p.multiplyTransforms(position, orientation, pts[0,:], rotIdentity)
    px, _ = p.multiplyTransforms(position, orientation, pts[1,:], rotIdentity)
    py, _ = p.multiplyTransforms(position, orientation, pts[2,:], rotIdentity)
    pz, _ = p.multiplyTransforms(position, orientation, pts[3,:], rotIdentity)
    p.addUserDebugLine(po, px, xColor, lineWidth, lifeTime, parentObjectUniqueId, parentLinkIndex, physicsClientId)
    p.addUserDebugLine(po, py, yColor, lineWidth, lifeTime, parentObjectUniqueId, parentLinkIndex, physicsClientId)
    p.addUserDebugLine(po, pz, zColor, lineWidth, lifeTime, parentObjectUniqueId, parentLinkIndex, physicsClientId)
    if textOrientation is None:
        textOrientation = orientation
    p.addUserDebugText(text, [0,0,0.1],textColorRGB=textColor,textSize=textSize,
                       parentObjectUniqueId=parentObjectUniqueId,
                       parentLinkIndex=parentLinkIndex,
                       physicsClientId=physicsClientId)

p.loadURDF("plane.urdf",useMaximalCoordinates=True)
p.loadURDF("tray/traybox.urdf",useMaximalCoordinates=True)
kuka = p.loadURDF("kuka_iiwa/model.urdf", basePosition=[0.5, 0.5, 0])

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

createPoseMarker(parentObjectUniqueId=kuka, parentLinkIndex=6,
                 text="tip")

p.setGravity(0,0,-10)
p.setRealTimeSimulation(1)
angle = 0
angleIncrement = 0.01
while True:
    gravX = p.readUserDebugParameter(gravXid)
    gravY = p.readUserDebugParameter(gravYid)
    gravZ = p.readUserDebugParameter(gravZid)
    p.setGravity(gravX, gravY, gravZ)
    p.resetJointState(kuka,2,angle)
    p.resetJointState(kuka,3,angle)
    angle += angleIncrement
    time.sleep(0.001)

