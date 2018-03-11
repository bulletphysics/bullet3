
import pybullet as p
import math
import numpy as np



p.connect(p.GUI)
plane = p.loadURDF("plane100.urdf")
cube = p.loadURDF("cube.urdf",[0,0,1])

def getRayFromTo(mouseX,mouseY):
        width, height, viewMat, projMat, cameraUp, camForward, horizon,vertical, _,_,dist, camTarget = p.getDebugVisualizerCamera()
        camPos = [camTarget[0] - dist*camForward[0],camTarget[1] - dist*camForward[1],camTarget[2] - dist*camForward[2]]
        farPlane = 10000
        rayForward = [(camTarget[0]-camPos[0]),(camTarget[1]-camPos[1]),(camTarget[2]-camPos[2])]
        lenFwd = math.sqrt(rayForward[0]*rayForward[0]+rayForward[1]*rayForward[1]+rayForward[2]*rayForward[2])
        invLen = farPlane*1./lenFwd
        rayForward = [invLen*rayForward[0],invLen*rayForward[1],invLen*rayForward[2]]
        rayFrom = camPos
        oneOverWidth = float(1)/float(width)
        oneOverHeight = float(1)/float(height)

        dHor = [horizon[0] * oneOverWidth,horizon[1] * oneOverWidth,horizon[2] * oneOverWidth]
        dVer = [vertical[0] * oneOverHeight,vertical[1] * oneOverHeight,vertical[2] * oneOverHeight]
        rayToCenter=[rayFrom[0]+rayForward[0],rayFrom[1]+rayForward[1],rayFrom[2]+rayForward[2]]
        ortho=[- 0.5 * horizon[0] + 0.5 * vertical[0]+float(mouseX)*dHor[0]-float(mouseY)*dVer[0],
             - 0.5 * horizon[1] + 0.5 * vertical[1]+float(mouseX)*dHor[1]-float(mouseY)*dVer[1],
             - 0.5 * horizon[2] + 0.5 * vertical[2]+float(mouseX)*dHor[2]-float(mouseY)*dVer[2]]

        rayTo = [rayFrom[0]+rayForward[0]  +ortho[0],
                                        rayFrom[1]+rayForward[1]  +ortho[1],
                                        rayFrom[2]+rayForward[2]  +ortho[2]]
        lenOrtho = math.sqrt(ortho[0]*ortho[0]+ortho[1]*ortho[1]+ortho[2]*ortho[2])
        alpha = math.atan(lenOrtho/farPlane)
        return rayFrom,rayTo, alpha


width, height, viewMat, projMat, cameraUp, camForward, horizon,vertical, _,_,dist, camTarget = p.getDebugVisualizerCamera()
camPos = [camTarget[0] - dist*camForward[0],camTarget[1] - dist*camForward[1],camTarget[2] - dist*camForward[2]]
farPlane = 10000
rayForward = [(camTarget[0]-camPos[0]),(camTarget[1]-camPos[1]),(camTarget[2]-camPos[2])]
lenFwd = math.sqrt(rayForward[0]*rayForward[0]+rayForward[1]*rayForward[1]+rayForward[2]*rayForward[2])
oneOverWidth = float(1)/float(width)
oneOverHeight = float(1)/float(height)
dHor = [horizon[0] * oneOverWidth,horizon[1] * oneOverWidth,horizon[2] * oneOverWidth]
dVer = [vertical[0] * oneOverHeight,vertical[1] * oneOverHeight,vertical[2] * oneOverHeight]
        
lendHor = math.sqrt(dHor[0]*dHor[0]+dHor[1]*dHor[1]+dHor[2]*dHor[2]) 
lendVer = math.sqrt(dVer[0]*dVer[0]+dVer[1]*dVer[1]+dVer[2]*dVer[2])

cornersX = [0,width,width,0]
cornersY = [0,0,height,height]
corners3D=[]

imgW = int(width/10)
imgH = int(height/10)

img = p.getCameraImage(imgW,imgH, renderer=p.ER_BULLET_HARDWARE_OPENGL)
rgbBuffer=img[2]
depthBuffer = img[3]
print("rgbBuffer.shape=",rgbBuffer.shape)
print("depthBuffer.shape=",depthBuffer.shape)

#disable rendering temporary makes adding objects faster
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER,0)
visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, rgbaColor=[1,1,1,1], radius=0.03 )
collisionShapeId = -1 #p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="duck_vhacd.obj", collisionFramePosition=shift,meshScale=meshScale)

for i in range (4):
	w = cornersX[i]
	h = cornersY[i]
	rayFrom,rayTo, _= getRayFromTo(w,h)
	rf = np.array(rayFrom)
	rt = np.array(rayTo)
	vec = rt-rf
	l = np.sqrt(np.dot(vec,vec))
	newTo = (0.01/l)*vec+rf
	#print("len vec=",np.sqrt(np.dot(vec,vec)))
			
	p.addUserDebugLine(rayFrom,newTo,[1,0,0])
	corners3D.append(newTo)
count = 0

stepX=5
stepY=5
for w in range(0,imgW,stepX):
	for h in range (0,imgH,stepY):
		count+=1
		if ((count % 100)==0):
			print(count,"out of ", imgW*imgH/(stepX*stepY))
		rayFrom,rayTo, alpha = getRayFromTo(w*(width/imgW),h*(height/imgH))
		rf = np.array(rayFrom)
		rt = np.array(rayTo)
		vec = rt-rf
		l = np.sqrt(np.dot(vec,vec))
		depthImg = float(depthBuffer[h,w])
		far=1000.
		near=0.01
		depth = far * near / (far - (far - near) * depthImg)
		depth/=math.cos(alpha)
		newTo = (depth/l)*vec+rf
		p.addUserDebugLine(rayFrom,newTo,[1,0,0])
		mb = p.createMultiBody(baseMass=0,baseCollisionShapeIndex=collisionShapeId, baseVisualShapeIndex = visualShapeId, basePosition = newTo, useMaximalCoordinates=True)
		color = rgbBuffer[h,w]
		color=[color[0]/255.,color[1]/255.,color[2]/255.,1]
		p.changeVisualShape(mb,-1,rgbaColor=color)
p.addUserDebugLine(corners3D[0],corners3D[1],[1,0,0])
p.addUserDebugLine(corners3D[1],corners3D[2],[1,0,0])
p.addUserDebugLine(corners3D[2],corners3D[3],[1,0,0])
p.addUserDebugLine(corners3D[3],corners3D[0],[1,0,0])
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
print("ready\n")
#p.removeBody(plane)
#p.removeBody(cube)
while (1):
	p.setGravity(0,0,-10)

		


