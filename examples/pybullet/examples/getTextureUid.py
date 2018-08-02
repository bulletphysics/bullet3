import pybullet as p
p.connect(p.GUI)
plane = p.loadURDF("plane.urdf")
visualData = p.getVisualShapeData(plane, p.VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS)
print(visualData)
curTexUid = visualData[0][8]
print(curTexUid)
texUid = p.loadTexture("tex256.png")
print("texUid=",texUid)

p.changeVisualShape(plane,-1,textureUniqueId=texUid)

for i in range (100):
	p.getCameraImage(320,200)
p.changeVisualShape(plane,-1,textureUniqueId=curTexUid)

for i in range (100):
	p.getCameraImage(320,200)

