import pybullet as p
p.connect(p.GUI)
r2d2 = p.loadURDF("r2d2.urdf")


aabb = p.getAABB(r2d2)
aabbMin = aabb[0]
aabbMax = aabb[1]

f = [aabbMin[0],aabbMin[1],aabbMin[2]]
t = [aabbMax[0],aabbMin[1],aabbMin[2]]
p.addUserDebugLine(f,t,[1,1,1])
f = [aabbMin[0],aabbMin[1],aabbMin[2]]
t = [aabbMin[0],aabbMax[1],aabbMin[2]]
p.addUserDebugLine(f,t,[1,1,1])
f = [aabbMin[0],aabbMin[1],aabbMin[2]]
t = [aabbMin[0],aabbMin[1],aabbMax[2]]
p.addUserDebugLine(f,t,[1,1,1])

f = [aabbMax[0],aabbMax[1],aabbMax[2]]
t = [aabbMin[0],aabbMax[1],aabbMax[2]]
p.addUserDebugLine(f,t,[1,1,1])
f = [aabbMax[0],aabbMax[1],aabbMax[2]]
t = [aabbMax[0],aabbMin[1],aabbMax[2]]
p.addUserDebugLine(f,t,[1,1,1])
f = [aabbMax[0],aabbMax[1],aabbMax[2]]
t = [aabbMax[0],aabbMax[1],aabbMin[2]]
p.addUserDebugLine(f,t,[1,1,1])

for i in range (p.getNumJoints(r2d2)):
	aabb = p.getAABB(r2d2,i)
	aabbMin = aabb[0]
	aabbMax = aabb[1]

	f = [aabbMin[0],aabbMin[1],aabbMin[2]]
	t = [aabbMax[0],aabbMin[1],aabbMin[2]]
	p.addUserDebugLine(f,t,[1,1,1])
	f = [aabbMin[0],aabbMin[1],aabbMin[2]]
	t = [aabbMin[0],aabbMax[1],aabbMin[2]]
	p.addUserDebugLine(f,t,[1,1,1])
	f = [aabbMin[0],aabbMin[1],aabbMin[2]]
	t = [aabbMin[0],aabbMin[1],aabbMax[2]]
	p.addUserDebugLine(f,t,[1,1,1])

	f = [aabbMax[0],aabbMax[1],aabbMax[2]]
	t = [aabbMin[0],aabbMax[1],aabbMax[2]]
	p.addUserDebugLine(f,t,[1,1,1])
	f = [aabbMax[0],aabbMax[1],aabbMax[2]]
	t = [aabbMax[0],aabbMin[1],aabbMax[2]]
	p.addUserDebugLine(f,t,[1,1,1])
	f = [aabbMax[0],aabbMax[1],aabbMax[2]]
	t = [aabbMax[0],aabbMax[1],aabbMin[2]]
	p.addUserDebugLine(f,t,[1,1,1])

while(1):
	a=0