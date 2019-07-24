import pybullet as p
cin = p.connect(p.SHARED_MEMORY)
if (cin < 0):
    cin = p.connect(p.GUI)
objects = [p.loadURDF("plane_transparent.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("microtaur.urdf", -0.505314,0.078555,0.228410,-0.013472,-0.004258,0.359966,0.932858)]
ob = objects[0]
jointPositions=[ 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.562331, 0.000000, -0.730446, -1.420420, -1.388973, 0.000000, -0.755517, -1.353138, 0.000000, 0.000000, 0.000000, 0.000000, 1.521366, 0.000000, -0.766233, -1.517965, 1.432633, 0.000000, -0.797550, -1.514139 ]
for jointIndex in range (p.getNumJoints(ob)):
	p.resetJointState(ob,jointIndex,jointPositions[jointIndex])

p.setGravity(0.000000,0.000000,-10.000000)
p.stepSimulation()
p.disconnect()
