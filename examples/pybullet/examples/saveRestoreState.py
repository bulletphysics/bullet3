import pybullet as p
import math, time
import difflib,sys

numSteps = 500
numSteps2 = 3
p.connect(p.GUI, options="--width=1024 --height=768")
numObjects = 10
verbose = 0

def setupWorld():
	p.resetSimulation()
	p.loadURDF("planeMesh.urdf")
	for i in range (numObjects):
		cube = p.loadURDF("cube_small.urdf",0,0,(i+1)*0.5)
	p.stepSimulation()
	p.setGravity(0,0,-10)


def dumpStateToFile(file):
	for i in  range (p.getNumBodies()):
		pos,orn = p.getBasePositionAndOrientation(i)
		linVel,angVel = p.getBaseVelocity(i)
		txtPos = "pos="+str(pos)+"\n"
		txtOrn = "orn="+str(orn)+"\n"
		txtLinVel = "linVel"+str(linVel)+"\n"
		txtAngVel = "angVel"+str(angVel)+"\n"
		file.write(txtPos)
		file.write(txtOrn)
		file.write(txtLinVel)
		file.write(txtAngVel)
		
def compareFiles(file1,file2):
	diff = difflib.unified_diff(
            file1.readlines(),
            file2.readlines(),
            fromfile='saveFile.txt',
            tofile='restoreFile.txt',
        )
	for line in diff:
		sys.stdout.write(line)
	
setupWorld()
for i in range (numSteps):
	p.stepSimulation()	
p.saveBullet("state.bullet")
if verbose:
	p.setInternalSimFlags(1)
p.stepSimulation()
if verbose:
	p.setInternalSimFlags(0)
	print("contact points=")
	for q in p.getContactPoints():
		print(q)

for i in range (numSteps2):
	p.stepSimulation()


file = open("saveFile.txt","w") 
dumpStateToFile(file)
file.close() 

#################################
setupWorld()
p.restoreState(fileName="state.bullet")
if verbose:
	p.setInternalSimFlags(1)
p.stepSimulation()
if verbose:
	p.setInternalSimFlags(0)
	print("contact points restored=")
	for q in p.getContactPoints():
		print(q)
for i in range (numSteps2):
	p.stepSimulation()

	
file = open("restoreFile.txt","w")
dumpStateToFile(file)
file.close() 

file1 = open("saveFile.txt","r") 
file2 = open("restoreFile.txt","r") 
#file3 = open("saveFile.txt","r")#saveFileReorder.txt","r") 

compareFiles(file1,file2)
#compareFiles(file1,file3)
            

while (p.getConnectionInfo()["isConnected"]):
	time.sleep(1)

file1.close()
file2.close()