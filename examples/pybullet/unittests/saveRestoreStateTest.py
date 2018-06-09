import unittest
import pybullet as p
import math, time
import difflib,sys

from utils import allclose, dot

class TestPybulletSaveRestoreMethods(unittest.TestCase):

	def setupWorld(self):
		numObjects = 50
		
		
		maximalCoordinates = False

		p.resetSimulation()
		p.setPhysicsEngineParameter(deterministicOverlappingPairs=1)
		p.loadURDF("planeMesh.urdf",useMaximalCoordinates=maximalCoordinates)
		kukaId = p.loadURDF("kuka_iiwa/model_free_base.urdf",[0,0,10], useMaximalCoordinates=maximalCoordinates)
		for i in range (p.getNumJoints(kukaId)):
			p.setJointMotorControl2(kukaId,i,p.POSITION_CONTROL,force=0)
		for i in range (numObjects):
			cube = p.loadURDF("cube_small.urdf",[0,i*0.02,(i+1)*0.2])
			#p.changeDynamics(cube,-1,mass=100)
		p.stepSimulation()
		p.setGravity(0,0,-10)


	def dumpStateToFile(self, file):
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
		
	def compareFiles(self, file1,file2):
		diff = difflib.unified_diff(
	            file1.readlines(),
	            file2.readlines(),
	            fromfile='saveFile.txt',
	            tofile='restoreFile.txt',
	        )
		numDifferences = 0
		for line in diff:
			numDifferences = numDifferences+1
			sys.stdout.write(line)
		self.assertEqual(numDifferences,0)
		

	def testSaveRestoreState(self):
		numSteps = 500
		numSteps2 = 30

		
		verbose = 0
		self.setupWorld()
		

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
		self.dumpStateToFile(file)
		file.close() 

		#################################
		self.setupWorld()

		#both restore from file or from in-memory state should work
		p.restoreState(fileName="state.bullet")
		stateId = p.saveState()

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
		self.dumpStateToFile(file)
		file.close() 

		p.restoreState(stateId)
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

			
		file = open("restoreFile2.txt","w")
		self.dumpStateToFile(file)
		file.close() 

		file1 = open("saveFile.txt","r") 
		file2 = open("restoreFile.txt","r") 
		self.compareFiles(file1,file2)
		file1.close()
		file2.close()

		file1 = open("saveFile.txt","r") 
		file2 = open("restoreFile2.txt","r") 
		self.compareFiles(file1,file2)
		file1.close()
		file2.close()        

		

#while (p.getConnectionInfo()["isConnected"]):
#	time.sleep(1)

if __name__ == '__main__':
	p.connect(p.DIRECT)
	unittest.main()
