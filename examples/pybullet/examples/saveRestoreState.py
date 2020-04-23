import pybullet as p
import math, time
import difflib, sys
import pybullet_data


numSteps = 500
numSteps2 = 30
p.connect(p.GUI, options="--width=1024 --height=768")
numObjects = 50
verbose = 0

p.setAdditionalSearchPath(pybullet_data.getDataPath())
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "saveRestoreTimings.log")


def setupWorld():
  p.resetSimulation()
  p.setPhysicsEngineParameter(deterministicOverlappingPairs=1)
  p.loadURDF("planeMesh.urdf")
  kukaId = p.loadURDF("kuka_iiwa/model_free_base.urdf", [0, 0, 10])
  for i in range(p.getNumJoints(kukaId)):
    p.setJointMotorControl2(kukaId, i, p.POSITION_CONTROL, force=0)
  for i in range(numObjects):
    cube = p.loadURDF("cube_small.urdf", [0, i * 0.02, (i + 1) * 0.2])
    #p.changeDynamics(cube,-1,mass=100)
  p.stepSimulation()
  p.setGravity(0, 0, -10)


def dumpStateToFile(file):
  for i in range(p.getNumBodies()):
    pos, orn = p.getBasePositionAndOrientation(i)
    linVel, angVel = p.getBaseVelocity(i)
    txtPos = "pos=" + str(pos) + "\n"
    txtOrn = "orn=" + str(orn) + "\n"
    txtLinVel = "linVel" + str(linVel) + "\n"
    txtAngVel = "angVel" + str(angVel) + "\n"
    file.write(txtPos)
    file.write(txtOrn)
    file.write(txtLinVel)
    file.write(txtAngVel)


def compareFiles(file1, file2):
  diff = difflib.unified_diff(
      file1.readlines(),
      file2.readlines(),
      fromfile='saveFile.txt',
      tofile='restoreFile.txt',
  )
  numDifferences = 0
  for line in diff:
    numDifferences = numDifferences + 1
    sys.stdout.write(line)
  if (numDifferences > 0):
    print("Error:", numDifferences, " lines are different between files.")
  else:
    print("OK, files are identical")


setupWorld()
for i in range(numSteps):
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

for i in range(numSteps2):
  p.stepSimulation()

file = open("saveFile.txt", "w")
dumpStateToFile(file)
file.close()

#################################
setupWorld()

#both restore from file or from in-memory state should work
p.restoreState(fileName="state.bullet")
stateId = p.saveState()
print("stateId=", stateId)
p.removeState(stateId)
stateId = p.saveState()
print("stateId=", stateId)

if verbose:
  p.setInternalSimFlags(1)
p.stepSimulation()
if verbose:
  p.setInternalSimFlags(0)
  print("contact points restored=")
  for q in p.getContactPoints():
    print(q)
for i in range(numSteps2):
  p.stepSimulation()

file = open("restoreFile.txt", "w")
dumpStateToFile(file)
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
for i in range(numSteps2):
  p.stepSimulation()

file = open("restoreFile2.txt", "w")
dumpStateToFile(file)
file.close()

file1 = open("saveFile.txt", "r")
file2 = open("restoreFile.txt", "r")
compareFiles(file1, file2)
file1.close()
file2.close()

file1 = open("saveFile.txt", "r")
file2 = open("restoreFile2.txt", "r")
compareFiles(file1, file2)
file1.close()
file2.close()

p.stopStateLogging(logId)

#while (p.getConnectionInfo()["isConnected"]):
#	time.sleep(1)
