import pybullet as p
import struct


def readLogFile(filename, verbose=True):
  f = open(filename, 'rb')

  print('Opened'),
  print(filename)

  keys = f.readline().decode('utf8').rstrip('\n').split(',')
  fmt = f.readline().decode('utf8').rstrip('\n')

  # The byte number of one record
  sz = struct.calcsize(fmt)
  # The type number of one record
  ncols = len(fmt)

  if verbose:
    print('Keys:'),
    print(keys)
    print('Format:'),
    print(fmt)
    print('Size:'),
    print(sz)
    print('Columns:'),
    print(ncols)

  # Read data
  wholeFile = f.read()
  # split by alignment word
  chunks = wholeFile.split(b'\xaa\xbb')
  log = list()
  for chunk in chunks:
    if len(chunk) == sz:
      values = struct.unpack(fmt, chunk)
      record = list()
      for i in range(ncols):
        record.append(values[i])
      log.append(record)

  return log


#clid = p.connect(p.SHARED_MEMORY)
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadSDF("kuka_iiwa/kuka_with_gripper.sdf")
p.loadURDF("tray/tray.urdf", [0, 0, 0])
p.loadURDF("block.urdf", [0, 0, 2])

log = readLogFile("data/block_grasp_log.bin")

recordNum = len(log)
itemNum = len(log[0])
objectNum = p.getNumBodies()

print('record num:'),
print(recordNum)
print('item num:'),
print(itemNum)


def Step(stepIndex):
  for objectId in range(objectNum):
    record = log[stepIndex * objectNum + objectId]
    Id = record[2]
    pos = [record[3], record[4], record[5]]
    orn = [record[6], record[7], record[8], record[9]]
    p.resetBasePositionAndOrientation(Id, pos, orn)
    numJoints = p.getNumJoints(Id)
    for i in range(numJoints):
      jointInfo = p.getJointInfo(Id, i)
      qIndex = jointInfo[3]
      if qIndex > -1:
        p.resetJointState(Id, i, record[qIndex - 7 + 17])


stepIndexId = p.addUserDebugParameter("stepIndex", 0, recordNum / objectNum - 1, 0)

while True:
  stepIndex = int(p.readUserDebugParameter(stepIndexId))
  Step(stepIndex)
  p.stepSimulation()
  Step(stepIndex)
