import pybullet as p
import pybullet_data

p.connect(p.SHARED_MEMORY)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
objects = [
    p.loadURDF("plane.urdf", 0.000000, 0.000000, -.300000, 0.000000, 0.000000, 0.000000, 1.000000)
]
objects = [
    p.loadURDF("quadruped/minitaur.urdf", [-0.000046, -0.000068, 0.200774],
               [-0.000701, 0.000387, -0.000252, 1.000000],
               useFixedBase=False)
]
ob = objects[0]
jointPositions = [
    0.000000, 1.531256, 0.000000, -2.240112, 1.527979, 0.000000, -2.240646, 1.533105, 0.000000,
    -2.238254, 1.530335, 0.000000, -2.238298, 0.000000, -1.528038, 0.000000, 2.242656, -1.525193,
    0.000000, 2.244008, -1.530011, 0.000000, 2.240683, -1.528687, 0.000000, 2.240517
]
for ji in range(p.getNumJoints(ob)):
  p.resetJointState(ob, ji, jointPositions[ji])
  p.setJointMotorControl2(bodyIndex=ob, jointIndex=ji, controlMode=p.VELOCITY_CONTROL, force=0)

cid0 = p.createConstraint(1, 3, 1, 6, p.JOINT_POINT2POINT, [0.000000, 0.000000, 0.000000],
                          [0.000000, 0.005000, 0.200000], [0.000000, 0.010000, 0.200000],
                          [0.000000, 0.000000, 0.000000, 1.000000],
                          [0.000000, 0.000000, 0.000000, 1.000000])
p.changeConstraint(cid0, maxForce=500.000000)
cid1 = p.createConstraint(1, 16, 1, 19, p.JOINT_POINT2POINT, [0.000000, 0.000000, 0.000000],
                          [0.000000, 0.005000, 0.200000], [0.000000, 0.010000, 0.200000],
                          [0.000000, 0.000000, 0.000000, 1.000000],
                          [0.000000, 0.000000, 0.000000, 1.000000])
p.changeConstraint(cid1, maxForce=500.000000)
cid2 = p.createConstraint(1, 9, 1, 12, p.JOINT_POINT2POINT, [0.000000, 0.000000, 0.000000],
                          [0.000000, 0.005000, 0.200000], [0.000000, 0.010000, 0.200000],
                          [0.000000, 0.000000, 0.000000, 1.000000],
                          [0.000000, 0.000000, 0.000000, 1.000000])
p.changeConstraint(cid2, maxForce=500.000000)
cid3 = p.createConstraint(1, 22, 1, 25, p.JOINT_POINT2POINT, [0.000000, 0.000000, 0.000000],
                          [0.000000, 0.005000, 0.200000], [0.000000, 0.010000, 0.200000],
                          [0.000000, 0.000000, 0.000000, 1.000000],
                          [0.000000, 0.000000, 0.000000, 1.000000])
p.changeConstraint(cid3, maxForce=500.000000)
p.setGravity(0.000000, 0.000000, 0.000000)
p.stepSimulation()
p.disconnect()
