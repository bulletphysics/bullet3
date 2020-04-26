import pybullet as p
import time

import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, -0.25])
minitaur = p.loadURDF("quadruped/minitaur_single_motor.urdf", useFixedBase=True)
print(p.getNumJoints(minitaur))
p.resetDebugVisualizerCamera(cameraDistance=1,
                             cameraYaw=23.2,
                             cameraPitch=-6.6,
                             cameraTargetPosition=[-0.064, .621, -0.2])
motorJointId = 1
p.setJointMotorControl2(minitaur, motorJointId, p.VELOCITY_CONTROL, targetVelocity=100000, force=0)

p.resetJointState(minitaur, motorJointId, targetValue=0, targetVelocity=1)
angularDampingSlider = p.addUserDebugParameter("angularDamping", 0, 1, 0)
jointFrictionForceSlider = p.addUserDebugParameter("jointFrictionForce", 0, 0.1, 0)

textId = p.addUserDebugText("jointVelocity=0", [0, 0, -0.2])
p.setRealTimeSimulation(1)
while (1):
  frictionForce = p.readUserDebugParameter(jointFrictionForceSlider)
  angularDamping = p.readUserDebugParameter(angularDampingSlider)
  p.setJointMotorControl2(minitaur,
                          motorJointId,
                          p.VELOCITY_CONTROL,
                          targetVelocity=0,
                          force=frictionForce)
  p.changeDynamics(minitaur, motorJointId, linearDamping=0, angularDamping=angularDamping)

  time.sleep(0.01)
  txt = "jointVelocity=" + str(p.getJointState(minitaur, motorJointId)[1])
  prevTextId = textId
  textId = p.addUserDebugText(txt, [0, 0, -0.2])
  p.removeUserDebugItem(prevTextId)
