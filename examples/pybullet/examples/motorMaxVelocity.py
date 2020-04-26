import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
cartpole = p.loadURDF("cartpole.urdf")
p.setRealTimeSimulation(1)
p.setJointMotorControl2(cartpole,
                        1,
                        p.POSITION_CONTROL,
                        targetPosition=1000,
                        targetVelocity=0,
                        force=1000,
                        positionGain=1,
                        velocityGain=0,
                        maxVelocity=0.5)
while (1):
  p.setGravity(0, 0, -10)
  js = p.getJointState(cartpole, 1)
  print("position=", js[0], "velocity=", js[1])
  time.sleep(0.01)
