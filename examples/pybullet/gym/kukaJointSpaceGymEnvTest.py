
from pybullet_envs.bullet.kukaGymEnv import KukaGymEnv
import time


environment = KukaGymEnv(renders=True)
environment._kuka.useInverseKinematics=0
  
motorsIds=[]
for i in range (len(environment._kuka.motorNames)):
  motor = environment._kuka.motorNames[i]
  motorJointIndex = environment._kuka.motorIndices[i]
  motorsIds.append(environment._p.addUserDebugParameter(motor,-3,3,environment._kuka.jointPositions[i]))

while (True):
    
  action=[]
  for motorId in motorsIds:
    action.append(environment._p.readUserDebugParameter(motorId))
  
  state, reward, done, info = environment.step(action)
  obs = environment.getExtendedObservation()
  time.sleep(0.01)
