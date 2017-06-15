
from envs.bullet.kukaGymEnv import KukaGymEnv
import time


environment = KukaGymEnv(renders=True)

  
motorsIds=[]
motorsIds.append(environment._p.addUserDebugParameter("posX",0.4,0.75,0.537))
motorsIds.append(environment._p.addUserDebugParameter("posY",-.22,.3,0.0))
motorsIds.append(environment._p.addUserDebugParameter("posZ",0.1,1,0.2))
motorsIds.append(environment._p.addUserDebugParameter("yaw",-3.14,3.14,0))
motorsIds.append(environment._p.addUserDebugParameter("fingerAngle",0,0.3,.3))

while (True):
    
  action=[]
  for motorId in motorsIds:
    action.append(environment._p.readUserDebugParameter(motorId))
  
  state, reward, done, info = environment.step(action)
  obs = environment.getExtendedObservation()
  
