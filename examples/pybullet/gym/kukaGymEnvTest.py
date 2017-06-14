
from envs.bullet.kukaGymEnv import KukaGymEnv
print ("hello")
environment = KukaGymEnv(renders=True)

  
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
  
