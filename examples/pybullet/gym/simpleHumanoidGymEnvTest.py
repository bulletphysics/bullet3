
from pybullet_envs.bullet.simpleHumanoidGymEnv import SimpleHumanoidGymEnv
print ("hello")
environment = SimpleHumanoidGymEnv(renders=True)

environment._p.setGravity(0,0,0)
	
motorsIds=[]
for motor in environment._humanoid.motor_names:
	motorsIds.append(environment._p.addUserDebugParameter(motor,-1,1,0))

while (True):
    
  action=[]
  for motorId in motorsIds:
  	action.append(environment._p.readUserDebugParameter(motorId))
  
  state, reward, done, info = environment.step(action)
  obs = environment.getExtendedObservation()
  print("obs")
  print(obs)
