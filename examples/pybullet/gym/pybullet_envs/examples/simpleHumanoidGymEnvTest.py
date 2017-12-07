#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)

from pybullet_envs.bullet.simpleHumanoidGymEnv import SimpleHumanoidGymEnv

def main():

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

if __name__=="__main__":
    main()