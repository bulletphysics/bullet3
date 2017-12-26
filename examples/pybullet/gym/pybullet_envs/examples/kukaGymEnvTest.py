#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)

from pybullet_envs.bullet.kukaGymEnv import KukaGymEnv
import time

def main():

	environment = KukaGymEnv(renders=True,isDiscrete=False, maxSteps = 10000000)
	
	  
	motorsIds=[]
	#motorsIds.append(environment._p.addUserDebugParameter("posX",0.4,0.75,0.537))
	#motorsIds.append(environment._p.addUserDebugParameter("posY",-.22,.3,0.0))
	#motorsIds.append(environment._p.addUserDebugParameter("posZ",0.1,1,0.2))
	#motorsIds.append(environment._p.addUserDebugParameter("yaw",-3.14,3.14,0))
	#motorsIds.append(environment._p.addUserDebugParameter("fingerAngle",0,0.3,.3))
	
	dv = 0.01 
	motorsIds.append(environment._p.addUserDebugParameter("posX",-dv,dv,0))
	motorsIds.append(environment._p.addUserDebugParameter("posY",-dv,dv,0))
	motorsIds.append(environment._p.addUserDebugParameter("posZ",-dv,dv,0))
	motorsIds.append(environment._p.addUserDebugParameter("yaw",-dv,dv,0))
	motorsIds.append(environment._p.addUserDebugParameter("fingerAngle",0,0.3,.3))
	
	done = False
	while (not done):
	    
	  action=[]
	  for motorId in motorsIds:
	    action.append(environment._p.readUserDebugParameter(motorId))
	  
	  state, reward, done, info = environment.step2(action)
	  obs = environment.getExtendedObservation()
	  
if __name__=="__main__":
    main()
