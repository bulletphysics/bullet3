#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)

from pybullet_envs.bullet.racecarGymEnv import RacecarGymEnv
isDiscrete = False

def main():
	
	environment = RacecarGymEnv(renders=True, isDiscrete=isDiscrete)
	environment.reset()
	
	targetVelocitySlider = environment._p.addUserDebugParameter("wheelVelocity",-1,1,0)
	steeringSlider = environment._p.addUserDebugParameter("steering",-1,1,0)
	
	while (True):
	  targetVelocity = environment._p.readUserDebugParameter(targetVelocitySlider)
	  steeringAngle = environment._p.readUserDebugParameter(steeringSlider)
	  if (isDiscrete):
	    discreteAction = 0
	    if (targetVelocity<-0.33):
	      discreteAction=0
	    else:
	      if (targetVelocity>0.33):
	        discreteAction=6
	      else:
	        discreteAction=3
	    if (steeringAngle>-0.17):
	      if (steeringAngle>0.17):
	        discreteAction=discreteAction+2
	      else:
	        discreteAction=discreteAction+1
	    action=discreteAction
	  else:
	    action=[targetVelocity,steeringAngle]
	  state, reward, done, info = environment.step(action)
	  obs = environment.getExtendedObservation()
	  print("obs")
	  print(obs)

if __name__=="__main__":
    main()