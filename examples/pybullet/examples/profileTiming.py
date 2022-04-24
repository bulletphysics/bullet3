import pybullet as p
import time
#you can visualize the timings using Google Chrome, visit about://tracing 
#and load the json file
import pybullet_data

p.connect(p.GUI)
#p.configureDebugVisualizer(p.ENABLE_RENDERING,0)
p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

t = time.time() + 3.1

logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "single_step_no_stepsim_chrome_about_tracing.json")
while (time.time() < t):
  #p.stepSimulation()
  p.submitProfileTiming("pythontest")
  time.sleep(1./240.)
  p.submitProfileTiming("nested")
  for i in range (100):
    p.submitProfileTiming("deep_nested")
    p.submitProfileTiming()
  time.sleep(1./1000.)
  p.submitProfileTiming()
  p.submitProfileTiming()

p.stopStateLogging(logId)
