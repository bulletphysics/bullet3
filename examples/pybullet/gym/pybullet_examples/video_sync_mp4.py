import pybullet as p
import time
import pybullet_data

#Once the video is recorded, you can extract all individual frames using ffmpeg
#mkdir frames
#ffmpeg -i test.mp4 "frames/out-%03d.png"

#by default, PyBullet runs at 240Hz
p.connect(p.GUI, options="--width=320 --height=200 --mp4=\"test.mp4\" --mp4fps=240")

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
p.loadURDF("plane.urdf")

#in 3 seconds, the object travels about 0.5*g*t^2 meter ~ 45 meter.
r2d2 = p.loadURDF("r2d2.urdf",[0,0,45])
#disable linear damping
p.changeDynamics(r2d2,-1, linearDamping=0)
p.setGravity(0,0,-10)

for i in range (3*240):
  txt = "frame "+str(i)
  item = p.addUserDebugText(txt, [0,1,0])
  p.stepSimulation()
  #synchronize the visualizer (rendering frames for the video mp4) with stepSimulation
  p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
  #print("r2d2 vel=", p.getBaseVelocity(r2d2)[0][2])
  p.removeUserDebugItem(item)

p.disconnect()
