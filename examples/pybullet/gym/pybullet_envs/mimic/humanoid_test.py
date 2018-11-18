import os,  inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)

from pybullet_envs.deep_mimic.humanoid import Humanoid
from pybullet_utils.bullet_client import BulletClient
from pybullet_envs.deep_mimic.motion_capture_data import MotionCaptureData
import pybullet_data
import pybullet
import time
import random

bc = BulletClient(connection_mode=pybullet.GUI)
bc.setAdditionalSearchPath(pybullet_data.getDataPath())
bc.configureDebugVisualizer(bc.COV_ENABLE_Y_AXIS_UP,1)
bc.setGravity(0,-9.8,0)
motion=MotionCaptureData()

motionPath = pybullet_data.getDataPath()+"/motions/humanoid3d_backflip.txt"#humanoid3d_spinkick.txt"#humanoid3d_backflip.txt"
motion.Load(motionPath)
print("numFrames = ", motion.NumFrames())
frameTimeId= bc.addUserDebugParameter("frameTime",0,motion.NumFrames()-1.1,0)

y2zOrn = bc.getQuaternionFromEuler([-1.57,0,0])
bc.loadURDF("plane.urdf",[0,-0.08,0], y2zOrn)

humanoid = Humanoid(bc, motion,[0,0,0])

frameTime = 0
keyFrameDuration = motion.KeyFrameDuraction()
print("keyFrameDuration=",keyFrameDuration)
#for i in range (50):
# bc.stepSimulation()

stage = 0





def Reset(humanoid):
	global frameTime
	humanoid.Reset()
	frameTime = 0#random.randint(0,motion.NumFrames()-2)
	print("RESET frametime=",frameTime)
	humanoid.SetFrameTime(frameTime)
	pose = humanoid.InitializePoseFromMotionData()
	humanoid.ApplyPose(pose, True)


Reset(humanoid)
bc.stepSimulation()

while (1):
  #frameTime = bc.readUserDebugParameter(frameTimeId)
  #print("keyFrameDuration=",keyFrameDuration)
  dt = (1./240.)/keyFrameDuration
  #print("dt=",dt)
  frameTime += dt
  if (frameTime >= (motion.NumFrames())-1.1):
    frameTime = motion.NumFrames()-1.1
  #print("frameTime=", frameTime)
  humanoid.SetFrameTime(frameTime)

  pose = humanoid.InitializePoseFromMotionData()

  #humanoid.ApplyPose(pose, False)#False, False)
  if (humanoid.Terminates()):
  	Reset(humanoid)
  	
  bc.stepSimulation()
  
  
  time.sleep(1./240.)

