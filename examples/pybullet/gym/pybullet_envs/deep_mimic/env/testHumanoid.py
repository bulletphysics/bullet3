from pybullet_utils import bullet_client
import time
import math
import motion_capture_data
from pybullet_envs.deep_mimic.env import humanoid_stable_pd
import pybullet_data
import pybullet as p1
import humanoid_pose_interpolator
import numpy as np

pybullet_client =  bullet_client.BulletClient(connection_mode=p1.GUI)

pybullet_client.setAdditionalSearchPath(pybullet_data.getDataPath())
z2y = pybullet_client.getQuaternionFromEuler([-math.pi*0.5,0,0])
#planeId = pybullet_client.loadURDF("plane.urdf",[0,0,0],z2y)
planeId= pybullet_client.loadURDF("plane_implicit.urdf",[0,0,0],z2y, useMaximalCoordinates=True)
pybullet_client.changeDynamics(planeId, linkIndex=-1, lateralFriction=0.9)        
#print("planeId=",planeId)

pybullet_client.configureDebugVisualizer(pybullet_client.COV_ENABLE_Y_AXIS_UP,1)
pybullet_client.setGravity(0,-9.8,0)

pybullet_client.setPhysicsEngineParameter(numSolverIterations=10)


mocapData = motion_capture_data.MotionCaptureData()
#motionPath = pybullet_data.getDataPath()+"/data/motions/humanoid3d_walk.txt"
motionPath = pybullet_data.getDataPath()+"/data/motions/humanoid3d_backflip.txt"
mocapData.Load(motionPath)
timeStep = 1./600
useFixedBase=False
humanoid = humanoid_stable_pd.HumanoidStablePD(pybullet_client, mocapData, timeStep, useFixedBase)
isInitialized = True

pybullet_client.setTimeStep(timeStep)
pybullet_client.setPhysicsEngineParameter(numSubSteps=2)
timeId = pybullet_client.addUserDebugParameter("time",0,10,0)


#startPose = humanoid_pose_interpolator.HumanoidPoseInterpolator()
#startPose.Reset(basePos=[-0.000000,0.889540,0.000000],baseOrn=[0.029215,-0.000525,-0.017963,0.999412],chestRot=[0.000432,0.000572,0.005500,0.999985],
#  neckRot=[0.001660,-0.011168,-0.140597,0.990003],rightHipRot=[-0.024450,-0.000839,-0.038869,0.998945],rightKneeRot=[-0.014186],rightAnkleRot=[0.010757,-0.105223,0.035405,0.993760],
#  rightShoulderRot=[-0.003003,-0.124234,0.073280,0.989539],rightElbowRot=[0.240463],leftHipRot=[-0.020920,-0.012925,-0.006300,0.999678],leftKneeRot=[-0.027859],
#  leftAnkleRot=[-0.010764,0.105284,-0.009276,0.994341],leftShoulderRot=[0.055661,-0.019608,0.098917,0.993344],leftElbowRot=[0.148934],
#  baseLinVel=[-0.340837,0.377742,0.009662],baseAngVel=[0.047057,0.285253,-0.248554],chestVel=[-0.016455,-0.070035,-0.231662],neckVel=[0.072168,0.097898,-0.059063],
#  rightHipVel=[-0.315908,-0.131685,1.114815],rightKneeVel=[0.103419],rightAnkleVel=[-0.409780,-0.099954,-4.241572],rightShoulderVel=[-3.324227,-2.510209,1.834637],
#  rightElbowVel=[-0.212299],leftHipVel=[0.173056,-0.191063,1.226781,0.000000],leftKneeVel=[-0.665322],leftAnkleVel=[0.282716,0.086217,-3.007098,0.000000],
#  leftShoulderVel=[4.253144,2.038637,1.170750],leftElbowVel=[0.387993])
#  
#targetPose = humanoid_pose_interpolator.HumanoidPoseInterpolator()
#targetPose.Reset(basePos=[0.000000,0.000000,0.000000],baseOrn=[0.000000,0.000000,0.000000,1.000000],chestRot=[-0.006711,0.007196,-0.027119,0.999584],neckRot=[-0.017613,-0.033879,-0.209250,0.977116],
#  rightHipRot=[-0.001697,-0.006510,0.046117,0.998913],rightKneeRot=[0.366954],rightAnkleRot=[0.012605,0.001208,-0.187007,0.982277],rightShoulderRot=[-0.468057,-0.044589,0.161134,0.867739],
#  rightElbowRot=[-0.593650],leftHipRot=[0.006993,0.017242,0.049703,0.998591],leftKneeRot=[0.395147],leftAnkleRot=[-0.008922,0.026517,-0.217852,0.975581],
#  leftShoulderRot=[0.426160,-0.266177,0.044672,0.863447],leftElbowRot=[-0.726281])
  
#out_tau= [0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,-33.338211,-1.381748,-118.708975,0.000000,-2.813919,-2.773850,-0.772481,0.000000,31.885372,2.658243,64.988216,0.000000,94.773133,1.784944,6.240010,5.407563,0.000000,-180.441290,-6.821173,-19.502417,0.000000,-44.518261,9.992627,-2.380950,53.057697,0.000000,111.728594,-1.218496,-4.630812,4.268995,0.000000,89.741829,-8.460265,-117.727884,0.000000,-79.481906]
#,mSimWorld->stepSimulation(timestep:0.001667, mParams.mNumSubsteps:2, subtimestep:0.000833)
#cImpPDController::CalcControlForces timestep=0.001667
  
  
def isKeyTriggered(keys, key):
  o = ord(key)
  if o in keys:
    return keys[ord(key)] & pybullet_client.KEY_WAS_TRIGGERED
  return False
  
animating = False
singleStep = False

#humanoid.initializePose(pose=startPose, phys_model = humanoid._sim_model, initBase=True, initializeVelocity=True)
t=0
while (1):

  keys = pybullet_client.getKeyboardEvents()
  #print(keys)
  if isKeyTriggered(keys, ' '):
      animating = not animating

  if isKeyTriggered(keys, 'b'):
      singleStep = True
  
  if animating or singleStep:
    
    
    singleStep = False
    #t = pybullet_client.readUserDebugParameter(timeId)
    #print("t=",t)
    for i in range (1):

      print("t=",t)
      humanoid.setSimTime(t)
        
      humanoid.computePose(humanoid._frameFraction)
      pose = humanoid._poseInterpolator
      #humanoid.initializePose(pose=pose, phys_model = humanoid._sim_model, initBase=True, initializeVelocity=True)
      #humanoid.resetPose()
      
      
      desiredPose = humanoid.computePose(humanoid._frameFraction)
      #desiredPose = desiredPose.GetPose() 
      #curPose = HumanoidPoseInterpolator()
      #curPose.reset()
      s = humanoid.getState()
      #np.savetxt("pb_record_state_s.csv", s, delimiter=",")
      maxForces = [0,0,0,0,0,0,0,200,200,200,200, 50,50,50,50, 200,200,200,200, 150, 90,90,90,90, 100,100,100,100, 60, 200,200,200,200,  150, 90, 90, 90, 90, 100,100,100,100, 60]
      taus = humanoid.computePDForces(desiredPose, desiredVelocities=None, maxForces=maxForces)
      
      #print("taus=",taus)
      humanoid.applyPDForces(taus)
      
      pybullet_client.stepSimulation()
      t+=1./600.
      
    
  time.sleep(1./600.)