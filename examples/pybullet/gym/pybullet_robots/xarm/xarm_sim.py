import time
import numpy as np
import math

useNullSpace = 0
useDynamics = 1
useIKFast = 0 #ikfast doesn't get solutions and is actually slower than pybullet IK (300us versus 150us), probably a configuration issue
if useIKFast:
  import ikfastpy

ikSolver = 0
xarmEndEffectorIndex = 6
xarmNumDofs = 6

ll = [-17]*xarmNumDofs
#upper limits for null space (todo: set them to proper range)
ul = [17]*xarmNumDofs
#joint ranges for null space (todo: set them to proper range)
jr = [17]*xarmNumDofs
#restposes for null space
jointPositions=[0,0,0,0,0,0]
rp = jointPositions
jointPoses = jointPositions
class XArm6Sim(object):
  def __init__(self, bullet_client, offset):
    if useIKFast:
      # Initialize kinematics for UR5 robot arm
      self.xarm_kin = ikfastpy.PyKinematics()
      self.n_joints = self.xarm_kin.getDOF()
      print("numJoints IKFast=", self.n_joints)

    self.bullet_client = bullet_client
    self.offset = np.array(offset)
    self.jointPoses = [0]*xarmNumDofs
    #print("offset=",offset)
    flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
    legos=[]
    self.bullet_client.loadURDF("tray/traybox.urdf", [0.4+offset[0], offset[1], offset[2]], [0,0,0,1], flags=flags)
    legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([0.3, 0.1, 0.3])+self.offset, flags=flags))
    legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([0.3, -0.1, 0.3])+self.offset, flags=flags))
    legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([0.5, 0.1, 0.3])+self.offset, flags=flags))
    sphereId = self.bullet_client.loadURDF("sphere_small.urdf",np.array( [0.4, 0, 0.3])+self.offset, flags=flags)
    self.bullet_client.loadURDF("sphere_small.urdf",np.array( [0.3, 0, 0.3])+self.offset, flags=flags)
    self.bullet_client.loadURDF("sphere_small.urdf",np.array( [0.5, 0, 0.3])+self.offset, flags=flags)
    orn=[0,0,0,1]
    self.xarm = self.bullet_client.loadURDF("xarm/xarm6_robot.urdf", np.array([0,0,0])+self.offset, orn, useFixedBase=True, flags=flags)
    index = 0
    for j in range(self.bullet_client.getNumJoints(self.xarm)):
      self.bullet_client.changeDynamics(self.xarm, j, linearDamping=0, angularDamping=0)
      info = self.bullet_client.getJointInfo(self.xarm, j)
  
      jointName = info[1]
      jointType = info[2]
      if (jointType == self.bullet_client.JOINT_PRISMATIC):
        
        self.bullet_client.resetJointState(self.xarm, j, jointPositions[index]) 
        index=index+1
      if (jointType == self.bullet_client.JOINT_REVOLUTE):
        self.bullet_client.resetJointState(self.xarm, j, jointPositions[index]) 
        index=index+1
    self.t = 0.
  def reset(self):
    pass

  def step(self):
    t = self.t
    self.t += 1./60.
    
    pos = [0.407+self.offset[0]+0.2 * math.sin(1.5 * t), 0.0+self.offset[1]+0.2 * math.cos(1.5 * t), 0.12+self.offset[2]]
    #orn = self.bullet_client.getQuaternionFromEuler([math.pi/2.,0.,0.])
    orn = [1,0,0,0]
    
    if useIKFast:
      
      #mat = self.bullet_client.getMatrixFromQuaternion(orn)
      #ee_pose = np.array([mat[0],mat[1],mat[2],pos[0],
      #          mat[3],mat[4],mat[5],pos[1],
      #          mat[6],mat[7],mat[8],pos[2]])
      #ee_pose = np.asarray(ee_pose).reshape(3,4)
      joint_angles = [0.39,0,0,0,0,0]#-3.1,-1.6,1.6,-1.6,-1.6,0.] # in radians
      self.bullet_client.submitProfileTiming("ikfast.forwardDynamics")
      ee_pose = self.xarm_kin.forward(joint_angles)
      self.bullet_client.submitProfileTiming()
      ee_pose = np.asarray(ee_pose).reshape(3,4) 
      #print("ee_pose=",ee_pose)
      self.bullet_client.submitProfileTiming("ikfast.inverseDynamics")
      joint_configs = self.xarm_kin.inverse(ee_pose.reshape(-1).tolist())
      self.bullet_client.submitProfileTiming()
      n_solutions = int(len(joint_configs)/self.n_joints)
      print("%d solutions found:"%(n_solutions))
      joint_configs = np.asarray(joint_configs).reshape(n_solutions,self.n_joints)
      for joint_config in joint_configs:
        print(joint_config)
      jointPoses = [0]*self.n_joints
      
    else: 
      if useNullSpace:
        jointPoses = self.bullet_client.calculateInverseKinematics(self.xarm,xarmEndEffectorIndex, pos, orn,lowerLimits=ll, 
          upperLimits=ul,jointRanges=jr, restPoses=np.array(self.jointPoses).tolist(),residualThreshold=1e-5, maxNumIterations=50)
        self.jointPoses = [0,0,jointPoses[2],jointPoses[3],jointPoses[4],jointPoses[5]]
      else:
        jointPoses = self.bullet_client.calculateInverseKinematics(self.xarm,xarmEndEffectorIndex, pos, orn, maxNumIterations=50)
    #print("jointPoses=",jointPoses)
    if useDynamics:
      for i in range(xarmNumDofs):
          self.bullet_client.setJointMotorControl2(self.xarm, i+1, self.bullet_client.POSITION_CONTROL, jointPoses[i],force=5 * 240.)
    else:
      for i in range(xarmNumDofs):
        self.bullet_client.resetJointState(self.xarm, i+1, jointPoses[i])
    ls = self.bullet_client.getLinkState(self.xarm, xarmEndEffectorIndex, computeForwardKinematics=True)
    linkComPos=np.array(ls[0])
    linkComOrn=ls[1]
    linkUrdfPos=np.array(ls[4])
    linkUrdfOrn=ls[5]
    #print("linkComPos=",linkComPos)
    #print("linkUrdfOrn=",linkUrdfOrn)
    mat = self.bullet_client.getMatrixFromQuaternion(linkUrdfOrn)
    #print("mat=",mat)
    self.bullet_client.addUserDebugLine(pos, linkUrdfPos, [1,0,0,1],lifeTime=100)
    diff = linkUrdfPos-np.array(pos)
    #print("diff=",diff)
    
  
