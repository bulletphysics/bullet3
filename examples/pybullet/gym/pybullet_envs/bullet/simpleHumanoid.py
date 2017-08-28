import os
import pybullet as p
import numpy as np
import copy
import math
import time

import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print ("current_dir=" + currentdir)
os.sys.path.insert(0,currentdir)

import pybullet_data


class SimpleHumanoid:

  def __init__(self, urdfRootPath=pybullet_data.getDataPath(), timeStep=0.01):
    self.urdfRootPath = urdfRootPath
    self.timeStep = timeStep
    self.reset()
    

  def reset(self):
    self.initial_z = None
   
    objs = p.loadMJCF(os.path.join(self.urdfRootPath,"mjcf/humanoid_symmetric_no_ground.xml"),flags = p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)
    self.human = objs[0]
    self.jdict = {}
    self.ordered_joints = []
    self.ordered_joint_indices = []

    for j in range( p.getNumJoints(self.human) ):
      info = p.getJointInfo(self.human, j)
      link_name = info[12].decode("ascii")
      if link_name=="left_foot": self.left_foot = j
      if link_name=="right_foot": self.right_foot = j
      self.ordered_joint_indices.append(j)
      if info[2] != p.JOINT_REVOLUTE: continue
      jname = info[1].decode("ascii")
      self.jdict[jname] = j
      lower, upper = (info[8], info[9])
      self.ordered_joints.append( (j, lower, upper) )
      p.setJointMotorControl2(self.human, j, controlMode=p.VELOCITY_CONTROL, force=0)

    self.motor_names  = ["abdomen_z", "abdomen_y", "abdomen_x"]
    self.motor_power  = [100, 100, 100]
    self.motor_names += ["right_hip_x", "right_hip_z", "right_hip_y", "right_knee"]
    self.motor_power += [100, 100, 300, 200]
    self.motor_names += ["left_hip_x", "left_hip_z", "left_hip_y", "left_knee"]
    self.motor_power += [100, 100, 300, 200]
    self.motor_names += ["right_shoulder1", "right_shoulder2", "right_elbow"]
    self.motor_power += [75, 75, 75]
    self.motor_names += ["left_shoulder1", "left_shoulder2", "left_elbow"]
    self.motor_power += [75, 75, 75]
    self.motors = [self.jdict[n] for n in self.motor_names]
    print("self.motors")
    print(self.motors)
    print("num motors")
    print(len(self.motors))



  def current_relative_position(self, jointStates, human, j, lower, upper):
    #print("j")
    #print(j)
    #print (len(jointStates))
    #print(j)
    temp  = jointStates[j]
    pos = temp[0]
    vel = temp[1]
    #print("pos")
    #print(pos)
    #print("vel")
    #print(vel)
    pos_mid = 0.5 * (lower + upper);
    return (
      2 * (pos - pos_mid) / (upper - lower),
      0.1 * vel
      )
  
  def collect_observations(self, human):
    #print("ordered_joint_indices")
    #print(ordered_joint_indices)
    
    
    jointStates = p.getJointStates(human,self.ordered_joint_indices)
    j = np.array([self.current_relative_position(jointStates, human, *jtuple) for jtuple in self.ordered_joints]).flatten()
    #print("j")
    #print(j)
    body_xyz, (qx, qy, qz, qw) = p.getBasePositionAndOrientation(human)
    #print("body_xyz")
    #print(body_xyz, qx,qy,qz,qw)
    z = body_xyz[2]
    self.distance = body_xyz[0]
    if self.initial_z==None:
      self.initial_z = z
    (vx, vy, vz), _ = p.getBaseVelocity(human)
    more = np.array([z-self.initial_z, 0.1*vx, 0.1*vy, 0.1*vz, qx, qy, qz, qw])
    rcont = p.getContactPoints(human, -1, self.right_foot, -1)
    #print("rcont")
    #print(rcont)
    lcont = p.getContactPoints(human, -1, self.left_foot, -1)
    #print("lcont")
    #print(lcont)
    feet_contact = np.array([len(rcont)>0, len(lcont)>0])
    return np.clip( np.concatenate([more] + [j] + [feet_contact]), -5, +5)

  def getActionDimension(self):
    return len(self.motors)

  def getObservationDimension(self):
    return len(self.getObservation())

  def getObservation(self):
    observation = self.collect_observations(self.human)
    return observation

  def applyAction(self, actions):
    forces = [0.] * len(self.motors)
    for m in range(len(self.motors)):
      forces[m] = self.motor_power[m]*actions[m]*0.082
    p.setJointMotorControlArray(self.human, self.motors,controlMode=p.TORQUE_CONTROL, forces=forces)

    p.stepSimulation()
    time.sleep(0.01)
    distance=5
    yaw = 0
    #humanPos, humanOrn = p.getBasePositionAndOrientation(self.human)
    #p.resetDebugVisualizerCamera(distance,yaw,-20,humanPos);
