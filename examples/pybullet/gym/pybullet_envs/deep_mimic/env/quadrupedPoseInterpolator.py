from pybullet_utils import bullet_client
import math
	
class QuadrupedPoseInterpolator(object):
  def __init__(self):
    pass
    
    
  def ComputeLinVel(self,posStart, posEnd, deltaTime):
    vel = [(posEnd[0]-posStart[0])/deltaTime,(posEnd[1]-posStart[1])/deltaTime,(posEnd[2]-posStart[2])/deltaTime]
    return vel
  
  def ComputeAngVel(self,ornStart, ornEnd, deltaTime, bullet_client):
    dorn = bullet_client.getDifferenceQuaternion(ornStart,ornEnd)
    axis,angle = bullet_client.getAxisAngleFromQuaternion(dorn)
    angVel = [(axis[0]*angle)/deltaTime,(axis[1]*angle)/deltaTime,(axis[2]*angle)/deltaTime]
    return angVel
  
  def ComputeAngVelRel(self,ornStart, ornEnd, deltaTime, bullet_client):
    ornStartConjugate = [-ornStart[0],-ornStart[1],-ornStart[2],ornStart[3]]
    pos_diff, q_diff =bullet_client.multiplyTransforms([0,0,0], ornStartConjugate, [0,0,0], ornEnd)
    axis,angle = bullet_client.getAxisAngleFromQuaternion(q_diff)
    angVel = [(axis[0]*angle)/deltaTime,(axis[1]*angle)/deltaTime,(axis[2]*angle)/deltaTime]
    return angVel
  
  def Slerp(self, frameFraction, frameData, frameDataNext,bullet_client ):
    keyFrameDuration = frameData[0]
    basePos1Start = [frameData[1],frameData[2],frameData[3]]
    basePos1End = [frameDataNext[1],frameDataNext[2],frameDataNext[3]]
    self._basePos = [basePos1Start[0]+frameFraction*(basePos1End[0]-basePos1Start[0]), 
      basePos1Start[1]+frameFraction*(basePos1End[1]-basePos1Start[1]), 
      basePos1Start[2]+frameFraction*(basePos1End[2]-basePos1Start[2])]
    self._baseLinVel = self.ComputeLinVel(basePos1Start,basePos1End, keyFrameDuration)
    baseOrn1Start = [frameData[5],frameData[6], frameData[7],frameData[4]]
    baseOrn1Next = [frameDataNext[5],frameDataNext[6], frameDataNext[7],frameDataNext[4]]
    self._baseOrn = bullet_client.getQuaternionSlerp(baseOrn1Start,baseOrn1Next,frameFraction)
    self._baseAngVel = self.ComputeAngVel(baseOrn1Start,baseOrn1Next, keyFrameDuration, bullet_client)
    
    jointPositions=[self._basePos[0],self._basePos[1],self._basePos[2],
       self._baseOrn[0],self._baseOrn[1],self._baseOrn[2],self._baseOrn[3]]
    jointVelocities=[self._baseLinVel[0],self._baseLinVel[1],self._baseLinVel[2],
      self._baseAngVel[0],self._baseAngVel[1],self._baseAngVel[2]]
    
    for j in range (12):
      index=j+8
      jointPosStart=frameData[index]
      jointPosEnd=frameDataNext[index]
      jointPos=jointPosStart+frameFraction*(jointPosEnd-jointPosStart)
      jointVel=(jointPosEnd-jointPosStart)/keyFrameDuration
      jointPositions.append(jointPos)
      jointVelocities.append(jointVel)
    self._jointPositions = jointPositions
    self._jointVelocities = jointVelocities
    return jointPositions,jointVelocities
