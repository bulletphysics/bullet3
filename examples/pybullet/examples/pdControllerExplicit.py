import numpy as np


class PDControllerExplicitMultiDof(object):

  def __init__(self, pb):
    self._pb = pb

  def computePD(self, bodyUniqueId, jointIndices, desiredPositions, desiredVelocities, kps, kds,
                maxForces, timeStep):

    numJoints = len(jointIndices)  #self._pb.getNumJoints(bodyUniqueId)
    curPos, curOrn = self._pb.getBasePositionAndOrientation(bodyUniqueId)
    q1 = [curPos[0], curPos[1], curPos[2], curOrn[0], curOrn[1], curOrn[2], curOrn[3]]
    baseLinVel, baseAngVel = self._pb.getBaseVelocity(bodyUniqueId)
    qdot1 = [
        baseLinVel[0], baseLinVel[1], baseLinVel[2], baseAngVel[0], baseAngVel[1], baseAngVel[2], 0
    ]
    qError = [0, 0, 0, 0, 0, 0, 0]
    qIndex = 7
    qdotIndex = 7
    zeroAccelerations = [0, 0, 0, 0, 0, 0, 0]
    for i in range(numJoints):
      js = self._pb.getJointStateMultiDof(bodyUniqueId, jointIndices[i])

      jointPos = js[0]
      jointVel = js[1]
      q1 += jointPos

      if len(js[0]) == 1:
        desiredPos = desiredPositions[qIndex]

        qdiff = desiredPos - jointPos[0]
        qError.append(qdiff)
        zeroAccelerations.append(0.)
        qdot1 += jointVel
        qIndex += 1
        qdotIndex += 1
      if len(js[0]) == 4:
        desiredPos = [
            desiredPositions[qIndex], desiredPositions[qIndex + 1], desiredPositions[qIndex + 2],
            desiredPositions[qIndex + 3]
        ]
        axis = self._pb.getAxisDifferenceQuaternion(desiredPos, jointPos)
        jointVelNew = [jointVel[0], jointVel[1], jointVel[2], 0]
        qdot1 += jointVelNew
        qError.append(axis[0])
        qError.append(axis[1])
        qError.append(axis[2])
        qError.append(0)
        desiredVel = [
            desiredVelocities[qdotIndex], desiredVelocities[qdotIndex + 1],
            desiredVelocities[qdotIndex + 2]
        ]
        zeroAccelerations += [0., 0., 0., 0.]
        qIndex += 4
        qdotIndex += 4

    q = np.array(q1)
    qdot = np.array(qdot1)
    qdotdesired = np.array(desiredVelocities)
    qdoterr = qdotdesired - qdot
    Kp = np.diagflat(kps)
    Kd = np.diagflat(kds)
    p = Kp.dot(qError)
    d = Kd.dot(qdoterr)
    forces = p + d
    maxF = np.array(maxForces)
    forces = np.clip(forces, -maxF, maxF)
    return forces


class PDControllerExplicit(object):

  def __init__(self, pb):
    self._pb = pb

  def computePD(self, bodyUniqueId, jointIndices, desiredPositions, desiredVelocities, kps, kds,
                maxForces, timeStep):
    numJoints = self._pb.getNumJoints(bodyUniqueId)
    jointStates = self._pb.getJointStates(bodyUniqueId, jointIndices)
    q1 = []
    qdot1 = []
    for i in range(numJoints):
      q1.append(jointStates[i][0])
      qdot1.append(jointStates[i][1])
    q = np.array(q1)
    qdot = np.array(qdot1)
    qdes = np.array(desiredPositions)
    qdotdes = np.array(desiredVelocities)
    qError = qdes - q
    qdotError = qdotdes - qdot
    Kp = np.diagflat(kps)
    Kd = np.diagflat(kds)
    forces = Kp.dot(qError) + Kd.dot(qdotError)
    maxF = np.array(maxForces)
    forces = np.clip(forces, -maxF, maxF)
    return forces
