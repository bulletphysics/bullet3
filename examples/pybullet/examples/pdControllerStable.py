import numpy as np


class PDControllerStableMultiDof(object):

  def __init__(self, pb):
    self._pb = pb

  def computePD(self, bodyUniqueId, jointIndices, desiredPositions, desiredVelocities, kps, kds,
                maxForces, timeStep):

    numJoints = len(jointIndices)  #self._pb.getNumJoints(bodyUniqueId)
    curPos, curOrn = self._pb.getBasePositionAndOrientation(bodyUniqueId)
    #q1 = [desiredPositions[0],desiredPositions[1],desiredPositions[2],desiredPositions[3],desiredPositions[4],desiredPositions[5],desiredPositions[6]]
    q1 = [curPos[0], curPos[1], curPos[2], curOrn[0], curOrn[1], curOrn[2], curOrn[3]]

    #qdot1 = [0,0,0, 0,0,0,0]
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

    #np.savetxt("pb_qError.csv", qError, delimiter=",")
    #np.savetxt("pb_p.csv", p, delimiter=",")

    d = Kd.dot(qdoterr)

    #np.savetxt("pb_d.csv", d, delimiter=",")
    #np.savetxt("pbqdoterr.csv", qdoterr, delimiter=",")

    M1 = self._pb.calculateMassMatrix(bodyUniqueId, q1, flags=1)

    M2 = np.array(M1)
    #np.savetxt("M2.csv", M2, delimiter=",")

    M = (M2 + Kd * timeStep)

    #np.savetxt("pbM_tKd.csv",M, delimiter=",")

    c1 = self._pb.calculateInverseDynamics(bodyUniqueId, q1, qdot1, zeroAccelerations, flags=1)

    c = np.array(c1)
    #np.savetxt("pbC.csv",c, delimiter=",")
    A = M
    #p = [0]*43
    b = p + d - c
    #np.savetxt("pb_acc.csv",b, delimiter=",")
    qddot = np.linalg.solve(A, b)
    tau = p + d - Kd.dot(qddot) * timeStep
    #print("len(tau)=",len(tau))
    maxF = np.array(maxForces)
    forces = np.clip(tau, -maxF, maxF)
    return forces


class PDControllerStable(object):

  def __init__(self, pb):
    self._pb = pb

  def computePD(self, bodyUniqueId, jointIndices, desiredPositions, desiredVelocities, kps, kds,
                maxForces, timeStep):
    numJoints = self._pb.getNumJoints(bodyUniqueId)
    jointStates = self._pb.getJointStates(bodyUniqueId, jointIndices)
    q1 = []
    qdot1 = []
    zeroAccelerations = []
    for i in range(numJoints):
      q1.append(jointStates[i][0])
      qdot1.append(jointStates[i][1])
      zeroAccelerations.append(0)
    q = np.array(q1)
    qdot = np.array(qdot1)
    qdes = np.array(desiredPositions)
    qdotdes = np.array(desiredVelocities)
    qError = qdes - q
    qdotError = qdotdes - qdot
    Kp = np.diagflat(kps)
    Kd = np.diagflat(kds)
    p = Kp.dot(qError)
    d = Kd.dot(qdotError)
    forces = p + d

    M1 = self._pb.calculateMassMatrix(bodyUniqueId, q1)
    M2 = np.array(M1)
    M = (M2 + Kd * timeStep)
    c1 = self._pb.calculateInverseDynamics(bodyUniqueId, q1, qdot1, zeroAccelerations)
    c = np.array(c1)
    A = M
    b = -c + p + d
    qddot = np.linalg.solve(A, b)
    tau = p + d - Kd.dot(qddot) * timeStep
    maxF = np.array(maxForces)
    forces = np.clip(tau, -maxF, maxF)
    #print("c=",c)
    return tau
