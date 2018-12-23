import numpy as np

class PDControllerExplicit(object):
		def __init__(self, pb):
			self._pb = pb

		def computePD(self, bodyUniqueId, jointIndices, desiredPositions, desiredVelocities, kps, kds, maxForces, timeStep):
			numJoints = self._pb.getNumJoints(bodyUniqueId)
			jointStates = self._pb.getJointStates(bodyUniqueId, jointIndices)
			q1 = []
			qdot1 = []
			for i in range (numJoints):
				q1.append(jointStates[i][0])
				qdot1.append(jointStates[i][1])
			q = np.array(q1)
			qdot=np.array(qdot1)
			qdes = np.array(desiredPositions)
			qdotdes = np.array(desiredVelocities)
			qError = qdes - q
			qdotError = qdotdes - qdot
			Kp = np.diagflat(kps)
			Kd = np.diagflat(kds)
			forces = Kp.dot(qError) + Kd.dot(qdotError)
			maxF = np.array(maxForces)
			forces = np.clip(forces, -maxF , maxF )
			return forces
			
			
class PDControllerStable(object):
		def __init__(self, pb):
			self._pb = pb

		def computePD(self, bodyUniqueId, jointIndices, desiredPositions, desiredVelocities, kps, kds, maxForces, timeStep):
			numJoints = self._pb.getNumJoints(bodyUniqueId)
			jointStates = self._pb.getJointStates(bodyUniqueId, jointIndices)
			q1 = []
			qdot1 = []
			zeroAccelerations = []
			for i in range (numJoints):
				q1.append(jointStates[i][0])
				qdot1.append(jointStates[i][1])
				zeroAccelerations.append(0)
			q = np.array(q1)
			qdot=np.array(qdot1)
			qdes = np.array(desiredPositions)
			qdotdes = np.array(desiredVelocities)
			qError = qdes - q
			qdotError = qdotdes - qdot
			Kp = np.diagflat(kps)
			Kd = np.diagflat(kds)
			p =  Kp.dot(qError)
			d = Kd.dot(qdotError)
			forces = p + d
			
			M1 = self._pb.calculateMassMatrix(bodyUniqueId,q1)
			M2 = np.array(M1)
			M = (M2 + Kd * timeStep)
			c1 = self._pb.calculateInverseDynamics(bodyUniqueId, q1, qdot1, zeroAccelerations)
			c = np.array(c1)
			A = M
			b = -c + p + d
			qddot = np.linalg.solve(A, b)
			tau = p + d - Kd.dot(qddot) * timeStep
			maxF = np.array(maxForces)
			forces = np.clip(tau, -maxF , maxF )
			#print("c=",c)
			return tau