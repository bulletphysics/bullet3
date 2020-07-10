import pybullet as p
import time
import math

# This simple snake logic is from some 15 year old Havok C++ demo
# Thanks to Michael Ewert!
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.createCollisionShape(p.GEOM_PLANE)

p.createMultiBody(0, plane)

useMaximalCoordinates = True
sphereRadius = 0.25
#colBoxId = p.createCollisionShapeArray([p.GEOM_BOX, p.GEOM_SPHERE],radii=[sphereRadius+0.03,sphereRadius+0.03], halfExtents=[[sphereRadius,sphereRadius,sphereRadius],[sphereRadius,sphereRadius,sphereRadius]])
colBoxId = p.createCollisionShape(p.GEOM_BOX,
                                  halfExtents=[sphereRadius, sphereRadius, sphereRadius])

mass = 1
visualShapeId = -1

link_Masses = []
linkCollisionShapeIndices = []
linkVisualShapeIndices = []
linkPositions = []
linkOrientations = []
linkInertialFramePositions = []
linkInertialFrameOrientations = []
indices = []
jointTypes = []
axis = []

for i in range(36):
  link_Masses.append(1)
  linkCollisionShapeIndices.append(colBoxId)
  linkVisualShapeIndices.append(-1)
  linkPositions.append([0, sphereRadius * 2.0 + 0.01, 0])
  linkOrientations.append([0, 0, 0, 1])
  linkInertialFramePositions.append([0, 0, 0])
  linkInertialFrameOrientations.append([0, 0, 0, 1])
  indices.append(i)
  jointTypes.append(p.JOINT_REVOLUTE)
  axis.append([0, 0, 1])

basePosition = [0, 0, 1]
baseOrientation = [0, 0, 0, 1]
sphereUid = p.createMultiBody(mass,
                              colBoxId,
                              visualShapeId,
                              basePosition,
                              baseOrientation,
                              linkMasses=link_Masses,
                              linkCollisionShapeIndices=linkCollisionShapeIndices,
                              linkVisualShapeIndices=linkVisualShapeIndices,
                              linkPositions=linkPositions,
                              linkOrientations=linkOrientations,
                              linkInertialFramePositions=linkInertialFramePositions,
                              linkInertialFrameOrientations=linkInertialFrameOrientations,
                              linkParentIndices=indices,
                              linkJointTypes=jointTypes,
                              linkJointAxis=axis,
                              useMaximalCoordinates=useMaximalCoordinates)

p.setGravity(0, 0, -10)
p.setRealTimeSimulation(0)

anistropicFriction = [1, 0.01, 0.01]
p.changeDynamics(sphereUid, -1, lateralFriction=2, anisotropicFriction=anistropicFriction)
p.getNumJoints(sphereUid)
for i in range(p.getNumJoints(sphereUid)):
  p.getJointInfo(sphereUid, i)
  p.changeDynamics(sphereUid, i, lateralFriction=2, anisotropicFriction=anistropicFriction)

dt = 1. / 240.
SNAKE_NORMAL_PERIOD = 0.1  #1.5
m_wavePeriod = SNAKE_NORMAL_PERIOD

m_waveLength = 4
m_wavePeriod = 1.5
m_waveAmplitude = 0.4
m_waveFront = 0.0
#our steering value
m_steering = 0.0
m_segmentLength = sphereRadius * 2.0
forward = 0

while (1):
  keys = p.getKeyboardEvents()
  for k, v in keys.items():

    if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_TRIGGERED)):
      m_steering = -.2
    if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
      m_steering = 0
    if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_TRIGGERED)):
      m_steering = .2
    if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
      m_steering = 0

  amp = 0.2
  offset = 0.6
  numMuscles = p.getNumJoints(sphereUid)
  scaleStart = 1.0

  #start of the snake with smaller waves.
  #I think starting the wave at the tail would work better ( while it still goes from head to tail )
  if (m_waveFront < m_segmentLength * 4.0):
    scaleStart = m_waveFront / (m_segmentLength * 4.0)

  segment = numMuscles - 1

  #we simply move a sin wave down the body of the snake.
  #this snake may be going backwards, but who can tell ;)
  for joint in range(p.getNumJoints(sphereUid)):
    segment = joint  #numMuscles-1-joint
    #map segment to phase
    phase = (m_waveFront - (segment + 1) * m_segmentLength) / m_waveLength
    phase -= math.floor(phase)
    phase *= math.pi * 2.0

    #map phase to curvature
    targetPos = math.sin(phase) * scaleStart * m_waveAmplitude

    #// steer snake by squashing +ve or -ve side of sin curve
    if (m_steering > 0 and targetPos < 0):
      targetPos *= 1.0 / (1.0 + m_steering)

    if (m_steering < 0 and targetPos > 0):
      targetPos *= 1.0 / (1.0 - m_steering)

    #set our motor
    p.setJointMotorControl2(sphereUid,
                            joint,
                            p.POSITION_CONTROL,
                            targetPosition=targetPos + m_steering,
                            force=30)

  #wave keeps track of where the wave is in time
  m_waveFront += dt / m_wavePeriod * m_waveLength
  p.stepSimulation()

  time.sleep(dt)
