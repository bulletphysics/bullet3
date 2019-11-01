"""The Python implementation of the PyBullet GRPC client."""

from __future__ import print_function

import grpc

import pybullet_pb2
import pybullet_pb2_grpc

#todo: how to add this?
MJCF_COLORS_FROM_FILE = 512


def run():
  print("grpc.insecure_channel")
  channel = grpc.insecure_channel('localhost:6667')
  print("pybullet_pb2_grpc.PyBulletAPIStub")
  stub = pybullet_pb2_grpc.PyBulletAPIStub(channel)
  response = 0

  print("submit CheckVersionCommand")
  response = stub.SubmitCommand(
      pybullet_pb2.PyBulletCommand(checkVersionCommand=pybullet_pb2.CheckVersionCommand(
          clientVersion=123)))
  print("PyBullet client received: ", response)

  print("submit_ResetSimulationCommand")
  response = stub.SubmitCommand(
      pybullet_pb2.PyBulletCommand(resetSimulationCommand=pybullet_pb2.ResetSimulationCommand()))
  print("PyBullet client received: ", response)

  print("submit LoadUrdfCommand ")
  response = stub.SubmitCommand(
      pybullet_pb2.PyBulletCommand(loadUrdfCommand=pybullet_pb2.LoadUrdfCommand(
          fileName="door.urdf",
          initialPosition=pybullet_pb2.vec3(x=0, y=0, z=0),
          useMultiBody=True,
          useFixedBase=True,
          globalScaling=2,
          flags=1)))
  print("PyBullet client received: ", response)
  bodyUniqueId = response.urdfStatus.bodyUniqueId

  print("submit LoadSdfCommand")
  response = stub.SubmitCommand(
      pybullet_pb2.PyBulletCommand(loadSdfCommand=pybullet_pb2.LoadSdfCommand(
          fileName="two_cubes.sdf", useMultiBody=True, globalScaling=2)))
  print("PyBullet client received: ", response)

  print("submit LoadMjcfCommand")
  response = stub.SubmitCommand(
      pybullet_pb2.PyBulletCommand(loadMjcfCommand=pybullet_pb2.LoadMjcfCommand(
          fileName="mjcf/humanoid.xml", flags=MJCF_COLORS_FROM_FILE)))
  print("PyBullet client received: ", response)

  print("submit ChangeDynamicsCommand ")
  response = stub.SubmitCommand(
      pybullet_pb2.PyBulletCommand(changeDynamicsCommand=pybullet_pb2.ChangeDynamicsCommand(
          bodyUniqueId=bodyUniqueId, linkIndex=-1, mass=10)))
  print("PyBullet client received: ", response)

  print("submit GetDynamicsCommand ")
  response = stub.SubmitCommand(
      pybullet_pb2.PyBulletCommand(getDynamicsCommand=pybullet_pb2.GetDynamicsCommand(
          bodyUniqueId=bodyUniqueId, linkIndex=-1)))
  print("PyBullet client received: ", response)

  print("submit InitPoseCommand")
  response = stub.SubmitCommand(
      pybullet_pb2.PyBulletCommand(initPoseCommand=pybullet_pb2.InitPoseCommand(
          bodyUniqueId=bodyUniqueId, initialStateQ=[1, 2, 3], hasInitialStateQ=[1, 1, 1])))
  print("PyBullet client received: ", response)

  print("submit RequestActualStateCommand")
  response = stub.SubmitCommand(
      pybullet_pb2.
      PyBulletCommand(requestActualStateCommand=pybullet_pb2.RequestActualStateCommand(
          bodyUniqueId=bodyUniqueId, computeForwardKinematics=True, computeLinkVelocities=True)))
  print("PyBullet client received: ", response)

  i = 0
  while (True):
    i = i + 1
    print("submit StepSimulationCommand: ", i)
    response = stub.SubmitCommand(
        pybullet_pb2.PyBulletCommand(stepSimulationCommand=pybullet_pb2.StepSimulationCommand()))
    print("PyBullet client received: ", response.statusType)


#print("TerminateServerCommand")
#response = stub.SubmitCommand(pybullet_pb2.PyBulletCommand(terminateServerCommand=pybullet_pb2.TerminateServerCommand()))
#print("PyBullet client received: " , response.statusType)

if __name__ == '__main__':
  run()
