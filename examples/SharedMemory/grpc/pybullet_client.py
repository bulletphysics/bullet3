"""The Python implementation of the PyBullet GRPC client."""

from __future__ import print_function

import grpc

import pybullet_pb2
import pybullet_pb2_grpc


def run():
  channel = grpc.insecure_channel('localhost:50051')
  stub = pybullet_pb2_grpc.PyBulletAPIStub(channel)
  response = stub.SubmitCommand(pybullet_pb2.PyBulletCommand(loadUrdfCommand=pybullet_pb2.LoadUrdfCommand(urdfFileName="plane.urdf", initialPosition=pybullet_pb2.vec3(x=0,y=0,z=0), useMultiBody=False, useFixedBase=True, globalScaling=2, urdfFlags = 1)))
  print("PyBullet client received: " , response.statusType)
  print("URDF objectid =", response.urdfStatus.objectUniqueId)
  
	
  response = stub.SubmitCommand(pybullet_pb2.PyBulletCommand(stepSimulationCommand=pybullet_pb2.StepSimulationCommand()))
  print("PyBullet client received: " , response.statusType)
  
  
  #response = stub.SubmitCommand(pybullet_pb2.PyBulletCommand(terminateServerCommand=pybullet_pb2.TerminateServerCommand()))
  #print("PyBullet client received: " , response.statusType)


if __name__ == '__main__':
  run()
