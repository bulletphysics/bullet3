import pybullet as pb
import time
from pybullet_utils import bullet_client

server = bullet_client.BulletClient(connection_mode=pb.SHARED_MEMORY_SERVER)

print("Connecting to bullet server")
CONNECTION_METHOD = pb.SHARED_MEMORY
client = bullet_client.BulletClient(connection_mode=CONNECTION_METHOD)

PLANE_PATH = "plane.urdf"
client.loadURDF(PLANE_PATH)

client.setGravity(0, 0, -10)

print("Adding plane object")
plane_id = client.loadURDF(PLANE_PATH)
print("Plane ID: %s" % plane_id)

print("Adding user data to plane")
MyKey1 = client.addUserData(plane_id, "MyKey1", "MyValue1")
MyKey2 = client.addUserData(plane_id, "MyKey2", "MyValue2")
MyKey3 = client.addUserData(plane_id, "MyKey3", "MyValue3")
MyKey4 = client.addUserData(plane_id, "MyKey4", "MyValue4")

print("Retrieving cached user data")
print(client.getUserData(MyKey1))
print(client.getUserData(MyKey2))
print(client.getUserData(MyKey3))
print(client.getUserData(MyKey4))

print("Disconnecting")
del client

print("Reconnecting")
client = bullet_client.BulletClient(connection_mode=CONNECTION_METHOD)

print("Retrieving synced user data")
print(client.getUserData(MyKey1))
print(client.getUserData(MyKey2))
print(client.getUserData(MyKey3))
print(client.getUserData(MyKey4))

print("Number of user data entries: %s" % client.getNumUserData(plane_id))

print("Overriding user data")
client.addUserData(plane_id, "MyKey1", "MyNewValue")

print("Cached overridden data")
print(client.getUserData(MyKey1))

print("Disconnecting")
del client

print("Reconnecting")
client = bullet_client.BulletClient(connection_mode=CONNECTION_METHOD)

print("Synced overridden data")
print(client.getUserData(MyKey1))

print("Getting user data ID")
print("Retrieved ID: %s, ID retrieved from addUserData: %s" %
      (client.getUserDataId(plane_id, "MyKey2"), MyKey2))

print("Removing user data")
client.removeUserData(MyKey2)

print("Retrieving cached removed data")
print(client.getUserData(MyKey2))

print("Syncing")
client.syncUserData()

print("Retrieving removed removed data")
print(client.getUserData(MyKey2))

print("Iterating over all user data entries and printing results")
for i in range(client.getNumUserData(plane_id)):
  userDataId, key, bodyId, linkIndex, visualShapeIndex = client.getUserDataInfo(plane_id, i)
  print("Info: (%s, %s, %s, %s, %s)" % (userDataId, key, bodyId, linkIndex, visualShapeIndex))
  print("Value: %s" % client.getUserData(userDataId))

print("Removing body")
client.removeBody(plane_id)

print("Retrieving user data")
print(client.getUserData(MyKey1))
print(client.getUserData(MyKey3))
print(client.getUserData(MyKey4))

print("Syncing")
client.syncUserData()

print("Retrieving user data")
print(client.getUserData(MyKey1))
print(client.getUserData(MyKey3))
print(client.getUserData(MyKey4))

plane_id2 = client.loadURDF(PLANE_PATH)
print("Plane1: %s, plane2: %s" % (plane_id, plane_id2))

print("Retrieving user data")
print(client.getUserData(MyKey1))
print(client.getUserData(MyKey3))
print(client.getUserData(MyKey4))
