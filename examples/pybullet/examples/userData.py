import pybullet as pb
import time
from pybullet_utils import bullet_client


server = bullet_client.BulletClient(connection_mode=pb.GUI_SERVER)


print ("Connecting to bullet server")
CONNECTION_METHOD = pb.SHARED_MEMORY
client = bullet_client.BulletClient(connection_mode=CONNECTION_METHOD)


PLANE_PATH = "plane.urdf"
client.loadURDF(PLANE_PATH)

client.setGravity(0, 0, -10)

print ("Adding plane object")
plane_id = client.loadURDF(PLANE_PATH)
print ("Plane ID: %s" % plane_id)

print ("Adding user data to plane")
MyKey1 = client.addUserData(plane_id, 0, "MyKey1", "MyValue1")
MyKey2 = client.addUserData(plane_id, 0, "MyKey2", "MyValue2")
MyKey3 = client.addUserData(plane_id, 0, "MyKey3", "MyValue3")
MyKey4 = client.addUserData(plane_id, 0, "MyKey4", "MyValue4")

print ("Retrieving cached user data")
print (client.getUserData(plane_id, 0, MyKey1))
print (client.getUserData(plane_id, 0, MyKey2))
print (client.getUserData(plane_id, 0, MyKey3))
print (client.getUserData(plane_id, 0, MyKey4))

print ("Disconnecting")
del client

print ("Reconnecting")
client = bullet_client.BulletClient(connection_mode=CONNECTION_METHOD)

print ("Retrieving synced user data")
print (client.getUserData(plane_id, 0, MyKey1))
print (client.getUserData(plane_id, 0, MyKey2))
print (client.getUserData(plane_id, 0, MyKey3))
print (client.getUserData(plane_id, 0, MyKey4))

print ("Number of user data entries: %s" % client.getNumUserData(plane_id, 0))

print ("Overriding user data")
client.addUserData(plane_id, 0, "MyKey1", "MyNewValue")

print ("Cached overridden data")
print (client.getUserData(plane_id, 0, MyKey1))


print ("Disconnecting")
del client

print ("Reconnecting")
client = bullet_client.BulletClient(connection_mode=CONNECTION_METHOD)


print ("Synced overridden data")
print (client.getUserData(plane_id, 0, MyKey1))

print ("Getting user data ID")
print ("Retrieved ID: %s, ID retrieved from addUserData: %s"  % (client.getUserDataId(plane_id, 0, "MyKey2"), MyKey2))

print ("Removing user data")
client.removeUserData(plane_id, 0, MyKey2)

print ("Retrieving cached removed data")
print (client.getUserData(plane_id, 0, MyKey2))

print ("Syncing")
client.syncUserData()

print ("Retrieving removed removed data")
print (client.getUserData(plane_id, 0, MyKey2))

print ("Iterating over all user data entries and printing results")
for i in range(client.getNumUserData(plane_id, 0)):
  userDataId, key = client.getUserDataInfo(plane_id, 0, i)
  print ("Info: (%s, %s)" % (userDataId, key))
  print ("Value: %s" % client.getUserData(plane_id, 0, userDataId))

print ("Removing body")
client.removeBody(plane_id)

print ("Retrieving user data")
print (client.getUserData(plane_id, 0, MyKey1))
print (client.getUserData(plane_id, 0, MyKey3))
print (client.getUserData(plane_id, 0, MyKey4))

print ("Syncing")
client.syncUserData()

print ("Retrieving user data")
print (client.getUserData(plane_id, 0, MyKey1))
print (client.getUserData(plane_id, 0, MyKey3))
print (client.getUserData(plane_id, 0, MyKey4))

plane_id2 = client.loadURDF(PLANE_PATH)
print ("Plane1: %s, plane2: %s" % (plane_id, plane_id2))

print ("Retrieving user data")
print (client.getUserData(plane_id, 0, MyKey1))
print (client.getUserData(plane_id, 0, MyKey3))
print (client.getUserData(plane_id, 0, MyKey4))

