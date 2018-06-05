import unittest
import pybullet

from pybullet_utils import bullet_client

PLANE_PATH = "plane.urdf"
ROBOT_PATH = "r2d2.urdf"


class TestUserDataMethods(unittest.TestCase):

	@classmethod
	def setUpClass(cls):
		TestUserDataMethods.server = bullet_client.BulletClient(connection_mode=pybullet.SHARED_MEMORY_SERVER)

	@classmethod
	def tearDownClass(cls):
		del TestUserDataMethods.server

	def setUp(self):
		self.client = bullet_client.BulletClient(pybullet.SHARED_MEMORY)

	def tearDown(self):
		self.client.resetSimulation()
		del self.client

	def testAddUserData(self):
		plane_id = self.client.loadURDF(PLANE_PATH)
		uid1 = self.client.addUserData(plane_id, 0, "MyKey1", "MyValue1")
		uid2 = self.client.addUserData(plane_id, 0, "MyKey2", "MyValue2")
		uid3 = self.client.addUserData(plane_id, 0, "MyKey3", "MyValue3")
		uid4 = self.client.addUserData(plane_id, 0, "MyKey4", "MyValue4")

		# Retrieve user data and make sure it's correct.
		self.assertEqual(b"MyValue1", self.client.getUserData(plane_id, 0, uid1))
		self.assertEqual(b"MyValue2", self.client.getUserData(plane_id, 0, uid2))
		self.assertEqual(b"MyValue3", self.client.getUserData(plane_id, 0, uid3))
		self.assertEqual(b"MyValue4", self.client.getUserData(plane_id, 0, uid4))

		# Disconnect/reconnect and make sure that the user data is synced back.
		del self.client
		self.client = bullet_client.BulletClient(pybullet.SHARED_MEMORY)

		self.assertEqual(b"MyValue1", self.client.getUserData(plane_id, 0, uid1))
		self.assertEqual(b"MyValue2", self.client.getUserData(plane_id, 0, uid2))
		self.assertEqual(b"MyValue3", self.client.getUserData(plane_id, 0, uid3))
		self.assertEqual(b"MyValue4", self.client.getUserData(plane_id, 0, uid4))

		self.client.resetSimulation()
		self.assertEqual(None, self.client.getUserData(plane_id, 0, uid1))
		self.assertEqual(None, self.client.getUserData(plane_id, 0, uid2))
		self.assertEqual(None, self.client.getUserData(plane_id, 0, uid3))
		self.assertEqual(None, self.client.getUserData(plane_id, 0, uid4))


	def testGetNumUserData(self):
		plane_id = self.client.loadURDF(PLANE_PATH)
		uid1 = self.client.addUserData(plane_id, 0, "MyKey1", "MyValue1")
		uid2 = self.client.addUserData(plane_id, 0, "MyKey2", "MyValue2")
		uid3 = self.client.addUserData(plane_id, 0, "MyKey3", "MyValue3")
		uid4 = self.client.addUserData(plane_id, 0, "MyKey4", "MyValue4")

		self.assertEqual(4, self.client.getNumUserData(plane_id, 0))
		
		del self.client
		self.client = bullet_client.BulletClient(pybullet.SHARED_MEMORY)

		self.assertEqual(4, self.client.getNumUserData(plane_id, 0))


	def testReplaceUserData(self):
		plane_id = self.client.loadURDF(PLANE_PATH)
		uid = self.client.addUserData(plane_id, 0, "MyKey", "MyValue")

		self.assertEqual(b"MyValue", self.client.getUserData(plane_id, 0, uid))
		
		new_uid = self.client.addUserData(plane_id, 0, "MyKey", "MyNewValue")
		self.assertEqual(uid, new_uid)
		self.assertEqual(b"MyNewValue", self.client.getUserData(plane_id, 0, uid))

		del self.client
		self.client = bullet_client.BulletClient(pybullet.SHARED_MEMORY)

		self.assertEqual(b"MyNewValue", self.client.getUserData(plane_id, 0, uid))

	def testGetUserDataId(self):
		plane_id = self.client.loadURDF(PLANE_PATH)
		uid1 = self.client.addUserData(plane_id, 0, "MyKey1", "MyValue1")
		uid2 = self.client.addUserData(plane_id, 0, "MyKey2", "MyValue2")
		uid3 = self.client.addUserData(plane_id, 0, "MyKey3", "MyValue3")
		uid4 = self.client.addUserData(plane_id, 0, "MyKey4", "MyValue4")

		self.assertEqual(uid1, self.client.getUserDataId(plane_id, 0, "MyKey1"))
		self.assertEqual(uid2, self.client.getUserDataId(plane_id, 0, "MyKey2"))
		self.assertEqual(uid3, self.client.getUserDataId(plane_id, 0, "MyKey3"))
		self.assertEqual(uid4, self.client.getUserDataId(plane_id, 0, "MyKey4"))

		del self.client
		self.client = bullet_client.BulletClient(pybullet.SHARED_MEMORY)

		self.assertEqual(uid1, self.client.getUserDataId(plane_id, 0, "MyKey1"))
		self.assertEqual(uid2, self.client.getUserDataId(plane_id, 0, "MyKey2"))
		self.assertEqual(uid3, self.client.getUserDataId(plane_id, 0, "MyKey3"))
		self.assertEqual(uid4, self.client.getUserDataId(plane_id, 0, "MyKey4"))


	def testRemoveUserData(self):
		plane_id = self.client.loadURDF(PLANE_PATH)
		uid1 = self.client.addUserData(plane_id, 0, "MyKey1", "MyValue1")
		uid2 = self.client.addUserData(plane_id, 0, "MyKey2", "MyValue2")
		uid3 = self.client.addUserData(plane_id, 0, "MyKey3", "MyValue3")
		uid4 = self.client.addUserData(plane_id, 0, "MyKey4", "MyValue4")

		self.client.removeUserData(plane_id, 0, uid2)

		self.assertEqual(3, self.client.getNumUserData(plane_id, 0))
		self.assertEqual(-1, self.client.getUserDataId(plane_id, 0, "MyKey2"))
		self.assertEqual(None, self.client.getUserData(plane_id, 0, uid2))
		self.assertEqual(b"MyValue1", self.client.getUserData(plane_id, 0, uid1))
		self.assertEqual(b"MyValue3", self.client.getUserData(plane_id, 0, uid3))
		self.assertEqual(b"MyValue4", self.client.getUserData(plane_id, 0, uid4))

		del self.client
		self.client = bullet_client.BulletClient(pybullet.SHARED_MEMORY)

		self.assertEqual(3, self.client.getNumUserData(plane_id, 0))
		self.assertEqual(-1, self.client.getUserDataId(plane_id, 0, "MyKey2"))
		self.assertEqual(None, self.client.getUserData(plane_id, 0, uid2))
		self.assertEqual(b"MyValue1", self.client.getUserData(plane_id, 0, uid1))
		self.assertEqual(b"MyValue3", self.client.getUserData(plane_id, 0, uid3))
		self.assertEqual(b"MyValue4", self.client.getUserData(plane_id, 0, uid4))


	def testIterateAllUserData(self):
		plane_id = self.client.loadURDF(PLANE_PATH)
		uid1 = self.client.addUserData(plane_id, 0, "MyKey1", "MyValue1")
		uid2 = self.client.addUserData(plane_id, 0, "MyKey2", "MyValue2")
		uid3 = self.client.addUserData(plane_id, 0, "MyKey3", "MyValue3")
		uid4 = self.client.addUserData(plane_id, 0, "MyKey4", "MyValue4")

		entries = set()
		for i in range(self.client.getNumUserData(plane_id, 0)):
			userDataId, key = self.client.getUserDataInfo(plane_id, 0, i)
			value = self.client.getUserData(plane_id, 0, userDataId);
			entries.add((userDataId, key, value))

		self.assertTrue((uid1, b"MyKey1", b"MyValue1") in entries)
		self.assertTrue((uid2, b"MyKey2", b"MyValue2") in entries)
		self.assertTrue((uid3, b"MyKey3", b"MyValue3") in entries)
		self.assertTrue((uid4, b"MyKey4", b"MyValue4") in entries)
		self.assertEqual(4, len(entries))


	def testRemoveBody(self):
		plane_id = self.client.loadURDF(PLANE_PATH)
		uid1 = self.client.addUserData(plane_id, 0, "MyKey1", "MyValue1")
		uid2 = self.client.addUserData(plane_id, 0, "MyKey2", "MyValue2")
		uid3 = self.client.addUserData(plane_id, 0, "MyKey3", "MyValue3")
		uid4 = self.client.addUserData(plane_id, 0, "MyKey4", "MyValue4")

		self.client.removeBody(plane_id)
		self.assertEqual(None, self.client.getUserData(plane_id, 0, uid1))
		self.assertEqual(None, self.client.getUserData(plane_id, 0, uid2))
		self.assertEqual(None, self.client.getUserData(plane_id, 0, uid3))
		self.assertEqual(None, self.client.getUserData(plane_id, 0, uid4))

		del self.client
		self.client = bullet_client.BulletClient(pybullet.SHARED_MEMORY)

		self.assertEqual(None, self.client.getUserData(plane_id, 0, uid1))
		self.assertEqual(None, self.client.getUserData(plane_id, 0, uid2))
		self.assertEqual(None, self.client.getUserData(plane_id, 0, uid3))
		self.assertEqual(None, self.client.getUserData(plane_id, 0, uid4))

	def testMultipleBodies(self):
		plane1 = self.client.loadURDF(PLANE_PATH)
		plane2 = self.client.loadURDF(PLANE_PATH)
		
		uid1 = self.client.addUserData(plane1, 0, "MyKey1", "This is plane 1 - 1")
		uid2 = self.client.addUserData(plane1, 0, "MyKey2", "This is plane 1 - 2")

		uid3 = self.client.addUserData(plane2, 0, "MyKey1", "This is plane 2 - 1")
		uid4 = self.client.addUserData(plane2, 0, "MyKey2", "This is plane 2 - 2")
		uid5 = self.client.addUserData(plane2, 0, "MyKey3", "This is plane 2 - 3")

		self.assertEqual(b"This is plane 1 - 1", self.client.getUserData(plane1, 0, self.client.getUserDataId(plane1, 0, "MyKey1")))
		self.assertEqual(b"This is plane 1 - 2", self.client.getUserData(plane1, 0, self.client.getUserDataId(plane1, 0, "MyKey2")))

		self.assertEqual(b"This is plane 2 - 1", self.client.getUserData(plane2, 0, self.client.getUserDataId(plane2, 0, "MyKey1")))
		self.assertEqual(b"This is plane 2 - 2", self.client.getUserData(plane2, 0, self.client.getUserDataId(plane2, 0, "MyKey2")))
		self.assertEqual(b"This is plane 2 - 3", self.client.getUserData(plane2, 0, self.client.getUserDataId(plane2, 0, "MyKey3")))


	def testMultipleLinks(self):
		body_id = self.client.loadURDF(ROBOT_PATH)
		num_links = self.client.getNumJoints(body_id)

		self.assertTrue(num_links > 1)

		for link_index in range(num_links):
			uid1 = self.client.addUserData(body_id, link_index, "MyKey1", "Value1 for link %s" % link_index)
			uid2 = self.client.addUserData(body_id, link_index, "MyKey2", "Value2 for link %s" % link_index)

		for link_index in range(num_links):
			uid1 = self.client.getUserDataId(body_id, link_index, "MyKey1")
			uid2 = self.client.getUserDataId(body_id, link_index, "MyKey2")
			self.assertEqual(("Value1 for link %s" % link_index).encode(), self.client.getUserData(body_id, link_index, uid1))
			self.assertEqual(("Value2 for link %s" % link_index).encode(), self.client.getUserData(body_id, link_index, uid2))

	def testMultipleClients(self):
		client1 = self.client
		client2 = bullet_client.BulletClient(pybullet.SHARED_MEMORY)

		plane_id = client1.loadURDF(PLANE_PATH)
		client2.syncBodyInfo()

		# Add user data on client 1, check on client 1
		uid = client1.addUserData(plane_id, 0, "MyKey", "MyValue")
		self.assertEqual(None, client2.getUserData(plane_id, 0, uid))
		client2.syncUserData()
		self.assertEqual(b"MyValue", client2.getUserData(plane_id, 0, uid))

		# Overwrite the value on client 2, check on client 1
		client2.addUserData(plane_id, 0, "MyKey", "MyNewValue")
		self.assertEqual(b"MyValue", client1.getUserData(plane_id, 0, uid))
		client1.syncUserData()
		self.assertEqual(b"MyNewValue", client1.getUserData(plane_id, 0, uid))

		# Remove user data on client 1, check on client 2
		client1.removeUserData(plane_id, 0, uid)
		self.assertEqual(b"MyNewValue", client2.getUserData(plane_id, 0, uid))
		client2.syncUserData()
		self.assertEqual(None, client2.getUserData(plane_id, 0, uid))

		del client2


if __name__ == "__main__":
    unittest.main()
