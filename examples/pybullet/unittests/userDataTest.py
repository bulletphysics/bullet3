import unittest
import pybullet

PLANE_PATH = "plane.urdf"
ROBOT_PATH = "r2d2.urdf"

class TestUserDataMethods(unittest.TestCase):

	def setUp(self):
		self.client_id = pybullet.connect(pybullet.SHARED_MEMORY)

	def tearDown(self):
		pybullet.resetSimulation()
		pybullet.disconnect()

	def testAddUserData(self):
		plane_id = pybullet.loadURDF(PLANE_PATH)
		uid1 = pybullet.addUserData(plane_id, 0, "MyKey1", "MyValue1")
		uid2 = pybullet.addUserData(plane_id, 0, "MyKey2", "MyValue2")
		uid3 = pybullet.addUserData(plane_id, 0, "MyKey3", "MyValue3")
		uid4 = pybullet.addUserData(plane_id, 0, "MyKey4", "MyValue4")

		# Retrieve user data and make sure it's correct.
		self.assertEqual(b"MyValue1", pybullet.getUserData(plane_id, 0, uid1))
		self.assertEqual(b"MyValue2", pybullet.getUserData(plane_id, 0, uid2))
		self.assertEqual(b"MyValue3", pybullet.getUserData(plane_id, 0, uid3))
		self.assertEqual(b"MyValue4", pybullet.getUserData(plane_id, 0, uid4))

		# Disconnect/reconnect and make sure that the user data is synced back.
		pybullet.disconnect()
		pybullet.connect(pybullet.SHARED_MEMORY)

		self.assertEqual(b"MyValue1", pybullet.getUserData(plane_id, 0, uid1))
		self.assertEqual(b"MyValue2", pybullet.getUserData(plane_id, 0, uid2))
		self.assertEqual(b"MyValue3", pybullet.getUserData(plane_id, 0, uid3))
		self.assertEqual(b"MyValue4", pybullet.getUserData(plane_id, 0, uid4))

		pybullet.resetSimulation()
		self.assertEqual(None, pybullet.getUserData(plane_id, 0, uid1))
		self.assertEqual(None, pybullet.getUserData(plane_id, 0, uid2))
		self.assertEqual(None, pybullet.getUserData(plane_id, 0, uid3))
		self.assertEqual(None, pybullet.getUserData(plane_id, 0, uid4))


	def testGetNumUserData(self):
		pybullet.connect(pybullet.SHARED_MEMORY)
		plane_id = pybullet.loadURDF(PLANE_PATH)
		uid1 = pybullet.addUserData(plane_id, 0, "MyKey1", "MyValue1")
		uid2 = pybullet.addUserData(plane_id, 0, "MyKey2", "MyValue2")
		uid3 = pybullet.addUserData(plane_id, 0, "MyKey3", "MyValue3")
		uid4 = pybullet.addUserData(plane_id, 0, "MyKey4", "MyValue4")

		self.assertEqual(4, pybullet.getNumUserData(plane_id, 0))
		
		pybullet.disconnect()
		pybullet.connect(pybullet.SHARED_MEMORY)

		self.assertEqual(4, pybullet.getNumUserData(plane_id, 0))


	def testReplaceUserData(self):
		pybullet.connect(pybullet.SHARED_MEMORY)
		plane_id = pybullet.loadURDF(PLANE_PATH)
		uid = pybullet.addUserData(plane_id, 0, "MyKey", "MyValue")

		self.assertEqual(b"MyValue", pybullet.getUserData(plane_id, 0, uid))
		
		new_uid = pybullet.addUserData(plane_id, 0, "MyKey", "MyNewValue")
		self.assertEqual(uid, new_uid)
		self.assertEqual(b"MyNewValue", pybullet.getUserData(plane_id, 0, uid))

		pybullet.disconnect()
		pybullet.connect(pybullet.SHARED_MEMORY)

		self.assertEqual(b"MyNewValue", pybullet.getUserData(plane_id, 0, uid))

	def testGetUserDataId(self):
		pybullet.connect(pybullet.SHARED_MEMORY)
		plane_id = pybullet.loadURDF(PLANE_PATH)
		uid1 = pybullet.addUserData(plane_id, 0, "MyKey1", "MyValue1")
		uid2 = pybullet.addUserData(plane_id, 0, "MyKey2", "MyValue2")
		uid3 = pybullet.addUserData(plane_id, 0, "MyKey3", "MyValue3")
		uid4 = pybullet.addUserData(plane_id, 0, "MyKey4", "MyValue4")

		self.assertEqual(uid1, pybullet.getUserDataId(plane_id, 0, "MyKey1"))
		self.assertEqual(uid2, pybullet.getUserDataId(plane_id, 0, "MyKey2"))
		self.assertEqual(uid3, pybullet.getUserDataId(plane_id, 0, "MyKey3"))
		self.assertEqual(uid4, pybullet.getUserDataId(plane_id, 0, "MyKey4"))

		pybullet.disconnect()
		pybullet.connect(pybullet.SHARED_MEMORY)

		self.assertEqual(uid1, pybullet.getUserDataId(plane_id, 0, "MyKey1"))
		self.assertEqual(uid2, pybullet.getUserDataId(plane_id, 0, "MyKey2"))
		self.assertEqual(uid3, pybullet.getUserDataId(plane_id, 0, "MyKey3"))
		self.assertEqual(uid4, pybullet.getUserDataId(plane_id, 0, "MyKey4"))


	def testRemoveUserData(self):
		pybullet.connect(pybullet.SHARED_MEMORY)
		plane_id = pybullet.loadURDF(PLANE_PATH)
		uid1 = pybullet.addUserData(plane_id, 0, "MyKey1", "MyValue1")
		uid2 = pybullet.addUserData(plane_id, 0, "MyKey2", "MyValue2")
		uid3 = pybullet.addUserData(plane_id, 0, "MyKey3", "MyValue3")
		uid4 = pybullet.addUserData(plane_id, 0, "MyKey4", "MyValue4")

		pybullet.removeUserData(plane_id, 0, uid2)

		self.assertEqual(3, pybullet.getNumUserData(plane_id, 0))
		self.assertEqual(-1, pybullet.getUserDataId(plane_id, 0, "MyKey2"))
		self.assertEqual(None, pybullet.getUserData(plane_id, 0, uid2))
		self.assertEqual(b"MyValue1", pybullet.getUserData(plane_id, 0, uid1))
		self.assertEqual(b"MyValue3", pybullet.getUserData(plane_id, 0, uid3))
		self.assertEqual(b"MyValue4", pybullet.getUserData(plane_id, 0, uid4))

		pybullet.disconnect()
		pybullet.connect(pybullet.SHARED_MEMORY)

		self.assertEqual(3, pybullet.getNumUserData(plane_id, 0))
		self.assertEqual(-1, pybullet.getUserDataId(plane_id, 0, "MyKey2"))
		self.assertEqual(None, pybullet.getUserData(plane_id, 0, uid2))
		self.assertEqual(b"MyValue1", pybullet.getUserData(plane_id, 0, uid1))
		self.assertEqual(b"MyValue3", pybullet.getUserData(plane_id, 0, uid3))
		self.assertEqual(b"MyValue4", pybullet.getUserData(plane_id, 0, uid4))


	def testIterateAllUserData(self):
		pybullet.connect(pybullet.SHARED_MEMORY)
		plane_id = pybullet.loadURDF(PLANE_PATH)
		uid1 = pybullet.addUserData(plane_id, 0, "MyKey1", "MyValue1")
		uid2 = pybullet.addUserData(plane_id, 0, "MyKey2", "MyValue2")
		uid3 = pybullet.addUserData(plane_id, 0, "MyKey3", "MyValue3")
		uid4 = pybullet.addUserData(plane_id, 0, "MyKey4", "MyValue4")

		entries = set()
		for i in range(pybullet.getNumUserData(plane_id, 0)):
			userDataId, key = pybullet.getUserDataInfo(plane_id, 0, i)
			value = pybullet.getUserData(plane_id, 0, userDataId);
			entries.add((userDataId, key, value))

		self.assertTrue((uid1, b"MyKey1", b"MyValue1") in entries)
		self.assertTrue((uid2, b"MyKey2", b"MyValue2") in entries)
		self.assertTrue((uid3, b"MyKey3", b"MyValue3") in entries)
		self.assertTrue((uid4, b"MyKey4", b"MyValue4") in entries)
		self.assertEqual(4, len(entries))


	def testRemoveBody(self):
		pybullet.connect(pybullet.SHARED_MEMORY)
		plane_id = pybullet.loadURDF(PLANE_PATH)
		uid1 = pybullet.addUserData(plane_id, 0, "MyKey1", "MyValue1")
		uid2 = pybullet.addUserData(plane_id, 0, "MyKey2", "MyValue2")
		uid3 = pybullet.addUserData(plane_id, 0, "MyKey3", "MyValue3")
		uid4 = pybullet.addUserData(plane_id, 0, "MyKey4", "MyValue4")

		pybullet.removeBody(plane_id)
		self.assertEqual(None, pybullet.getUserData(plane_id, 0, uid1))
		self.assertEqual(None, pybullet.getUserData(plane_id, 0, uid2))
		self.assertEqual(None, pybullet.getUserData(plane_id, 0, uid3))
		self.assertEqual(None, pybullet.getUserData(plane_id, 0, uid4))

		pybullet.disconnect()
		pybullet.connect(pybullet.SHARED_MEMORY)

		self.assertEqual(None, pybullet.getUserData(plane_id, 0, uid1))
		self.assertEqual(None, pybullet.getUserData(plane_id, 0, uid2))
		self.assertEqual(None, pybullet.getUserData(plane_id, 0, uid3))
		self.assertEqual(None, pybullet.getUserData(plane_id, 0, uid4))

	def testMultipleBodies(self):
		pybullet.connect(pybullet.SHARED_MEMORY)
		plane1 = pybullet.loadURDF(PLANE_PATH)
		plane2 = pybullet.loadURDF(PLANE_PATH)
		
		uid1 = pybullet.addUserData(plane1, 0, "MyKey1", "This is plane 1 - 1")
		uid2 = pybullet.addUserData(plane1, 0, "MyKey2", "This is plane 1 - 2")

		uid3 = pybullet.addUserData(plane2, 0, "MyKey1", "This is plane 2 - 1")
		uid4 = pybullet.addUserData(plane2, 0, "MyKey2", "This is plane 2 - 2")
		uid5 = pybullet.addUserData(plane2, 0, "MyKey3", "This is plane 2 - 3")

		self.assertEqual(b"This is plane 1 - 1", pybullet.getUserData(plane1, 0, pybullet.getUserDataId(plane1, 0, "MyKey1")))
		self.assertEqual(b"This is plane 1 - 2", pybullet.getUserData(plane1, 0, pybullet.getUserDataId(plane1, 0, "MyKey2")))

		self.assertEqual(b"This is plane 2 - 1", pybullet.getUserData(plane2, 0, pybullet.getUserDataId(plane2, 0, "MyKey1")))
		self.assertEqual(b"This is plane 2 - 2", pybullet.getUserData(plane2, 0, pybullet.getUserDataId(plane2, 0, "MyKey2")))
		self.assertEqual(b"This is plane 2 - 3", pybullet.getUserData(plane2, 0, pybullet.getUserDataId(plane2, 0, "MyKey3")))


	def testMultipleLinks(self):
		pybullet.connect(pybullet.SHARED_MEMORY)

		body_id = pybullet.loadURDF(ROBOT_PATH)
		num_links = pybullet.getNumJoints(body_id)

		self.assertTrue(num_links > 1)

		for link_index in range(num_links):
			uid1 = pybullet.addUserData(body_id, link_index, "MyKey1", "Value1 for link %s" % link_index)
			uid2 = pybullet.addUserData(body_id, link_index, "MyKey2", "Value2 for link %s" % link_index)

		for link_index in range(num_links):
			uid1 = pybullet.getUserDataId(body_id, link_index, "MyKey1")
			uid2 = pybullet.getUserDataId(body_id, link_index, "MyKey2")
			self.assertEqual(("Value1 for link %s" % link_index).encode(), pybullet.getUserData(body_id, link_index, uid1))
			self.assertEqual(("Value2 for link %s" % link_index).encode(), pybullet.getUserData(body_id, link_index, uid2))

	def testMultipleClients(self):
		client1 = self.client_id
		client2 = pybullet.connect(pybullet.SHARED_MEMORY)

		plane_id = pybullet.loadURDF(PLANE_PATH, physicsClientId=client1)
		pybullet.syncBodyInfo(physicsClientId=client2)

		# Add user data on client 1, check on client 1
		uid = pybullet.addUserData(plane_id, 0, "MyKey", "MyValue", physicsClientId=client1)
		self.assertEqual(None, pybullet.getUserData(plane_id, 0, uid, physicsClientId=client2))
		pybullet.syncUserData(physicsClientId=client2)
		self.assertEqual(b"MyValue", pybullet.getUserData(plane_id, 0, uid, physicsClientId=client2))

		# Overwrite the value on client 2, check on client 1
		pybullet.addUserData(plane_id, 0, "MyKey", "MyNewValue", physicsClientId=client2)
		self.assertEqual(b"MyValue", pybullet.getUserData(plane_id, 0, uid, physicsClientId=client1))
		pybullet.syncUserData(physicsClientId=client1)
		self.assertEqual(b"MyNewValue", pybullet.getUserData(plane_id, 0, uid, physicsClientId=client1))

		# Remove user data on client 1, check on client 2
		pybullet.removeUserData(plane_id, 0, uid, physicsClientId=client1)
		self.assertEqual(b"MyNewValue", pybullet.getUserData(plane_id, 0, uid, physicsClientId=client2))
		pybullet.syncUserData(physicsClientId=client2)
		self.assertEqual(None, pybullet.getUserData(plane_id, 0, uid, physicsClientId=client2))

		pybullet.disconnect(physicsClientId=client2)


if __name__ == "__main__":
    unittest.main()
