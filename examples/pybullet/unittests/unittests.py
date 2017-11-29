import unittest
import pybullet
import time

class TestPybulletMethods(unittest.TestCase):

    def test_import(self):
        import pybullet as p
        self.assertGreater(p.getAPIVersion(), 201700000)

    def test_connect_direct(self):
        import pybullet as p
        cid = p.connect(p.DIRECT)
        self.assertEqual(cid,0)
        p.disconnect()

    def test_loadurdf(self):
        import pybullet as p
        p.connect(p.DIRECT)
        ob = p.loadURDF("r2d2.urdf")
        self.assertEqual(ob,0)
        p.disconnect()

    def test_rolling_friction(self):
        import pybullet as p
        p.connect(p.DIRECT)
        p.loadURDF("plane.urdf")
        sphere = p.loadURDF("sphere2.urdf",[0,0,1])
        p.resetBaseVelocity(sphere,linearVelocity=[1,0,0])
        p.changeDynamics(sphere,-1,linearDamping=0,angularDamping=0)
        #p.changeDynamics(sphere,-1,rollingFriction=0)
        p.setGravity(0,0,-10)
        for i in range (1000):
          p.stepSimulation()
        vel = p.getBaseVelocity(sphere)
        self.assertLess(vel[0][0],1e-10)
        self.assertLess(vel[0][1],1e-10)
        self.assertLess(vel[0][2],1e-10)
        self.assertLess(vel[1][0],1e-10)
        self.assertLess(vel[1][1],1e-10)
        self.assertLess(vel[1][2],1e-10)
        p.disconnect()
       
if __name__ == '__main__':
    unittest.main()

