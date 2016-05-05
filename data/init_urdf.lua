-- Very basic Lua script to create some Bullet objects. 
-- See also Demos3/AllBullet2Demos using Demos3/bullet2/LuaDemo

--right now we cannot interleave adding instances of different shapes, they have to be added in-order
--hence the two loops. this will be fixed soon

world = createDefaultDynamicsWorld()

cubeshape = createCubeShape(world, 30,30,1)
pos={0,0,-3.0}
orn = {0,0,0,1}
mass = 0
body = createRigidBody(world,cubeshape,mass,pos,orn)

pos={0,10,0}
orn = {0,0,0,1}
mb = loadMultiBodyFromUrdf(world,"r2d2.urdf", pos, orn);
pos={2,2,0}
orn = {0,0,0,1}
mb = loadMultiBodyFromUrdf(world,"r2d2.urdf", pos, orn);
