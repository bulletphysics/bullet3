-- Very basic Lua script to create some Bullet objects. 
-- See also Demos3/AllBullet2Demos using Demos3/bullet2/LuaDemo

--right now we cannot interleave adding instances of different shapes, they have to be added in-order
--hence the two loops. this will be fixed soon

world = createDefaultDynamicsWorld()

cubeshape = createCubeShape(world, 30,1,30)
pos={0,0,0}
orn = {0,0,0,1}
mass = 0
body = createRigidBody(world,cubeshape,mass,pos,orn)

shape = createCubeShape(world, 1,1,1)

x=0
z=0
maxy = 10

toggle=1

for x=0,10 do
	for y=0,5 do
		if toggle==1 then
			toggle = 0
			for z=0,10 do
				mass = 1
				if (y==maxy) then
					--mass=30;
				end
				pos = {-14+x*2,2+2*y,z*2}
			
				body = createRigidBody(world,shape,mass,pos,orn)
				--setBodyPosition(world,body,pos)
				--setBodyOrientation(world,body,orn)
			end
		else
			toggle = 1
			
		end
	end
end

toggle=1
shape = createSphereShape(world, 1)

for x=0,10 do
	for y=0,5 do
		if toggle==1 then
			toggle = 0
		else
			toggle = 1
			for z=0,10 do
				mass = 1
				if (y==maxy) then
					--mass=30;
				end
			
			

				pos = {-14+x*2,2+2*y,z*2}
			
				body = createRigidBody(world,shape,mass,pos,orn)
			end
		end
	end
end
