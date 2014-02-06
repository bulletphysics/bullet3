#include "LuaDemo.h"
#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include <iostream>

#include "BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h"

extern "C" {
	#include "lua.h"
	#include "lualib.h"
#include "lauxlib.h"
}


char* sLuaFileName = "init_physics.lua";

static const float scaling=0.35f;
static LuaDemo* sLuaDemo = 0;

static btVector4 colors[4] =
{
	btVector4(1,0,0,1),
	btVector4(0,1,0,1),
	btVector4(0,1,1,1),
	btVector4(1,1,0,1),
};

//todo: allow to create solver, broadphase, multiple worlds etc.
static int createDefaultDynamicsWorld(lua_State *L)
{
	sLuaDemo->m_config = new btDefaultCollisionConfiguration;
	sLuaDemo->m_dispatcher = new btCollisionDispatcher(sLuaDemo->m_config);
	sLuaDemo->m_bp = new btDbvtBroadphase();
	sLuaDemo->m_solver = new btNNCGConstraintSolver();
	sLuaDemo->m_dynamicsWorld = new btDiscreteDynamicsWorld(sLuaDemo->m_dispatcher,sLuaDemo->m_bp,sLuaDemo->m_solver,sLuaDemo->m_config);
	lua_pushlightuserdata (L, sLuaDemo->m_dynamicsWorld);
	return 1;
}


static int deleteDynamicsWorld(lua_State *L)
{
	return 0;
}

ATTRIBUTE_ALIGNED16(struct) CustomShapeData
{
	btVector3 m_localScaling;
	int	m_shapeIndex;
	
	
};


ATTRIBUTE_ALIGNED16(struct) CustomRigidBodyData
{
	int m_graphicsInstanceIndex;
};

static int createCubeShape(lua_State *L)
{
	int argc = lua_gettop(L);
	if (argc==4)
	{
		btVector3 halfExtents(1,1,1);
		if (!lua_isuserdata(L,1))
		{
			std::cerr << "error: first argument to createCubeShape should be world";
			return 0;
		}
		//expect userdata = sLuaDemo->m_dynamicsWorld
		halfExtents = btVector3(lua_tonumber(L,2),lua_tonumber(L,3),lua_tonumber(L,4));
		btCollisionShape* colShape = new btBoxShape(halfExtents);
		
		CustomShapeData* shapeData = new CustomShapeData();
		shapeData->m_shapeIndex = sLuaDemo->m_glApp->registerCubeShape();
		shapeData->m_localScaling = halfExtents;

		colShape->setUserPointer(shapeData);
		lua_pushlightuserdata (L, colShape);
		return 1;
	} else
	{
		std::cerr << "Error: invalid number of arguments to createCubeShape, expected 4 (world,halfExtentsX,halfExtentsY,halfExtentsX) but got " << argc;
	}
	return 0;
}

static int createSphereShape(lua_State *L)
{
	int argc = lua_gettop(L);
	if (argc==2)
	{
		btVector3 halfExtents(1,1,1);
		if (!lua_isuserdata(L,1))
		{
			std::cerr << "error: first argument to createSphereShape should be world";
			return 0;
		}
		//expect userdata = sLuaDemo->m_dynamicsWorld
		btScalar radius = lua_tonumber(L,2);
		btCollisionShape* colShape = new btSphereShape(radius);
		
		CustomShapeData* shapeData = new CustomShapeData();
		shapeData->m_shapeIndex = sLuaDemo->m_glApp->registerGraphicsSphereShape(radius,false,100,0.5);
		shapeData->m_localScaling = halfExtents;

		colShape->setUserPointer(shapeData);
		lua_pushlightuserdata (L, colShape);
		return 1;
	} else
	{
		std::cerr << "Error: invalid number of arguments to createSphereShape, expected 2 (world,radius) but got " << argc;
	}
	return 0;
}

int luaL_returnlen(lua_State* L, int index) 
{ 
  lua_len(L, index); 
  int len = lua_tointeger(L,-1); 
  lua_pop(L, 1); 
  return len; 
} 

btVector3 getLuaVectorArg(lua_State* L, int index)
{
	btVector3 pos(0,0,0);

	int sz = luaL_returnlen(L, index);  // get size of table
	{
		lua_rawgeti(L, index, 1);  // push t[i]
		pos[0] = lua_tonumber(L,-1); 
		lua_pop(L, 1);
		lua_rawgeti(L, index, 2);  // push t[i]
		pos[1] = lua_tonumber(L,-1); 
		lua_pop(L, 1);
		lua_rawgeti(L, index, 3);  // push t[i]
		pos[2] = lua_tonumber(L,-1); 
		lua_pop(L, 1);
	}
	return pos;
}

btQuaternion getLuaQuaternionArg(lua_State* L, int index)
{
	btQuaternion orn(0,0,0,1);

	int sz = luaL_returnlen(L, index);  // get size of table
	{
		lua_rawgeti(L, index, 1);  // push t[i]
		orn[0] = lua_tonumber(L,-1); 
		lua_pop(L, 1);
		lua_rawgeti(L, index, 2);  // push t[i]
		orn[1] = lua_tonumber(L,-1); 
		lua_pop(L, 1);
		lua_rawgeti(L, index, 3);  // push t[i]
		orn[2] = lua_tonumber(L,-1); 
		lua_pop(L, 1);
		lua_rawgeti(L, index, 4);  // push t[i]
		orn[3] = lua_tonumber(L,-1); 
		lua_pop(L, 1);
	}
	return orn;
}




static int createRigidBody (lua_State *L)
{
	int argc = lua_gettop(L);
	if (argc==5)
	{

		btTransform startTransform;
		startTransform.setIdentity();


		if (!lua_isuserdata(L,1))
		{
			std::cerr << "error: first argument to b3CreateRigidbody should be world";
			return 0;
		}
		btDiscreteDynamicsWorld* world = (btDiscreteDynamicsWorld*) lua_touserdata(L,1);
		if (world != sLuaDemo->m_dynamicsWorld)
		{
			std::cerr << "error: first argument expected to be a world";
			return 0;
		}

		if (!lua_isuserdata(L,2))
		{
			std::cerr << "error: second argument to b3CreateRigidbody should be world";
			return 0;
		}

		btScalar mass = lua_tonumber(L,3);

		luaL_checktype(L,4, LUA_TTABLE);

		btVector3 pos = getLuaVectorArg(L,4);
		
		btQuaternion orn = getLuaQuaternionArg(L,5);

		btCollisionShape* colShape = (btCollisionShape* )lua_touserdata(L,2);
		//expect userdata = sLuaDemo->m_dynamicsWorld
		
		btVector3 inertia(0,0,0);
		if (mass)
		{
			colShape->calculateLocalInertia(mass,inertia);
		}

	

		btRigidBody* body = new btRigidBody(mass,0,colShape,inertia);
		body->getWorldTransform().setOrigin(pos);
		body->getWorldTransform().setRotation(orn);
		

		CustomShapeData* shapeData = (CustomShapeData*)colShape->getUserPointer();
		if (shapeData)
		{
			CustomRigidBodyData* rbd = new CustomRigidBodyData;
			static int curColor = 0;
			btVector4 color = colors[curColor];
			curColor++;
			curColor&=3;

			CustomShapeData* shapeData = (CustomShapeData*)body->getCollisionShape()->getUserPointer();
			if (shapeData)
			{

				rbd ->m_graphicsInstanceIndex = sLuaDemo->m_glApp->m_instancingRenderer->registerGraphicsInstance(shapeData->m_shapeIndex,startTransform.getOrigin(),startTransform.getRotation(),color,shapeData->m_localScaling);
				body->setUserPointer(rbd);
			}
		}

		world->addRigidBody(body);
		lua_pushlightuserdata (L, body);
		return 1;
	} else
	{
		std::cerr << "Error: invalid number of arguments to createRigidBody, expected 5 (world,shape,mass,pos,orn) but got " << argc;
	}
	return 0;
}

static int setBodyPosition(lua_State *L)
{
	int argc = lua_gettop(L);
	if (argc==3)
	{
		if (!lua_isuserdata(L,1))
		{
			std::cerr << "error: first argument needs to be a world";
			return 0;
		}
		if (!lua_isuserdata(L,2))
		{
			std::cerr << "error: second argument needs to be a body";
			return 0;
		}
		btRigidBody* body = (btRigidBody*)lua_touserdata(L,2);
		btVector3 pos = getLuaVectorArg(L,3);
		
		btTransform& tr = body ->getWorldTransform();
		tr.setOrigin(pos);
		body->setWorldTransform(tr);
	} else
	{
		std::cerr << "error: setBodyPosition expects 6 arguments like setBodyPosition(world,body,0,1,0)";
	}
	return 0;
}

static int setBodyOrientation(lua_State *L)
{
	int argc = lua_gettop(L);
	if (argc==3)
	{
		if (!lua_isuserdata(L,1))
		{
			std::cerr << "error: first argument needs to be a world";
			return 0;
		}
		if (!lua_isuserdata(L,2))
		{
			std::cerr << "error: second argument needs to be a body";
			return 0;
		}
		btRigidBody* body = (btRigidBody*)lua_touserdata(L,2);
		btQuaternion orn = getLuaQuaternionArg(L,3);
		btTransform& tr = body ->getWorldTransform();
		tr.setRotation(orn);
		body->setWorldTransform(tr);
	} else
	{
		std::cerr << "error: setBodyOrientation expects 3 arguments like setBodyOrientation(world,body,orn)";
	}
	return 0;
}

//b3CreateConvexShape(world, points)

//b3CreateHingeConstraint(world,bodyA,bodyB,...)





LuaDemo::LuaDemo(SimpleOpenGL3App* app)
:Bullet2RigidBodyDemo(app)
{
	sLuaDemo = this;
}

LuaDemo::~LuaDemo()
{
	sLuaDemo = 0;
}

static void report_errors(lua_State *L, int status)
{
  if ( status!=0 ) {
    std::cerr << "-- " << lua_tostring(L, -1) << std::endl;
    lua_pop(L, 1); // remove error message
  }
}


void	LuaDemo::initPhysics()
{

	
	const char* prefix[]={"./","./data/","../data/","../../data/","../../../data/","../../../../data/"};
	int numPrefixes = sizeof(prefix)/sizeof(const char*);
	char relativeFileName[1024];
	FILE* f=0;
	int result = 0;

	for (int i=0;!f && i<numPrefixes;i++)
	{
		sprintf(relativeFileName,"%s%s",prefix[i],sLuaFileName);
		f = fopen(relativeFileName,"rb");
	}
	if (f)
	{
		fclose(f);

		lua_State *L = luaL_newstate();

		luaopen_io(L); // provides io.*
		luaopen_base(L);
		luaopen_table(L);
		luaopen_string(L);
		luaopen_math(L);
		//luaopen_package(L);
		luaL_openlibs(L);

		 // make my_function() available to Lua programs
		lua_register(L, "createDefaultDynamicsWorld", createDefaultDynamicsWorld);
		lua_register(L, "deleteDynamicsWorld", deleteDynamicsWorld);
		lua_register(L, "createCubeShape", createCubeShape);
		lua_register(L, "createSphereShape", createSphereShape);

		lua_register(L, "createRigidBody", createRigidBody);
		lua_register(L, "setBodyPosition", setBodyPosition);
		lua_register(L, "setBodyOrientation", setBodyOrientation);
		
	

		int s = luaL_loadfile(L, relativeFileName);

		if ( s==0 ) {
		  // execute Lua program
		  s = lua_pcall(L, 0, LUA_MULTRET, 0);
		}

		report_errors(L, s);
		lua_close(L);
	} else
	{
		b3Error("Cannot find Lua file%s\n",sLuaFileName);
	}

#if 0
	int curColor=0;
	//create ground
	int cubeShapeId = m_glApp->registerCubeShape();
	float pos[]={0,0,0};
	float orn[]={0,0,0,1};
		

	

	{
		float halfExtents[]={scaling,scaling,scaling,1};
		btVector4 colors[4] =
		{
			btVector4(1,0,0,1),
			btVector4(0,1,0,1),
			btVector4(0,1,1,1),
			btVector4(1,1,0,1),
		};
		


		btTransform startTransform;
		startTransform.setIdentity();
		btScalar mass = 1.f;
		btVector3 localInertia;
		btBoxShape* colShape = new btBoxShape(btVector3(halfExtents[0],halfExtents[1],halfExtents[2]));
		colShape ->calculateLocalInertia(mass,localInertia);
		
		for (int k=0;k<3;k++)
		{
			for (int i=0;i<3;i++)
			{
				for(int j = 0;j<3;j++)
				{
					
					btVector4 color = colors[curColor];
					curColor++;
					curColor&=3;
					startTransform.setOrigin(btVector3(
										btScalar(2.0*scaling*i),
										btScalar(2.*scaling+2.0*scaling*k),
										btScalar(2.0*scaling*j)));

					m_glApp->m_instancingRenderer->registerGraphicsInstance(cubeShapeId,startTransform.getOrigin(),startTransform.getRotation(),color,halfExtents);
			
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
					btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
					btRigidBody* body = new btRigidBody(rbInfo);
					

					m_dynamicsWorld->addRigidBody(body);
				}
			}
		}
	}
#endif

	m_glApp->m_instancingRenderer->writeTransforms();
}
void	LuaDemo::exitPhysics()
{
	//todo: delete bodies, shapes, constraints

	if (m_dynamicsWorld)
	{
		delete m_dynamicsWorld;
		m_dynamicsWorld=0;
		delete m_solver;
		m_solver=0;
		delete m_bp;
		m_bp=0;
		delete m_dispatcher;
		m_dispatcher=0;
		delete m_config;
		m_config=0;
	}
}
void	LuaDemo::renderScene()
{
	//sync graphics -> physics world transforms
	if (m_dynamicsWorld)
	{
		for (int i=0;i<m_dynamicsWorld->getNumCollisionObjects();i++)
		{
			btVector3 pos = m_dynamicsWorld->getCollisionObjectArray()[i]->getWorldTransform().getOrigin();
			btQuaternion orn = m_dynamicsWorld->getCollisionObjectArray()[i]->getWorldTransform().getRotation();
			if (m_dynamicsWorld->getCollisionObjectArray()[i]->getUserPointer())
			{
				CustomRigidBodyData* rbd = (CustomRigidBodyData*)m_dynamicsWorld->getCollisionObjectArray()[i]->getUserPointer();
				
				
				m_glApp->m_instancingRenderer->writeSingleInstanceTransformToCPU(pos,orn,rbd->m_graphicsInstanceIndex);
			}
		}
		m_glApp->m_instancingRenderer->writeTransforms();
	}

	m_glApp->m_instancingRenderer->renderScene();
}

	
void	LuaDemo::stepSimulation(float dt)
{
	if (m_dynamicsWorld)
		m_dynamicsWorld->stepSimulation(dt);
}



