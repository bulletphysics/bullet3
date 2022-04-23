/*
 Bullet Continuous Collision Detection and Physics Library
 Copyright (c) 2019 Google Inc. http://bulletphysics.org
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */

#include "LoadDeformed.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btDeformableBodySolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include <stdio.h>  //printf debugging

#include "../CommonInterfaces/CommonDeformableBodyBase.h"
#include "../Utils/b3ResourcePath.h"
#include "../Utils/b3BulletDefaultFileIO.h"
#include <iostream>
#include <iomanip> 
#include <sstream>
#include <string.h>

struct CustomSoftBodyHelper : public btSoftBodyHelpers
{
	static std::string loadDeformableState(btAlignedObjectArray<btVector3>& qs, btAlignedObjectArray<btVector3>& vs, const char* filename, CommonFileIOInterface* fileIO);
	
};

static inline bool isSpace(const char c)
{
	return (c == ' ') || (c == '\t');
}
static inline bool isNewLine(const char c)
{
	return (c == '\r') || (c == '\n') || (c == '\0');
}
static inline float parseFloat(const char*& token)
{
	token += strspn(token, " \t");
	float f = (float)atof(token);
	token += strcspn(token, " \t\r");
	return f;
}
static inline void parseFloat3(
	float& x, float& y, float& z,
	const char*& token)
{
	x = parseFloat(token);
	y = parseFloat(token);
	z = parseFloat(token);
}


std::string CustomSoftBodyHelper::loadDeformableState(btAlignedObjectArray<btVector3>& qs, btAlignedObjectArray<btVector3>& vs, const char* filename, CommonFileIOInterface* fileIO)
{
	{
		qs.clear();
		vs.clear();
		std::string tmp = filename;
		std::stringstream err;
#ifdef USE_STREAM
		std::ifstream ifs(filename);
		if (!ifs)
		{
			err << "Cannot open file [" << filename << "]" << std::endl;
			return err.str();
		}
#else
		int fileHandle = fileIO->fileOpen(filename, "r");
		if (fileHandle < 0)
		{
			err << "Cannot open file [" << filename << "]" << std::endl;
			return err.str();
		}
#endif

		std::string name;

		int maxchars = 8192;              // Alloc enough size.
		std::vector<char> buf(maxchars);  // Alloc enough size.
		std::string linebuf;
		linebuf.reserve(maxchars);

#ifdef USE_STREAM
		while (ifs.peek() != -1)
#else
		char* line = 0;
		do
#endif
		{
			linebuf.resize(0);
#ifdef USE_STREAM
			safeGetline(ifs, linebuf);
#else
			char tmpBuf[1024];
			line = fileIO->readLine(fileHandle, tmpBuf, 1024);
			if (line)
			{
				linebuf = line;
			}
#endif
			// Trim newline '\r\n' or '\r'
			if (linebuf.size() > 0)
			{
				if (linebuf[linebuf.size() - 1] == '\n') linebuf.erase(linebuf.size() - 1);
			}
			if (linebuf.size() > 0)
			{
				if (linebuf[linebuf.size() - 1] == '\n') linebuf.erase(linebuf.size() - 1);
			}

			// Skip if empty line.
			if (linebuf.empty())
			{
				continue;
			}

			// Skip leading space.
			const char* token = linebuf.c_str();
			token += strspn(token, " \t");

			btAssert(token);
			if (token[0] == '\0') continue;  // empty line

			if (token[0] == '#') continue;  // comment line

			// q
			if (token[0] == 'q' && isSpace((token[1])))
			{
				token += 2;
				float x, y, z;
				parseFloat3(x, y, z, token);
				qs.push_back(btVector3(x, y, z));
				continue;
			}

			// v
			if (token[0] == 'v' && isSpace((token[1])))
			{
				token += 3;
				float x, y, z;
				parseFloat3(x, y, z, token);
				vs.push_back(btVector3(x, y, z));
				continue;
			}

			// Ignore unknown command.
		}
#ifndef USE_STREAM
		while (line)
			;
#endif

		if (fileHandle >= 0)
		{
			fileIO->fileClose(fileHandle);
		}
		return err.str();
	}
}


class LoadDeformed : public CommonDeformableBodyBase
{
	int steps;
	btSoftBody* psb;
	char filename;
	int reset_frame;
	float sim_time;

public:
	LoadDeformed(struct GUIHelperInterface* helper)
		: CommonDeformableBodyBase(helper)
	{
		steps = 0;
		psb = nullptr;
		reset_frame = 0;
		sim_time = 0;
	}
	virtual ~LoadDeformed()
	{
	}
	void initPhysics();

	void exitPhysics();

	void resetCamera()
	{
		float dist = 2;
		float pitch = -45;
		float yaw = 100;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

	void stepSimulation(float deltaTime)
	{
		steps++;
		sim_time += deltaTime;
		////        int seconds = 1/deltaTime;
		if (0)
		{
			//        if (reset_frame==0 && steps<100){
			////            printf("steps %d, seconds %d, steps/seconds %d\n", steps,seconds,steps/seconds);
			char filename[100];
			sprintf(filename, "%s_%d_%d.txt", "states", reset_frame, steps);
			btSoftBodyHelpers::writeState(filename, psb);
		}
		if (sim_time + reset_frame * 0.05 >= 5) exit(0);
		float internalTimeStep = 1. / 240.f;
		//        float internalTimeStep = 0.1f;
		m_dynamicsWorld->stepSimulation(deltaTime, deltaTime / internalTimeStep, internalTimeStep);
	}

	void addCloth(const btVector3& origin);

	virtual void renderScene()
	{
		CommonDeformableBodyBase::renderScene();
		btDeformableMultiBodyDynamicsWorld* deformableWorld = getDeformableDynamicsWorld();

		for (int i = 0; i < deformableWorld->getSoftBodyArray().size(); i++)
		{
			btSoftBody* psb = (btSoftBody*)deformableWorld->getSoftBodyArray()[i];
			{
				btSoftBodyHelpers::DrawFrame(psb, deformableWorld->getDebugDrawer());
				btSoftBodyHelpers::Draw(psb, deformableWorld->getDebugDrawer(), deformableWorld->getDrawFlags());
			}
		}
	}
};

void LoadDeformed::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();
	btDeformableBodySolver* deformableBodySolver = new btDeformableBodySolver();

	btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
	sol->setDeformableSolver(deformableBodySolver);
	m_solver = sol;

	m_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, deformableBodySolver);
	btVector3 gravity = btVector3(0, -9.8, 0);
	m_dynamicsWorld->setGravity(gravity);
	getDeformableDynamicsWorld()->getWorldInfo().m_gravity = gravity;
	getDeformableDynamicsWorld()->getWorldInfo().m_sparsesdf.setDefaultVoxelsz(0.25);

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	{
		///create a ground
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(150.), btScalar(2.5), btScalar(150.)));
		groundShape->setMargin(0.02);
		m_collisionShapes.push_back(groundShape);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, -3.5, 0));
		groundTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0));
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass, localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		body->setFriction(4);

		//add the ground to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}
	addCloth(btVector3(0, 1, 0));
	getDeformableDynamicsWorld()->setImplicit(false);
	getDeformableDynamicsWorld()->setLineSearch(false);
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void LoadDeformed::addCloth(const btVector3& origin)
// create a piece of cloth
{
	const btScalar s = 0.6;
	const btScalar h = 0;

	psb = btSoftBodyHelpers::CreatePatch(getDeformableDynamicsWorld()->getWorldInfo(), btVector3(-s, h, -2 * s),
										 btVector3(+s, h, -2 * s),
										 btVector3(-s, h, +2 * s),
										 btVector3(+s, h, +2 * s),
										 15, 30,
										 0, true, 0.0);

	psb->getCollisionShape()->setMargin(0.02);
	psb->generateBendingConstraints(2);
	psb->setTotalMass(.5);
	psb->m_cfg.kKHR = 1;  // collision hardness with kinematic objects
	psb->m_cfg.kCHR = 1;  // collision hardness with rigid body
	psb->m_cfg.kDF = 0.1;
	psb->rotate(btQuaternion(0, SIMD_PI / 2, 0));
	btTransform clothTransform;
	clothTransform.setIdentity();
	clothTransform.setOrigin(btVector3(0, 0.2, 0) + origin);
	psb->transform(clothTransform);

	b3BulletDefaultFileIO fileio;
	char absolute_path[1024];
	char filename[100];
	sprintf(filename, "/Users/fuchuyuan/Documents/mybullet/build_cmake/examples/ExampleBrowser/states_0_%d.txt", reset_frame);
	fileio.findResourcePath(filename, absolute_path, 1024);
	btAlignedObjectArray<btVector3> qs;
	btAlignedObjectArray<btVector3> vs;
	CustomSoftBodyHelper::loadDeformableState(qs, vs, absolute_path, &fileio);
	if (reset_frame > 0)
		psb->updateState(qs, vs);
	psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
	psb->m_cfg.collisions |= btSoftBody::fCollision::SDF_MDF;
	psb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDF;
	//    psb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDN;
	psb->setCollisionFlags(0);
	psb->setCacheBarycenter(true);
	getDeformableDynamicsWorld()->addSoftBody(psb);
	psb->setSelfCollision(false);

	btDeformableMassSpringForce* mass_spring = new btDeformableMassSpringForce(2, 0.2, true);
	psb->setSpringStiffness(4);
	getDeformableDynamicsWorld()->addForce(psb, mass_spring);
	m_forces.push_back(mass_spring);
	btVector3 gravity = btVector3(0, -9.8, 0);
	btDeformableGravityForce* gravity_force = new btDeformableGravityForce(gravity);
	getDeformableDynamicsWorld()->addForce(psb, gravity_force);
	//    getDeformableDynamicsWorld()->setUseProjection(true);
	m_forces.push_back(gravity_force);
}

void LoadDeformed::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization
	removePickingConstraint();
	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}
	// delete forces
	for (int j = 0; j < m_forces.size(); j++)
	{
		btDeformableLagrangianForce* force = m_forces[j];
		delete force;
	}
	m_forces.clear();

	//delete collision shapes
	for (int j = 0; j < m_collisionShapes.size(); j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_dynamicsWorld;

	delete m_solver;

	delete m_broadphase;

	delete m_dispatcher;

	delete m_collisionConfiguration;
}

class CommonExampleInterface* LoadDeformedCreateFunc(struct CommonExampleOptions& options)
{
	return new LoadDeformed(options.m_guiHelper);
}
