/*
 * Copyright 1993-2006 NVIDIA Corporation.  All rights reserved.
 *
 * NOTICE TO USER:   
 *
 * This source code is subject to NVIDIA ownership rights under U.S. and 
 * international Copyright laws.  
 *
 * NVIDIA MAKES NO REPRESENTATION ABOUT THE SUITABILITY OF THIS SOURCE 
 * CODE FOR ANY PURPOSE.  IT IS PROVIDED "AS IS" WITHOUT EXPRESS OR 
 * IMPLIED WARRANTY OF ANY KIND.  NVIDIA DISCLAIMS ALL WARRANTIES WITH 
 * REGARD TO THIS SOURCE CODE, INCLUDING ALL IMPLIED WARRANTIES OF 
 * MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE.   
 * IN NO EVENT SHALL NVIDIA BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL, 
 * OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS 
 * OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE 
 * OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE 
 * OR PERFORMANCE OF THIS SOURCE CODE.  
 *
 * U.S. Government End Users.  This source code is a "commercial item" as 
 * that term is defined at 48 C.F.R. 2.101 (OCT 1995), consisting  of 
 * "commercial computer software" and "commercial computer software 
 * documentation" as such terms are used in 48 C.F.R. 12.212 (SEPT 1995) 
 * and is provided to the U.S. Government only as a commercial end item.  
 * Consistent with 48 C.F.R.12.212 and 48 C.F.R. 227.7202-1 through 
 * 227.7202-4 (JUNE 1995), all U.S. Government End Users acquire the 
 * source code with only those rights set forth herein.
 */

#include "particleSystem.h"
#include "particleSystem.cuh"
#include "radixsort.cuh"
#include "particles_kernel.cuh"


#include <assert.h>
#include <math.h>
#include <memory.h>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <GL/glew.h>

#include <btBulletDynamicsCommon.h>

#include "btCudaBroadphase.h"


#ifndef CUDART_PI_F
#define CUDART_PI_F         3.141592654f
#endif


ParticleSystem::ParticleSystem(uint numParticles, uint3 gridSize) :
	m_simulationMode(SIMULATION_BULLET_CPU)//SIMULATION_CUDA)
{
	this->m_params.numBodies = numParticles;
	this->m_params.m_gridSize = gridSize;
	initializeBullet();
}

ParticleSystem::~ParticleSystem()
{
	finalizeBullet();
}
#include "../../Demos/OpenGL/GLDebugDrawer.h"

GLDebugDrawer debugDrawer;

void	ParticleSystem::initializeBullet()
{
	
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
//	m_broadphase = new btDbvtBroadphase();
	//m_broadphase = new btAxisSweep3(btVector3(-3,-3,-3),btVector3(3,3,3));
	m_broadphase = new btCudaBroadphase(m_params,m_params.numBodies+6);



	m_constraintSolver=new btSequentialImpulseConstraintSolver();
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_constraintSolver,m_collisionConfiguration);
	m_dynamicsWorld->setDebugDrawer(&debugDrawer);
	//debugDrawer.setDebugMode(btIDebugDraw::DBG_DrawPairs);


	m_dynamicsWorld->setGravity(100*btVector3(m_params.gravity.x,m_params.gravity.y,m_params.gravity.z));
	m_dynamicsWorld->getSolverInfo().m_numIterations=1;

	btBoxShape* worldBox = new btBoxShape(btVector3(m_params.worldSize.x/2,m_params.worldSize.y/2,m_params.worldSize.z/2));
	worldBox->setMargin(0.f);

	//create 6 static planes for the world cube
	btStaticPlaneShape* planeShape;
	btRigidBody* body;
	btVector3 worldSize();

	int i;

	

	btSphereShape* particleSphere = new btSphereShape(m_params.particleRadius);
	particleSphere->setMargin(0.0);
	btVector3 localInertia;
	particleSphere->calculateLocalInertia(1,localInertia);

	float* m_hPos = m_broadphase->getHposPtr();

	for (i=0;i<m_params.numBodies;i++)
	{
		btRigidBody::btRigidBodyConstructionInfo rbci(1.,0,particleSphere,localInertia);
		rbci.m_startWorldTransform.setOrigin(btVector3(m_hPos[i*4],m_hPos[i*4+1],m_hPos[i*4+2]));
		body = new btRigidBody(rbci);
		body->setActivationState(DISABLE_DEACTIVATION);
		m_bulletParticles.push_back(body);
		m_dynamicsWorld->addRigidBody(body);
	}

		reset(CONFIG_GRID);

/*	for (i=0;i<6;i++)
	{
		btVector4	planeEq;
		worldBox->getPlaneEquation(planeEq,i);

		planeShape = new btStaticPlaneShape(-planeEq,planeEq.getW());
		planeShape->setMargin(0.f);
		btRigidBody::btRigidBodyConstructionInfo rbci(0.f,0,planeShape);
		body = new btRigidBody(rbci);
		m_dynamicsWorld->addRigidBody(body);
	}
*/

}

void	ParticleSystem::finalizeBullet()
{
	delete m_dynamicsWorld;
	delete m_constraintSolver;
	delete m_broadphase;
	delete m_dispatcher ;
	delete m_collisionConfiguration;
}



void 
ParticleSystem::update(float deltaTime)
{
    assert(m_bInitialized);

	switch (m_simulationMode)
	{
		case SIMULATION_CUDA:
		{
			m_broadphase->quickHack(deltaTime);
			//todo
			break;
		}
		case SIMULATION_BULLET_CPU:
			{
				m_broadphase->integrate();


				///copy particles from device to main memory
				{
					float* hPosData = m_broadphase->copyBuffersFromDeviceToHost();
					float* m_hVel = m_broadphase->getHvelPtr();
					m_broadphase->copyBuffersFromHostToDevice();


					//sync transform and velocity from particle system to Bullet

					for (int i=0;i<m_params.numBodies;i++)
					{
						btTransform& trans = m_bulletParticles[i]->getWorldTransform();
						trans.setOrigin(btVector3(hPosData[i*4],hPosData[i*4+1],hPosData[i*4+2]));
						m_bulletParticles[i]->setLinearVelocity(btVector3(m_hVel[i*4],m_hVel[i*4+1],m_hVel[i*4+2])*10.);
					}
				}

				m_dynamicsWorld->stepSimulation(deltaTime);

/*				for (int i=0;i<m_numParticles;i++)
				{
					data[i*4+1] -= 0.001f;
					m_hVel[i*4]=0;
					m_hVel[i*4+1]=0;
					m_hVel[i*4+2]=0;
				}
				*/

				{
					float* hPosData = m_broadphase->copyBuffersFromDeviceToHost();
					float* m_hVel = m_broadphase->getHvelPtr();

					//sync transform and velocity from Bullet to particle system
					for (int i=0;i<m_params.numBodies;i++)
					{
						btTransform& trans = m_bulletParticles[i]->getWorldTransform();
						hPosData[i*4] = trans.getOrigin().getX();
						hPosData[i*4+1] = trans.getOrigin().getY();
						hPosData[i*4+2] = trans.getOrigin().getZ();

						m_hVel[i*4] = m_bulletParticles[i]->getLinearVelocity().getX()/10.f;
						m_hVel[i*4+1] = m_bulletParticles[i]->getLinearVelocity().getY()/10.f;
						m_hVel[i*4+2] = m_bulletParticles[i]->getLinearVelocity().getZ()/10.f;
					}

					m_broadphase->copyBuffersFromHostToDevice();
				}
	
				break;

			}



	default:
		{
			printf("unknown simulation method\n");
		}
	};

}



float*  ParticleSystem::getArray(ParticleArray array)
{
	return m_broadphase->getArray((btCudaBroadphase::ParticleArray)array);

}
void	ParticleSystem::debugDraw()
{
	glDisable(GL_DEPTH_TEST);
	m_dynamicsWorld->debugDrawWorld();
	glEnable(GL_DEPTH_TEST);
}


void ParticleSystem::reset(ParticleConfig config)
{
	m_broadphase->reset((btCudaBroadphase::ParticleConfig)config);
	for (int i=0;i<m_bulletParticles.size();i++)
	{
		m_bulletParticles[i]->setAngularVelocity(btVector3(0,0,0));
	}
					
}


void ParticleSystem::addSphere(int start, float *pos, float *vel, int r, float spacing)
{
	m_broadphase->addSphere(start,pos,vel,r,spacing);
}

unsigned int ParticleSystem::getCurrentReadBuffer() const
{
	return m_broadphase->getCurrentReadBuffer();
}
unsigned int ParticleSystem::getColorBuffer() const
{
	return m_broadphase->getColorBuffer();
}

void ParticleSystem::dumpGrid()
{
	return m_broadphase->dumpGrid();
}

void ParticleSystem::dumpParticles(uint start, uint count)
{
	m_broadphase->dumpParticles(start,count);
}

int   ParticleSystem::getNumParticles() const
{
	return m_params.numBodies;
}