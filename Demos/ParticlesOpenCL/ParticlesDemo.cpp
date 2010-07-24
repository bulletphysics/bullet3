/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006 - 2009 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#define START_POS_X btScalar(0.f)
#define START_POS_Y btScalar(0.f)
#define START_POS_Z btScalar(0.f)
//#define START_POS_Y btScalar(40.f)
//#define START_POS_Z btScalar(40.f)
//#define START_POS_Y btScalar(0.4f)
//#define START_POS_Z btScalar(0.4f)
#define ARRAY_SIZE_X 32
#define ARRAY_SIZE_Y 32
//#define ARRAY_SIZE_Y 16
#define ARRAY_SIZE_Z 32
//16
//#define ARRAY_SIZE_Z 1
//#define DIST btScalar(2.f)
#define DIST (DEF_PARTICLE_RADIUS * 2.f)

#define STRESS_X  20
//#define STRESS_Y  200
#define STRESS_Y  640



		

///The 3 following lines include the CPU implementation of the kernels, keep them in this order.
#include "BulletMultiThreaded/btGpuDefines.h"
#include "BulletMultiThreaded/btGpuUtilsSharedDefs.h"
#include "BulletMultiThreaded/btGpuUtilsSharedCode.h"
#ifndef __APPLE__
#include <GL/glew.h>
#endif


#include "GL_DialogDynamicsWorld.h"
#include "GL_DialogWindow.h"



#include "BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"
#include "GLDebugFont.h"
#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include <stdio.h> //printf debugging
#include "shaders.h"

#include "ParticlesDemo.h"





btScalar gTimeStep = 0.5f;//btScalar(1./60.);

#define SCALING btScalar(1.f)

void ParticlesDemo::clientMoveAndDisplay()
{


	updateCamera();
	glDisable(GL_LIGHTING);
	glColor3f(1.f, 1.f, 1.f);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	glDisable(GL_TEXTURE_2D); // we always draw wireframe in this demo

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();

	renderme(); 

	if (m_dialogDynamicsWorld)
		m_dialogDynamicsWorld->draw(gTimeStep);

	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(gTimeStep,0);//ms / 1000000.f);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}

	

	ms = getDeltaTimeMicroseconds();

	glFlush();

	glutSwapBuffers();

}



void ParticlesDemo::displayCallback(void) {

	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	//if (m_dialogDynamicsWorld)
	//	m_dialogDynamicsWorld->draw(gTimeStep);

	glFlush();
	glutSwapBuffers();
}

class btNullBroadphase : public btBroadphaseInterface
{
public:
	btNullBroadphase()
	{
	}
	virtual ~btNullBroadphase() 
	{
	}
	virtual btBroadphaseProxy*	createProxy(  const btVector3& aabbMin,  const btVector3& aabbMax,int shapeType,void* userPtr, short int collisionFilterGroup,short int collisionFilterMask, btDispatcher* dispatcher,void* multiSapProxy)
	{
		return NULL;
	}
	virtual void	destroyProxy(btBroadphaseProxy* proxy,btDispatcher* dispatcher)
	{
	}
	virtual void	setAabb(btBroadphaseProxy* proxy,const btVector3& aabbMin,const btVector3& aabbMax, btDispatcher* dispatcher)
	{
	}
	virtual void	getAabb(btBroadphaseProxy* proxy,btVector3& aabbMin, btVector3& aabbMax ) const
	{
	}
	virtual void	rayTest(const btVector3& rayFrom,const btVector3& rayTo, btBroadphaseRayCallback& rayCallback, const btVector3& aabbMin=btVector3(0,0,0), const btVector3& aabbMax = btVector3(0,0,0))
	{
	}
	virtual void	calculateOverlappingPairs(btDispatcher* dispatcher)
	{
	}
	virtual	btOverlappingPairCache*	getOverlappingPairCache()
	{
		return NULL;
	}
	virtual	const btOverlappingPairCache*	getOverlappingPairCache() const
	{
		return NULL;
	}
	virtual void getBroadphaseAabb(btVector3& aabbMin,btVector3& aabbMax) const
	{
	}
	virtual void resetPool(btDispatcher* dispatcher)
	{
	}
	virtual void	printStats()
	{
	}
	virtual void	aabbTest(const btVector3& aabbMin, const btVector3& aabbMax, btBroadphaseAabbCallback& callback)
	{
	}
};



void	ParticlesDemo::initPhysics()
{
	
	setTexturing(false);
	setShadows(false);

//	setCameraDistance(80.f);
	setCameraDistance(3.0f);
//	m_cameraTargetPosition.setValue(50, 10, 0);
	m_cameraTargetPosition.setValue(0, 0, 0);
//	m_azi = btScalar(0.f);
//	m_ele = btScalar(0.f);
	m_azi = btScalar(45.f);
	m_ele = btScalar(30.f);
	setFrustumZPlanes(0.1f, 10.f);

	///collision configuration contains default setup for memory, collision setup

	btDefaultCollisionConstructionInfo dci;
	dci.m_defaultMaxPersistentManifoldPoolSize=50000;
	dci.m_defaultMaxCollisionAlgorithmPoolSize=50000;

	m_collisionConfiguration = new btDefaultCollisionConfiguration(dci);

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_pairCache = new (btAlignedAlloc(sizeof(btHashedOverlappingPairCache),16))btHashedOverlappingPairCache(); 


//	m_broadphase = new btDbvtBroadphase(m_pairCache);
	m_broadphase = new btNullBroadphase();

	///the default constraint solver
	m_solver = new btSequentialImpulseConstraintSolver();

	m_pWorld = new btParticlesDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration, 65536);

	m_dialogDynamicsWorld = new GL_DialogDynamicsWorld();
	GL_DialogWindow* settings = m_dialogDynamicsWorld->createDialog(50,0,280,280,"CPU fallback");
	
	m_pWorld->m_useCpuControls[0] = 0;
	GL_ToggleControl* ctrl = 0;
	m_pWorld->m_useCpuControls[SIMSTAGE_INTEGRATE_MOTION] = m_dialogDynamicsWorld->createToggle(settings,"Integrate Motion");
	m_pWorld->m_useCpuControls[SIMSTAGE_COMPUTE_CELL_ID] = m_dialogDynamicsWorld->createToggle(settings,"Compute Cell ID");
	m_pWorld->m_useCpuControls[SIMSTAGE_SORT_CELL_ID] = m_dialogDynamicsWorld->createToggle(settings,"Sort Cell ID");
	m_pWorld->m_useCpuControls[SIMSTAGE_FIND_CELL_START] = m_dialogDynamicsWorld->createToggle(settings,"Find Cell Start");
	m_pWorld->m_useCpuControls[SIMSTAGE_COLLIDE_PARTICLES] = m_dialogDynamicsWorld->createToggle(settings,"Collide Particles");
	

	for(int i = 1; i < SIMSTAGE_TOTAL; i++)
	{
		m_pWorld->m_useCpuControls[i]->m_active = false;
	}
#if defined(CL_PLATFORM_MINI_CL)
	// these kernels use barrier()
	m_pWorld->m_useCpuControls[SIMSTAGE_SORT_CELL_ID]->m_active = true; 
	m_pWorld->m_useCpuControls[SIMSTAGE_FIND_CELL_START]->m_active = true; 
#endif

#if defined(CL_PLATFORM_AMD)
	// these kernels use barrier()
	m_pWorld->m_useCpuControls[SIMSTAGE_SORT_CELL_ID]->m_active = true; 
	m_pWorld->m_useCpuControls[SIMSTAGE_FIND_CELL_START]->m_active = true; 
#endif


	m_dynamicsWorld = m_pWorld;

	m_pWorld->getSimulationIslandManager()->setSplitIslands(true);
	m_pWorld->setGravity(btVector3(0,-10.,0));
	m_pWorld->getSolverInfo().m_numIterations = 4;

	{
//		btCollisionShape* colShape = new btSphereShape(btScalar(1.0f));
/*	
		btCollisionShape* colShape = new btSphereShape(DEF_PARTICLE_RADIUS);
		m_collisionShapes.push_back(colShape);
		btTransform startTransform;
		startTransform.setIdentity();
		btScalar	mass(1.f);
		btVector3 localInertia(0,0,0);
		colShape->calculateLocalInertia(mass,localInertia);
		float start_x = START_POS_X - ARRAY_SIZE_X * DIST * btScalar(0.5f);
		float start_y = START_POS_Y - ARRAY_SIZE_Y * DIST * btScalar(0.5f);
		float start_z = START_POS_Z - ARRAY_SIZE_Z * DIST * btScalar(0.5f);
		startTransform.setOrigin(btVector3(start_x, start_y, start_z));
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,0,colShape,localInertia);
		rbInfo.m_startWorldTransform = startTransform;
		btRigidBody* body = new btRigidBody(rbInfo);
		m_pWorld->addRigidBody(body);
		*/

		init_scene_directly();
	}
	clientResetScene();
	m_pWorld->initDeviceData();
}

inline float frand(void){
    return (float)rand() / (float)RAND_MAX;
}

void ParticlesDemo::init_scene_directly()
{


	srand(1969);
	float start_x = -1+DEF_PARTICLE_RADIUS;//START_POS_X - ARRAY_SIZE_X * DIST * btScalar(0.5f);
	float start_y = -1+DEF_PARTICLE_RADIUS;//START_POS_Y - ARRAY_SIZE_Y * DIST * btScalar(0.5f);
	float start_z = -1+DEF_PARTICLE_RADIUS;//START_POS_Z - ARRAY_SIZE_Z * DIST * btScalar(0.5f);
	int numParticles = ARRAY_SIZE_X * ARRAY_SIZE_Y * ARRAY_SIZE_Z;
	m_pWorld->m_hPos.resize(numParticles);
	m_pWorld->m_hVel.resize(numParticles);
	
	btScalar spacing = 2 * DEF_PARTICLE_RADIUS;

	for(int z=0; z<ARRAY_SIZE_Z; z++) 
    {
        for(int y=0; y<ARRAY_SIZE_Y; y++) 
        {
            for(int x=0; x<ARRAY_SIZE_X; x++) 
            {
                int i = (z * ARRAY_SIZE_Y * ARRAY_SIZE_X) + (y * ARRAY_SIZE_X) + x;
                if (i < numParticles) 
                {
					btVector3 jitter = 0.01f * 0.03f * btVector3(frand(), frand(), frand());
					m_pWorld->m_hVel[i]= btVector3(0,0,0);
					m_pWorld->m_hPos[i].setX((spacing * x) + 2*DEF_PARTICLE_RADIUS -WORLD_SIZE+jitter.getX());
					m_pWorld->m_hPos[i].setY((spacing * y) + 2*DEF_PARTICLE_RADIUS -WORLD_SIZE+jitter.getY());
					m_pWorld->m_hPos[i].setZ((spacing * z) + 2*DEF_PARTICLE_RADIUS -WORLD_SIZE+jitter.getZ());
                }
            }
        }
    }

	m_pWorld->m_numParticles = numParticles;
	
}


void ParticlesDemo::clientResetScene()
{
	static bool bFirstCall = true;
	DemoApplication::clientResetScene();
	init_scene_directly();
	if(bFirstCall)
	{
		bFirstCall = false;
	}
	else
	{
	
		m_pWorld->grabSimulationData();
	}
}


void	ParticlesDemo::exitPhysics()
{
	delete m_dialogDynamicsWorld;
	m_dialogDynamicsWorld = 0;

	//cleanup in the reverse order of creation/initialization
	int i;

	//remove the rigidbodies from the dynamics world and delete them
	for (i=m_pWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_pWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_pWorld->removeCollisionObject( obj );
		delete obj;
	}
	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	delete m_pWorld;

	delete m_solver;
	
	delete m_broadphase;
	
	delete m_dispatcher;

	delete m_collisionConfiguration;
}


void ParticlesDemo::keyboardCallback(unsigned char key, int x, int y)
{
	(void)x;
	(void)y;
	switch (key) 
	{
		case 'G' :
			{
				m_drawGridMode++;
				m_drawGridMode %= 3;
			}
			break;
		case 'q' : 
			exitPhysics();
			exit(0);
			break;
		default : 
			{
				DemoApplication::keyboardCallback(key, x, y);
			}
			break;
	}
	
	if(key == ' ')
	{
	}
}



void ParticlesDemo::renderme()
{

    glColor3f(1.0, 1.0, 1.0);
	glutWireCube(2*WORLD_SIZE);

	glPointSize(5.0f);
	glEnable(GL_POINT_SPRITE_ARB);

	glTexEnvi(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);
#ifndef __APPLE__
//	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE_NV);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
#endif //__APPLE__
	
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);

	glUseProgram(m_shaderProgram);

	btScalar dist = (m_glutScreenWidth > m_glutScreenHeight) ? m_glutScreenHeight : m_glutScreenWidth;
	glUniform1f( glGetUniformLocation(m_shaderProgram, "pointScale"), dist  );
//	glUniform1f( glGetUniformLocation(m_shaderProgram, "pointRadius"), 0.5f );
	int numParticles = m_pWorld->getNumParticles();
	int	col_vbo = m_pWorld->m_colVbo;
	int curr_vbo = m_pWorld->m_vbo;
	float sphere_rad = m_pWorld->m_particleRad;

	glUniform1f( glGetUniformLocation(m_shaderProgram, "pointRadius"), sphere_rad );
	glColor3f(1, 1, 1);

	// render from the vbo
    glBindBuffer(GL_ARRAY_BUFFER, curr_vbo);
    glVertexPointer(4, GL_FLOAT, 0, 0);
    glEnableClientState(GL_VERTEX_ARRAY);
    if(col_vbo) 
	{
		glBindBufferARB(GL_ARRAY_BUFFER_ARB, col_vbo);
		glColorPointer(4, GL_FLOAT, 0, 0);
		glEnableClientState(GL_COLOR_ARRAY);
	}
	glDrawArrays(GL_POINTS, 0, numParticles);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY); 
	glUseProgram(0);
	glDisable(GL_POINT_SPRITE_ARB);
	glBindBufferARB(GL_ARRAY_BUFFER,0);
	if(m_drawGridMode)
	{
		btVector3& wmin =  m_pWorld->m_worldMin;
		btVector3& wmax =  m_pWorld->m_worldMax;
		glBegin(GL_LINE_LOOP);
		glVertex3f(wmin[0], wmin[1], wmin[2]);
		glVertex3f(wmin[0], wmax[1], wmin[2]);
		glVertex3f(wmax[0], wmax[1], wmin[2]);
		glVertex3f(wmax[0], wmin[1], wmin[2]);
		glVertex3f(wmax[0], wmin[1], wmax[2]);
		glVertex3f(wmax[0], wmax[1], wmax[2]);
		glVertex3f(wmin[0], wmax[1], wmax[2]);
		glVertex3f(wmin[0], wmin[1], wmax[2]);
		glEnd();
		glBegin(GL_LINES);
		glVertex3f(wmin[0], wmin[1], wmin[2]);
		glVertex3f(wmax[0], wmin[1], wmin[2]);
		glVertex3f(wmin[0], wmin[1], wmax[2]);
		glVertex3f(wmax[0], wmin[1], wmax[2]);
		glVertex3f(wmin[0], wmax[1], wmin[2]);
		glVertex3f(wmin[0], wmax[1], wmax[2]);
		glVertex3f(wmax[0], wmax[1], wmin[2]);
		glVertex3f(wmax[0], wmax[1], wmax[2]);
		glEnd();
		if(m_drawGridMode == 2)
		{
			int szx = m_pWorld->m_simParams.m_gridSize[0];
			int szy = m_pWorld->m_simParams.m_gridSize[1];
			glBegin(GL_LINES);
			for(int i = 1; i < (szx-1); i++)
			{
				float wgt = (float)i / (float)(szx-1);
				btVector3 vtx = wmax * wgt + wmin * (1.0f - wgt); 
				glVertex3f(vtx[0], wmin[1], wmin[2]);
				glVertex3f(vtx[0], wmax[1], wmin[2]);
			}
			for(int i = 1; i < (szy-1); i++)
			{
				float wgt = (float)i / (float)(szy-1);
				btVector3 vtx = wmax * wgt + wmin * (1.0f - wgt); 
				glVertex3f(wmin[0], vtx[1], wmin[2]);
				glVertex3f(wmax[0], vtx[1], wmin[2]);
			}
		glEnd();
		}
	}

	if ((m_debugMode & btIDebugDraw::DBG_NoHelpText)==0)
	{
		setOrthographicProjection();
		int  xOffset = 10.f;
		int  yStart = 20.f;
		int  yIncr = 20.f;
		showProfileInfo(xOffset, yStart, yIncr);
		outputDebugInfo(xOffset, yStart, yIncr);
		resetPerspectiveProjection();
	}
}



void ParticlesDemo::outputDebugInfo(int & xOffset,int & yStart, int  yIncr)
{
	char buf[124];
	glDisable(GL_LIGHTING);
	glColor3f(0, 0, 0);
	
	sprintf(buf,"mouse move+buttons to interact");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"space to reset");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"cursor keys and z,x to navigate");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"i to toggle simulation, s single step");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"q to quit");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"h to toggle help text");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	
	sprintf(buf,"p to toggle profiling (+results to file)");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	sprintf(buf,"j to toggle between demos (integration/OECake2D/OECake3D)");
	GLDebugDrawString(xOffset,yStart,buf);
	yStart += yIncr;

	{
		sprintf(buf,"G to draw broadphase grid");
		GLDebugDrawString(xOffset,yStart,buf);
		yStart += yIncr;
		sprintf(buf,"D and U to toggle between GPU and CPU");
		GLDebugDrawString(xOffset,yStart,buf);
		yStart += yIncr;
		
	}
	
}


GLuint _compileProgram(const char *vsource, const char *fsource)
{
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

    glShaderSource(vertexShader, 1, &vsource, 0);
    glShaderSource(fragmentShader, 1, &fsource, 0);
    
    glCompileShader(vertexShader);
    glCompileShader(fragmentShader);

    GLuint program = glCreateProgram();

    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);

    glLinkProgram(program);

    // check if program linked
    GLint success = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &success);

    if (!success) {
        char temp[256];
        glGetProgramInfoLog(program, 256, 0, temp);
        printf("Failed to link program:\n%s\n", temp);
        glDeleteProgram(program);
        program = 0;
    }
    return program;
}


void ParticlesDemo::myinit()
{
	DemoApplication::myinit();
#ifndef __APPLE__
    glewInit();
    if (!glewIsSupported("GL_VERSION_2_0 GL_VERSION_1_5 GL_ARB_multitexture GL_ARB_vertex_buffer_object")) {
        fprintf(stderr, "Required OpenGL extensions missing.");
        exit(-1);
    }
#endif //__APPLE__
	
	m_shaderProgram = _compileProgram(vertexShader, spherePixelShader);
	m_pWorld->initCLKernels(m_argc, m_argv);
}






void ParticlesDemo::mouseFunc(int button, int state, int x, int y)
{

	if (!m_dialogDynamicsWorld->mouseFunc(button,state,x,y))
	{
		DemoApplication::mouseFunc(button,state,x,y);
	}
}

void	ParticlesDemo::mouseMotionFunc(int x,int y)
{
	m_dialogDynamicsWorld->mouseMotionFunc(x,y);
	DemoApplication::mouseMotionFunc(x,y);
}



void ParticlesDemo::reshape(int w, int h)
{
	if (m_dialogDynamicsWorld)
		m_dialogDynamicsWorld->setScreenSize(w,h);
	GlutDemoApplication::reshape(w,h);
}

