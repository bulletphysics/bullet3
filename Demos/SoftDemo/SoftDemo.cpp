/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///btSoftBody implementation by Nathanael Presson


#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"
#include "BMF_Api.h"
#include "../GimpactTestDemo/BunnyMesh.h"
#include "../GimpactTestDemo/TorusMesh.h"
#include <stdio.h> //printf debugging
#include "LinearMath/btConvexHull.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

static float	gCollisionMargin = 0.05f/*0.05f*/;
#include "SoftDemo.h"
#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"

extern float eye[3];
extern int glutScreenWidth;
extern int glutScreenHeight;

const int maxProxies = 32766;
const int maxOverlap = 65535;



#ifdef _DEBUG
const int gNumObjects = 1;
#else
const int gNumObjects = 1;//try this in release mode: 3000. never go above 16384, unless you increate maxNumObjects  value in DemoApplication.cp
#endif

const int maxNumObjects = 32760;

#define CUBE_HALF_EXTENTS 1.5
#define EXTRA_HEIGHT -10.f

//
void SoftDemo::createStack( btCollisionShape* boxShape, float halfCubeSize, int size, float zPos )
{
	btTransform trans;
	trans.setIdentity();

	for(int i=0; i<size; i++)
	{
		// This constructs a row, from left to right
		int rowSize = size - i;
		for(int j=0; j< rowSize; j++)
		{
			btVector4 pos;
			pos.setValue(
				-rowSize * halfCubeSize + halfCubeSize + j * 2.0f * halfCubeSize,
				halfCubeSize + i * halfCubeSize * 2.0f,
				zPos);

			trans.setOrigin(pos);
			btScalar mass = 1.f;

			btRigidBody* body = 0;
			body = localCreateRigidBody(mass,trans,boxShape);

		}
	}
}


////////////////////////////////////

extern int gNumManifold;
extern int gOverlappingPairs;
extern int gTotalContactPoints;

void SoftDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 




	float dt = getDeltaTimeMicroseconds() * 0.000001f;

	//	printf("dt = %f: ",dt);


	if (m_dynamicsWorld)
	{
#define FIXED_STEP
#ifdef FIXED_STEP
		m_dynamicsWorld->stepSimulation(dt=1.0f/60.f,0);

#else
		//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = m_idle ? 1 : 1;
		if (m_idle)
			dt = 1.0/420.f;

		int numSimSteps = 0;
		numSimSteps = m_dynamicsWorld->stepSimulation(dt,maxSimSubSteps);		

#ifdef VERBOSE_TIMESTEPPING_CONSOLEOUTPUT
		if (!numSimSteps)
			printf("Interpolated transforms\n");
		else
		{
			if (numSimSteps > maxSimSubSteps)
			{
				//detect dropping frames
				printf("Dropped (%i) simulation steps out of %i\n",numSimSteps - maxSimSubSteps,numSimSteps);
			} else
			{
				printf("Simulated (%i) steps\n",numSimSteps);
			}
		}
#endif //VERBOSE_TIMESTEPPING_CONSOLEOUTPUT

#endif		


		m_softBodyWorldInfo.m_sparsesdf.GarbageCollect();

		//optional but useful: debug drawing
		
	}

#ifdef USE_QUICKPROF 
	btProfiler::beginBlock("render"); 
#endif //USE_QUICKPROF 

	renderme(); 

	//render the graphics objects, with center of mass shift

	updateCamera();



#ifdef USE_QUICKPROF 
	btProfiler::endBlock("render"); 
#endif 
	glFlush();
	//some additional debugging info
#ifdef PRINT_CONTACT_STATISTICS
	printf("num manifolds: %i\n",gNumManifold);
	printf("num gOverlappingPairs: %i\n",gOverlappingPairs);
	printf("num gTotalContactPoints : %i\n",gTotalContactPoints );
#endif //PRINT_CONTACT_STATISTICS

	gTotalContactPoints = 0;
	glutSwapBuffers();

}



void SoftDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 


	renderme();

	glFlush();
	glutSwapBuffers();
}


//
// ImplicitShape
//

//
struct	ImplicitSphere : btSoftBody::ImplicitFn
{
btVector3	center;
btScalar	sqradius;
			ImplicitSphere() {}
			ImplicitSphere(const btVector3& c,btScalar r) : center(c),sqradius(r*r) {}
btScalar	Eval(const btVector3& x)
	{
	return((x-center).length2()-sqradius);
	}
};

//
// Tetra meshes
//

struct	TetraBunny
{
#include "bunny.inl"
};

struct	TetraCube
{
#include "cube.inl"
};


//
// Random
//

static inline btScalar	UnitRand()
{
	return(rand()/(btScalar)RAND_MAX);
}

static inline btScalar	SignedUnitRand()
{
	return(UnitRand()*2-1);
}

static inline btVector3	Vector3Rand()
{
	const btVector3	p=btVector3(SignedUnitRand(),SignedUnitRand(),SignedUnitRand());
	return(p.normalized());
}

//
// Rb rain
//
static void	Ctor_RbUpStack(SoftDemo* pdemo,int count)
{
	float				mass=10;
	btCollisionShape*	shape[]={	new btSphereShape(1.5),
		new btBoxShape(btVector3(1,1,1)),
		new btCylinderShapeX(btVector3(4,1,1))};
	static const int	nshapes=sizeof(shape)/sizeof(shape[0]);
	for(int i=0;i<count;++i)
	{
		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(btVector3(0,1+6*i,0));
		btRigidBody*		body=pdemo->localCreateRigidBody(mass,startTransform,shape[i%nshapes]);
	}
}

//
// Big ball
//
static void	Ctor_BigBall(SoftDemo* pdemo,btScalar mass=10)
{
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0,13,0));
	btRigidBody*		body=pdemo->localCreateRigidBody(mass,startTransform,new btSphereShape(3));
}

//
// Big plate
//
static void	Ctor_BigPlate(SoftDemo* pdemo,btScalar mass=15)
{
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0,4,0.5));
	btRigidBody*		body=pdemo->localCreateRigidBody(mass,startTransform,new btBoxShape(btVector3(5,1,5)));
	body->setFriction(1);
}

//
// Linear stair
//
static void Ctor_LinearStair(SoftDemo* pdemo,const btVector3& org,const btVector3& sizes,btScalar angle,int count)
{
	btBoxShape*	shape=new btBoxShape(sizes);
	for(int i=0;i<count;++i)
	{
		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(org+btVector3(sizes.x()*i*2,sizes.y()*i*2,0));
		btRigidBody* body=pdemo->localCreateRigidBody(0,startTransform,shape);
		body->setFriction(1);
	}
}

//
// Softbox
//
static btSoftBody* Ctor_SoftBox(SoftDemo* pdemo,const btVector3& p,const btVector3& s)
{
	const btVector3	h=s*0.5;
	const btVector3	c[]={	p+h*btVector3(-1,-1,-1),
		p+h*btVector3(+1,-1,-1),
		p+h*btVector3(-1,+1,-1),
		p+h*btVector3(+1,+1,-1),
		p+h*btVector3(-1,-1,+1),
		p+h*btVector3(+1,-1,+1),
		p+h*btVector3(-1,+1,+1),
		p+h*btVector3(+1,+1,+1)};
	btSoftBody*		psb=btSoftBodyHelpers::CreateFromConvexHull(pdemo->m_softBodyWorldInfo,c,8);
	psb->generateBendingConstraints(2);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
	
	return(psb);
}

//
// SoftBoulder
//
static btSoftBody* Ctor_SoftBoulder(SoftDemo* pdemo,const btVector3& p,const btVector3& s,int np,int id)
{
	btAlignedObjectArray<btVector3>	pts;
	if(id) srand(id);
	for(int i=0;i<np;++i)
	{
		pts.push_back(Vector3Rand()*s+p);
	}
	btSoftBody*		psb=btSoftBodyHelpers::CreateFromConvexHull(pdemo->m_softBodyWorldInfo,&pts[0],pts.size());
	psb->generateBendingConstraints(2);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

	return(psb);
}

//#define TRACEDEMO { pdemo->demoname=__FUNCTION__+5;printf("Launching demo: " __FUNCTION__ "\r\n"); }

//
// Basic ropes
//
static void	Init_Ropes(SoftDemo* pdemo)
{
	//TRACEDEMO
	const int n=15;
	for(int i=0;i<n;++i)
	{
		btSoftBody*	psb=btSoftBodyHelpers::CreateRope(pdemo->m_softBodyWorldInfo,	btVector3(-10,0,i*0.25),
			btVector3(10,0,i*0.25),
			16,
			1+2);
		psb->m_cfg.piterations		=	4;
		psb->m_materials[0]->m_kLST	=	0.1+(i/(btScalar)(n-1))*0.9;
		psb->setTotalMass(20);
		pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
	
	}
}

//
// Rope attach
//
static void	Init_RopeAttach(SoftDemo* pdemo)
{
	//TRACEDEMO
	pdemo->m_softBodyWorldInfo.m_sparsesdf.RemoveReferences(0);
	struct	Functors
	{
		static btSoftBody* CtorRope(SoftDemo* pdemo,const btVector3& p)
		{
			btSoftBody*	psb=btSoftBodyHelpers::CreateRope(pdemo->m_softBodyWorldInfo,p,p+btVector3(10,0,0),8,1);
			psb->setTotalMass(50);
			pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
			return(psb);
		}
	};
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(12,8,0));
	btRigidBody*		body=pdemo->localCreateRigidBody(50,startTransform,new btBoxShape(btVector3(2,6,2)));
	btSoftBody*	psb0=Functors::CtorRope(pdemo,btVector3(0,8,-1));
	btSoftBody*	psb1=Functors::CtorRope(pdemo,btVector3(0,8,+1));
	psb0->appendAnchor(psb0->m_nodes.size()-1,body);
	psb1->appendAnchor(psb1->m_nodes.size()-1,body);
}

//
// Cloth attach
//
static void	Init_ClothAttach(SoftDemo* pdemo)
{
	//TRACEDEMO
	const btScalar	s=4;
	const btScalar	h=6;
	const int		r=9;
	btSoftBody*		psb=btSoftBodyHelpers::CreatePatch(pdemo->m_softBodyWorldInfo,btVector3(-s,h,-s),
		btVector3(+s,h,-s),
		btVector3(-s,h,+s),
		btVector3(+s,h,+s),r,r,4+8,true);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0,h,-(s+3.5)));
	btRigidBody*		body=pdemo->localCreateRigidBody(20,startTransform,new btBoxShape(btVector3(s,1,3)));
	psb->appendAnchor(0,body);
	psb->appendAnchor(r-1,body);
	pdemo->m_cutting=true;
}

//
// Impact
//
static void	Init_Impact(SoftDemo* pdemo)
{
	//TRACEDEMO
	btSoftBody*	psb=btSoftBodyHelpers::CreateRope(pdemo->m_softBodyWorldInfo,	btVector3(0,0,0),
		btVector3(0,-1,0),
		0,
		1);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
	psb->m_cfg.kCHR=0.5;
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0,20,0));
	btRigidBody*		body=pdemo->localCreateRigidBody(10,startTransform,new btBoxShape(btVector3(2,2,2)));
}

//
// Collide
//
static void	Init_Collide(SoftDemo* pdemo)
{
	//TRACEDEMO
	struct Functor
		{
		static btSoftBody* Create(SoftDemo* pdemo,const btVector3& x,const btVector3& a)
			{
			btSoftBody*	psb=btSoftBodyHelpers::CreateFromTriMesh(pdemo->m_softBodyWorldInfo,gVertices,
				&gIndices[0][0],
				NUM_TRIANGLES);
			psb->generateBendingConstraints(2);
			psb->m_cfg.piterations=2;
			psb->m_cfg.collisions|=btSoftBody::fCollision::VF_SS;
			psb->randomizeConstraints();
			btMatrix3x3	m;
			m.setEulerZYX(a.x(),a.y(),a.z());
			psb->transform(btTransform(m,x));
			psb->scale(btVector3(2,2,2));
			psb->setTotalMass(50,true);
			pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
			return(psb);
			}
		};
	for(int i=0;i<3;++i)
		{
		Functor::Create(pdemo,btVector3(3*i,2,0),btVector3(SIMD_PI/2*(1-(i&1)),SIMD_PI/2*(i&1),0));
		}
	pdemo->m_cutting=true;
}

//
// Collide2
//
static void	Init_Collide2(SoftDemo* pdemo)
{
	//TRACEDEMO
	struct Functor
		{
		static btSoftBody* Create(SoftDemo* pdemo,const btVector3& x,const btVector3& a)
			{
			btSoftBody*	psb=btSoftBodyHelpers::CreateFromTriMesh(pdemo->m_softBodyWorldInfo,gVerticesBunny,
				&gIndicesBunny[0][0],
				BUNNY_NUM_TRIANGLES);
			btSoftBody::Material*	pm=psb->appendMaterial();
			pm->m_kLST				=	0.5;
			pm->m_flags				-=	btSoftBody::fMaterial::DebugDraw;
			psb->generateBendingConstraints(2,pm);
			psb->m_cfg.piterations	=	2;
			psb->m_cfg.kDF			=	0.5;
			psb->m_cfg.collisions	|=	btSoftBody::fCollision::VF_SS;
			psb->randomizeConstraints();
			btMatrix3x3	m;
			m.setEulerZYX(a.x(),a.y(),a.z());
			psb->transform(btTransform(m,x));
			psb->scale(btVector3(6,6,6));
			psb->setTotalMass(100,true);
			pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
			return(psb);
			}
		};
	for(int i=0;i<3;++i)
		{
		Functor::Create(pdemo,btVector3(0,-1+5*i,0),btVector3(0,SIMD_PI/2*(i&1),0));
		}	
	pdemo->m_cutting=true;
}

//
// Collide3
//
static void	Init_Collide3(SoftDemo* pdemo)
{
	//TRACEDEMO
		{
		const btScalar	s=8;
		btSoftBody*		psb=btSoftBodyHelpers::CreatePatch(	pdemo->m_softBodyWorldInfo,btVector3(-s,0,-s),
			btVector3(+s,0,-s),
			btVector3(-s,0,+s),
			btVector3(+s,0,+s),
			15,15,1+2+4+8,true);
		psb->m_materials[0]->m_kLST	=	0.4;
		psb->m_cfg.collisions		|=	btSoftBody::fCollision::VF_SS;
		psb->setTotalMass(150);
		pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
		}
		{		
		const btScalar	s=4;
		const btVector3	o=btVector3(5,10,0);
		btSoftBody*		psb=btSoftBodyHelpers::CreatePatch(	pdemo->m_softBodyWorldInfo,
			btVector3(-s,0,-s)+o,
			btVector3(+s,0,-s)+o,
			btVector3(-s,0,+s)+o,
			btVector3(+s,0,+s)+o,
			7,7,0,true);
		btSoftBody::Material*	pm=psb->appendMaterial();
		pm->m_kLST				=	0.1;
		pm->m_flags				-=	btSoftBody::fMaterial::DebugDraw;
		psb->generateBendingConstraints(2,pm);
		psb->m_materials[0]->m_kLST	=	0.5;
		psb->m_cfg.collisions		|=	btSoftBody::fCollision::VF_SS;
		psb->setTotalMass(150);
		pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
		pdemo->m_cutting=true;
		}
}

//
// Aerodynamic forces, 50x1g flyers
//
static void	Init_Aero(SoftDemo* pdemo)
{
	//TRACEDEMO
	const btScalar	s=2;
	const btScalar	h=10;
	const int		segments=6;
	const int		count=50;
	for(int i=0;i<count;++i)
	{
		btSoftBody*		psb=btSoftBodyHelpers::CreatePatch(pdemo->m_softBodyWorldInfo,btVector3(-s,h,-s),
			btVector3(+s,h,-s),
			btVector3(-s,h,+s),
			btVector3(+s,h,+s),
			segments,segments,
			0,true);
		psb->generateBendingConstraints(2);
		psb->m_cfg.kLF			=	0.004;
		psb->m_cfg.kDG			=	0.0003;
		psb->m_cfg.aeromodel	=	btSoftBody::eAeroModel::V_TwoSided;
		btTransform		trs;
		btQuaternion	rot;
		btVector3		ra=Vector3Rand()*0.1;
		btVector3		rp=Vector3Rand()*15+btVector3(0,20,80);
		rot.setEuler(SIMD_PI/8+ra.x(),-SIMD_PI/7+ra.y(),ra.z());
		trs.setIdentity();
		trs.setOrigin(rp);
		trs.setRotation(rot);
		psb->transform(trs);
		psb->setTotalMass(0.1);
		psb->addForce(btVector3(0,2,0),0);
		pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

	}
	pdemo->m_autocam=true;
}

//
// Friction
//
static void	Init_Friction(SoftDemo* pdemo)
{
	//TRACEDEMO
	const btScalar	bs=2;
	const btScalar	ts=bs+bs/4;
	for(int i=0,ni=20;i<ni;++i)
	{
		const btVector3	p(-ni*ts/2+i*ts,-10+bs,40);
		btSoftBody*		psb=Ctor_SoftBox(pdemo,p,btVector3(bs,bs,bs));
		psb->m_cfg.kDF	=	0.1 * ((i+1)/(btScalar)ni);
		psb->addVelocity(btVector3(0,0,-10));
	}
}

//
// Pressure
//
static void	Init_Pressure(SoftDemo* pdemo)
{
	//TRACEDEMO
	btSoftBody*	psb=btSoftBodyHelpers::CreateEllipsoid(pdemo->m_softBodyWorldInfo,btVector3(35,25,0),
		btVector3(1,1,1)*3,
		512);
	psb->m_materials[0]->m_kLST	=	0.1;
	psb->m_cfg.kDF				=	1;
	psb->m_cfg.kDP				=	0.001; // fun factor...
	psb->m_cfg.kPR				=	2500;
	psb->setTotalMass(30,true);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

	Ctor_BigPlate(pdemo);
	Ctor_LinearStair(pdemo,btVector3(0,0,0),btVector3(2,1,5),0,10);
	pdemo->m_autocam=true;
}

//
// Volume conservation
//
static void	Init_Volume(SoftDemo* pdemo)
{
	//TRACEDEMO
	btSoftBody*	psb=btSoftBodyHelpers::CreateEllipsoid(pdemo->m_softBodyWorldInfo,btVector3(35,25,0),
		btVector3(1,1,1)*3,
		512);
	psb->m_materials[0]->m_kLST	=	0.45;
	psb->m_cfg.kVC				=	20;
	psb->setTotalMass(50,true);
	psb->setPose(true,false);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

	Ctor_BigPlate(pdemo);
	Ctor_LinearStair(pdemo,btVector3(0,0,0),btVector3(2,1,5),0,10);
	pdemo->m_autocam=true;
}

//
// Stick+Bending+Rb's
//
static void	Init_Sticks(SoftDemo* pdemo)
{
	//TRACEDEMO
	const int		n=16;
	const int		sg=4;
	const btScalar	sz=5;
	const btScalar	hg=4;
	const btScalar	in=1/(btScalar)(n-1);
	for(int y=0;y<n;++y)
	{
		for(int x=0;x<n;++x)
		{
			const btVector3	org(-sz+sz*2*x*in,
				-10,
				-sz+sz*2*y*in);
			btSoftBody*		psb=btSoftBodyHelpers::CreateRope(	pdemo->m_softBodyWorldInfo,	org,
				org+btVector3(hg*0.001,hg,0),
				sg,
				1);
			psb->m_cfg.kDP		=	0.005;
			psb->m_cfg.kCHR		=	0.1;
			for(int i=0;i<3;++i)
			{
				psb->generateBendingConstraints(2+i);
			}
			psb->setMass(1,0);
			psb->setTotalMass(0.01);
			pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

		}
	}
	Ctor_BigBall(pdemo);
}

//
// 100kg cloth locked at corners, 10 falling 10kg rb's.
//
static void	Init_Cloth(SoftDemo* pdemo)
{
	//TRACEDEMO
	const btScalar	s=8;
	btSoftBody*		psb=btSoftBodyHelpers::CreatePatch(	pdemo->m_softBodyWorldInfo,btVector3(-s,0,-s),
		btVector3(+s,0,-s),
		btVector3(-s,0,+s),
		btVector3(+s,0,+s),
		31,31,

		//		31,31,
		1+2+4+8,true);
	btSoftBody::Material* pm=psb->appendMaterial();
	pm->m_kLST		=	0.4;
	pm->m_flags		-=	btSoftBody::fMaterial::DebugDraw;
	psb->generateBendingConstraints(2,pm);
	psb->setTotalMass(150);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

	Ctor_RbUpStack(pdemo,10);
	pdemo->m_cutting=true;
}

//
// 100kg Stanford's bunny
//
static void	Init_Bunny(SoftDemo* pdemo)
{
	//TRACEDEMO
	btSoftBody*	psb=btSoftBodyHelpers::CreateFromTriMesh(pdemo->m_softBodyWorldInfo,gVerticesBunny,
		&gIndicesBunny[0][0],
		BUNNY_NUM_TRIANGLES);
	btSoftBody::Material*	pm=psb->appendMaterial();
	pm->m_kLST				=	0.5;
	pm->m_flags				-=	btSoftBody::fMaterial::DebugDraw;
	psb->generateBendingConstraints(2,pm);
	psb->m_cfg.piterations	=	2;
	psb->m_cfg.kDF			=	0.5;
	psb->randomizeConstraints();
	psb->scale(btVector3(6,6,6));
	psb->setTotalMass(100,true);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

}

//
// 100kg Stanford's bunny with pose matching
//
static void	Init_BunnyMatch(SoftDemo* pdemo)
{
	//TRACEDEMO
	btSoftBody*	psb=btSoftBodyHelpers::CreateFromTriMesh(pdemo->m_softBodyWorldInfo,	gVerticesBunny,
		&gIndicesBunny[0][0],
		BUNNY_NUM_TRIANGLES);
	psb->m_cfg.kDF				=	0.5;
	psb->m_materials[0]->m_kLST	=	0.1;
	psb->m_cfg.kMT				=	0.05;
	psb->randomizeConstraints();
	psb->scale(btVector3(6,6,6));
	psb->setTotalMass(100,true);
	psb->setPose(true,true);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);	

}

//
// 50Kg Torus
//
static void	Init_Torus(SoftDemo* pdemo)
{
	//TRACEDEMO
	btSoftBody*	psb=btSoftBodyHelpers::CreateFromTriMesh(	pdemo->m_softBodyWorldInfo,	gVertices,
		&gIndices[0][0],
		NUM_TRIANGLES);
	psb->generateBendingConstraints(2);
	psb->m_cfg.piterations=2;
	psb->randomizeConstraints();
	btMatrix3x3	m;
	m.setEulerZYX(SIMD_PI/2,0,0);
	psb->transform(btTransform(m,btVector3(0,4,0)));
	psb->scale(btVector3(2,2,2));
	psb->setTotalMass(50,true);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

}

//
// 50Kg Torus with pose matching
//
static void	Init_TorusMatch(SoftDemo* pdemo)
{
	//TRACEDEMO
	btSoftBody*	psb=btSoftBodyHelpers::CreateFromTriMesh(pdemo->m_softBodyWorldInfo,	gVertices,
		&gIndices[0][0],
		NUM_TRIANGLES);
	psb->m_materials[0]->m_kLST	=	0.1;
	psb->m_cfg.kMT				=	0.05;
	psb->randomizeConstraints();
	btMatrix3x3	m;
	m.setEulerZYX(SIMD_PI/2,0,0);
	psb->transform(btTransform(m,btVector3(0,4,0)));
	psb->scale(btVector3(2,2,2));
	psb->setTotalMass(50,true);
	psb->setPose(true,true);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
}

//
// Cutting1
//
static void	Init_Cutting1(SoftDemo* pdemo)
{
	const btScalar	s=6;
	const btScalar	h=2;
	const int		r=16;	
	const btVector3	p[]={	btVector3(+s,h,-s),
							btVector3(-s,h,-s),
							btVector3(+s,h,+s),
							btVector3(-s,h,+s)};
	btSoftBody*	psb=btSoftBodyHelpers::CreatePatch(pdemo->m_softBodyWorldInfo,p[0],p[1],p[2],p[3],r,r,1+2+4+8,true);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
	psb->m_cfg.piterations=1;
	pdemo->m_cutting=true;	
}

#ifdef BT_SOFTBODY_USE_STL
//
// TetraBunny
//
static void	Init_TetraBunny(SoftDemo* pdemo)
{
	btSoftBody* psb=btSoftBodyHelpers::CreateFromTetGenData(pdemo->m_softBodyWorldInfo,
															TetraBunny::getElements(),
															0,
															TetraBunny::getNodes(),
															false,true,true);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
	psb->rotate(btQuaternion(SIMD_PI/2,0,0));
	psb->setVolumeMass(150);
	psb->m_cfg.piterations=2;
	pdemo->m_cutting=true;	
}

//
// TetraCube
//
static void	Init_TetraCube(SoftDemo* pdemo)
{
	btSoftBody* psb=btSoftBodyHelpers::CreateFromTetGenData(pdemo->m_softBodyWorldInfo,
															TetraCube::getElements(),
															0,
															TetraCube::getNodes(),
															false,true,true);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
	psb->scale(btVector3(4,4,4));
	psb->translate(btVector3(0,5,0));
	psb->setVolumeMass(300);
	psb->setMass(0,0);
	/*psb->setMass(10,0);
	psb->setMass(20,0);*/
	psb->m_cfg.piterations=1;
	//psb->m_materials[0]->m_kLST=0.05;
	pdemo->m_cutting=true;	
}
#endif //BT_SOFTBODY_USE_STL

//
// Tetra
//
static void	Init_Tetra(SoftDemo* pdemo)
{
	//TRACEDEMO
	#if 0
		{
		btVector3	pts[]={	btVector3(-1,-1,-1),
							btVector3(+1,-1,-1),
							btVector3(+1,+1,-1),
							btVector3(-1,+1,-1),
							btVector3(-1,-1,+1),
							btVector3(+1,-1,+1),
							btVector3(+1,+1,+1),
							btVector3(-1,+1,+1)};
		btSoftBody*	psb=btSoftBodyHelpers::CreateFromConvexHull(pdemo->m_softBodyWorldInfo,pts,8);
		btSoftBodyHelpers::ExportAsSMeshFile(psb,"C:/HomeH/Oss/tetgen/Release/cube.smesh");
		delete psb;
		/*btSoftBody*	psb=btSoftBodyHelpers::CreateFromTriMesh(	pdemo->m_softBodyWorldInfo,gVerticesBunny,
																&gIndicesBunny[0][0],
																BUNNY_NUM_TRIANGLES);
		psb->scale(btVector3(6,6,6));
		btSoftBodyHelpers::ExportAsSMeshFile(psb,"C:/HomeH/Oss/tetgen/Release/bunny.smesh");
		delete psb;*/
		}
	btSoftBody* psb=btSoftBodyHelpers::CreateFromTetGenFile(pdemo->m_softBodyWorldInfo,
												/*"C:/HomeH/Oss/tetgen/Release/bunny.1.ele",
												"C:/HomeH/Oss/tetgen/Release/bunny.1.face",
												"C:/HomeH/Oss/tetgen/Release/bunny.1.node",*/
												"C:/HomeH/Oss/tetgen/Release/cube.1.ele",
												0/*"C:/HomeH/Oss/tetgen/Release/cube.1.face"*/,
												"C:/HomeH/Oss/tetgen/Release/cube.1.node",
												true,true);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
	psb->scale(btVector3(4,4,4));
	/*psb->rotate(btQuaternion(SIMD_PI/4,SIMD_PI/4,0));
	psb->translate(btVector3(0,10,0));*/
	psb->setVolumeMass(30);
	psb->m_cfg.piterations=1;
	psb->m_cfg.kKHR=1;
	//psb->addVelocity(btVector3(0,50,0));
	//psb->m_tetras.clear();
	ImplicitSphere	fnc;
	fnc.center	=	btVector3(4,4,4);
	fnc.radius	=	4;
	psb->refine(&fnc,0.001,true);
	//psb->m_tetras.clear();
	printf("Nodes:  %u\r\n",psb->m_nodes.size());
	printf("Links:  %u\r\n",psb->m_links.size());
	printf("Faces:  %u\r\n",psb->m_faces.size());
	printf("Tetras: %u\r\n",psb->m_tetras.size());
	#else
	
	#if 1
	const btScalar	s=4;
	const int		r=32;
	const btVector3	p[]={	btVector3(+s,0,-s),
							btVector3(-s,0,-s),
							btVector3(+s,0,+s),
							btVector3(-s,0,+s)};
	btSoftBody*	psb=btSoftBodyHelpers::CreatePatch(pdemo->m_softBodyWorldInfo,p[0],p[1],p[2],p[3],r,r,1+2+4+8,true);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
	psb->m_cfg.piterations=2;
	/*ImplicitSphere	fnc;
	fnc.center	=	btVector3(0,0,0);
	fnc.radius	=	1.5;
	psb->refine(&fnc,0.001,true);*/
	//psb->m_faces.clear();
	/*fnc.center	=	btVector3(4,0,4);
	fnc.radius	=	2;
	psb->refine(&fnc,0.001,true);*/
	#else
	const btScalar	s=4;
	const btVector3	p[]={	btVector3(+s,-s,0),
							btVector3(-s,0,0),
							btVector3(0,0,+s),
							btVector3(0,+s,0)};
	btSoftBody*	psb=new btSoftBody(&pdemo->m_softBodyWorldInfo,4,p,0);
	psb->appendTetra(0,1,2,3);
	psb->appendLink(0,1,1,btSoftBody::eLType::Structural);
	psb->appendLink(1,2,1,btSoftBody::eLType::Structural);
	psb->appendLink(2,0,1,btSoftBody::eLType::Structural);
	psb->appendLink(0,3,1,btSoftBody::eLType::Structural);
	psb->appendLink(1,3,1,btSoftBody::eLType::Structural);
	psb->appendLink(2,3,1,btSoftBody::eLType::Structural);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);	
	psb->setSolver(btSoftBody::eSolverPresets::Velocities);
	psb->m_cfg.viterations=1;
	psb->m_cfg.diterations=1;
	psb->m_cfg.kDF=0;
	psb->m_cfg.kLST=0.000001;
	//psb1->m_cfg.diterations=1;
	/*btSoftBody*	psb0=btSoftBodyHelpers::CreateRope(	pdemo->m_softBodyWorldInfo,
													btVector3(0,0,0),btVector3(5,0,0),16,1);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb0);
	
	btSoftBody*	psb1=btSoftBodyHelpers::CreateRope(	pdemo->m_softBodyWorldInfo,
													btVector3(0,0,2),btVector3(5,0,2),16,1);
	psb1->m_cfg.viterations=1;
	psb1->m_cfg.diterations=1;
	psb1->setSolver(btSoftBody::eSolverPresets::Velocities);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb1);*/
	#endif
	#endif
	pdemo->toggleIdle();
}

unsigned	current_demo=0;

void	SoftDemo::clientResetScene()
{
	DemoApplication::clientResetScene();
	/* Clean up	*/ 
	for(int i=m_dynamicsWorld->getNumCollisionObjects()-1;i>0;i--)
	{
		btCollisionObject*	obj=m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody*		body=btRigidBody::upcast(obj);
		if(body&&body->getMotionState())
		{
			delete body->getMotionState();
		}
		btSoftBody* softBody = btSoftBody::upcast(obj);
		if (softBody)
		{
			getSoftDynamicsWorld()->removeSoftBody(softBody);
		} else
		{
			m_dynamicsWorld->removeCollisionObject(obj);
		}
		delete obj;
	}
	
	m_softBodyWorldInfo.m_sparsesdf.Reset();
	/* Init		*/ 
	void (*demofncs[])(SoftDemo*)=
	{
		Init_Cloth,
		Init_Cutting1,
#ifdef BT_SOFTBODY_USE_STL
		Init_TetraBunny,
		Init_TetraCube,
#endif //BT_SOFTBODY_USE_STL
		Init_Pressure,
		Init_Volume,
		Init_Ropes,
		Init_RopeAttach,
		Init_ClothAttach,
		Init_Sticks,	
		Init_Collide,
		Init_Collide2,
		Init_Collide3,
		Init_Impact,
		Init_Aero,
		Init_Friction,			
		Init_Torus,
		Init_TorusMatch,
		Init_Bunny,
		Init_BunnyMatch,
	};
	current_demo=current_demo%(sizeof(demofncs)/sizeof(demofncs[0]));


	m_softBodyWorldInfo.air_density		=	(btScalar)1.2;
	m_softBodyWorldInfo.water_density	=	0;
	m_softBodyWorldInfo.water_offset		=	0;
	m_softBodyWorldInfo.water_normal		=	btVector3(0,0,0);
	m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);


	m_autocam						=	false;
	m_raycast						=	false;
	m_cutting						=	false;
	demofncs[current_demo](this);
}

void	SoftDemo::renderme()
{
	btIDebugDraw*	idraw=m_dynamicsWorld->getDebugDrawer();
	
	m_dynamicsWorld->debugDrawWorld();
	
	/* Bodies		*/ 
	btVector3	ps(0,0,0);
	int			nps=0;
	
	btSoftBodyArray&	sbs=getSoftDynamicsWorld()->getSoftBodyArray();
	for(int ib=0;ib<sbs.size();++ib)
	{
		btSoftBody*	psb=sbs[ib];
		nps+=psb->m_nodes.size();
		for(int i=0;i<psb->m_nodes.size();++i)
		{
			ps+=psb->m_nodes[i].m_x;
		}		
	}
	ps/=nps;
	if(m_autocam)
		m_cameraTargetPosition+=(ps-m_cameraTargetPosition)*0.01;
	else
		m_cameraTargetPosition=btVector3(0,0,0);
	/* Anm			*/ 
	if(!isIdle())
		m_animtime=m_clock.getTimeMilliseconds()/1000.f;
	/* Ray cast		*/ 
	if(m_raycast)
		{		
		/* Prepare rays	*/ 
		const int		res=64;
		const btScalar	fres=res-1;
		const btScalar	size=8;
		const btScalar	dist=10;
		btTransform		trs;
		trs.setOrigin(ps);
		const btScalar	angle=m_animtime*0.2;
		trs.setRotation(btQuaternion(angle,SIMD_PI/4,0));
		const btVector3	dir=trs.getBasis()*btVector3(0,-1,0);
		trs.setOrigin(ps-dir*dist);
		btAlignedObjectArray<btVector3>	origins;
		btAlignedObjectArray<btScalar>	times;
		origins.resize(res*res);
		times.resize(res*res,SIMD_INFINITY);
		for(int y=0;y<res;++y)
			{
			for(int x=0;x<res;++x)
				{
				const int	idx=y*res+x;
				origins[idx]=trs*btVector3(-size+size*2*x/fres,dist,-size+size*2*y/fres);
				}
			}
		/* Cast rays	*/ 		
			{
			m_clock.reset();
			const btVector3*		org=&origins[0];
			btScalar*				mint=&times[0];
			btSoftBody**			psbs=&sbs[0];
			btSoftBody::sRayCast	results;
			for(int i=0,ni=origins.size(),nb=sbs.size();i<ni;++i)
				{
				for(int ib=0;ib<nb;++ib)
					{
					if(psbs[ib]->rayCast(*org,dir,results,*mint))
						{
						*mint=results.time;
						}
					}
				++org;++mint;
				}
			long	ms=btMax<long>(m_clock.getTimeMilliseconds(),1);
			long	rayperseconds=(1000*(origins.size()*sbs.size()))/ms;
			printf("%d ms (%d rays/s)\r\n",ms,rayperseconds);
			}
		/* Draw rays	*/ 
		const btVector3	c[]={	origins[0],
								origins[res-1],
								origins[res*(res-1)],
								origins[res*(res-1)+res-1]};
		idraw->drawLine(c[0],c[1],btVector3(0,0,0));
		idraw->drawLine(c[1],c[3],btVector3(0,0,0));
		idraw->drawLine(c[3],c[2],btVector3(0,0,0));
		idraw->drawLine(c[2],c[0],btVector3(0,0,0));
		for(int i=0,ni=origins.size();i<ni;++i)
			{
			const btScalar		tim=times[i];
			const btVector3&	org=origins[i];
			if(tim<SIMD_INFINITY)
				{
				idraw->drawLine(org,org+dir*tim,btVector3(1,0,0));
				}
				else
				{
				idraw->drawLine(org,org-dir*0.1,btVector3(0,0,0));
				}
			}
		#undef RES
		}
	/* Water level	*/ 
	static const btVector3	axis[]={btVector3(1,0,0),
		btVector3(0,1,0),
		btVector3(0,0,1)};
	if(m_softBodyWorldInfo.water_density>0)
	{
		const btVector3	c=	btVector3((btScalar)0.25,(btScalar)0.25,1);
		const btScalar	a=	(btScalar)0.5;
		const btVector3	n=	m_softBodyWorldInfo.water_normal;
		const btVector3	o=	-n*m_softBodyWorldInfo.water_offset;
		const btVector3	x=	cross(n,axis[n.minAxis()]).normalized();
		const btVector3	y=	cross(x,n).normalized();
		const btScalar	s=	25;
		idraw->drawTriangle(o-x*s-y*s,o+x*s-y*s,o+x*s+y*s,c,a);
		idraw->drawTriangle(o-x*s-y*s,o+x*s+y*s,o-x*s+y*s,c,a);
	}
	DemoApplication::renderme();
	
}

void	SoftDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch(key)
	{
	case	']':	++current_demo;clientResetScene();break;
	case	'[':	--current_demo;clientResetScene();break;
	case	',':	m_raycast=!m_raycast;break;
	case	';':	m_autocam=!m_autocam;break;
	case	'`':
		{
		btSoftBodyArray&	sbs=getSoftDynamicsWorld()->getSoftBodyArray();
		for(int ib=0;ib<sbs.size();++ib)
			{
			btSoftBody*	psb=sbs[ib];
			psb->staticSolve(128);
			}
		}
	break;
	default:		DemoApplication::keyboardCallback(key,x,y);
	}
}

//
void	SoftDemo::mouseFunc(int button, int state, int x, int y)
{
if(m_cutting&&(state==0)&&(button==0))
	{
	const btVector3			rayFrom=m_cameraPosition;
	const btVector3			rayTo=getRayTo(x,y);
	const btVector3			rayDir=(rayTo-rayFrom).normalized();
	btSoftBodyArray&		sbs=getSoftDynamicsWorld()->getSoftBodyArray();
	btSoftBody::sRayCast	results;
	results.time=SIMD_INFINITY;
	for(int ib=0;ib<sbs.size();++ib)
		{
		btSoftBody*				psb=sbs[ib];
		btSoftBody::sRayCast	res;
		if(psb->rayCast(rayFrom,rayDir,res,results.time))
			{
			results=res;
			}
		}
	if(results.time<SIMD_INFINITY)
		{
		#if 0
		const btVector3			x=rayFrom+rayDir*results.time;
		const btSoftBody::Face& f=results.body->m_faces[results.index];
		btScalar				bestarea=SIMD_INFINITY;
		const btSoftBody::Node*	n[2]={0,0};
		for(int i=2,j=0;j<3;i=j++)
			{
			const btScalar a2=cross(f.m_n[i]->m_x-x,f.m_n[j]->m_x-x).length2();
			if(a2<bestarea)
				{
				bestarea=a2;
				n[0]=f.m_n[i];
				n[1]=f.m_n[j];
				}
			}
		results.body->cutLink(n[0],n[1],0.5);
		#endif
		ImplicitSphere	isphere(rayFrom+rayDir*results.time,1);
		printf("Mass before: %f\r\n",results.body->getTotalMass());
		results.body->refine(&isphere,0.0001,true);
		printf("Mass after: %f\r\n",results.body->getTotalMass());
		return;
		}
	}
DemoApplication::mouseFunc(button,state,x,y);
}


void	SoftDemo::initPhysics()
{

	
	m_collisionShapes.push_back(new btBoxShape (btVector3(200,CUBE_HALF_EXTENTS,200)));

	m_collisionShapes.push_back(new btCylinderShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)));


	m_dispatcher=0;

	///register some softbody collision algorithms on top of the default btDefaultCollisionConfiguration
	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();


	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
	m_softBodyWorldInfo.m_dispatcher = m_dispatcher;

	////////////////////////////
	///Register softbody versus softbody collision algorithm


	///Register softbody versus rigidbody collision algorithm


	////////////////////////////

	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);

	m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);

	m_softBodyWorldInfo.m_broadphase = m_broadphase;

	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	m_solver = solver;

	btDiscreteDynamicsWorld* world = new btSoftRigidDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	m_dynamicsWorld = world;


	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));
	m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);





	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0,-12,0));



	btRigidBody* body = localCreateRigidBody(0.f,tr,m_collisionShapes[0]);


	//	clientResetScene();

	m_softBodyWorldInfo.m_sparsesdf.Initialize();
	clientResetScene();
}






void	SoftDemo::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		m_collisionShapes[j] = 0;
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;



	delete m_collisionConfiguration;


}





