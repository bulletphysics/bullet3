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
#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"
#include "BMF_Api.h"
#include "../GimpactTestDemo/BunnyMesh.h"
#include "../GimpactTestDemo/TorusMesh.h"
#include <stdio.h> //printf debugging
#include "../../Extras/ConvexHull/btConvexHull.h"


static float	gCollisionMargin = 0.05f/*0.05f*/;
#include "SoftDemo.h"
#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"

extern float eye[3];
extern int glutScreenWidth;
extern int glutScreenHeight;

const int maxProxies = 32766;
const int maxOverlap = 65535;

//
btSoftBody*		CreateFromConvexHull(	btSoftBody::ISoftBody*	isoftbody,
										const btVector3* vertices,
										int nvertices)
{
HullDesc		hdsc(QF_TRIANGLES,nvertices,vertices);
HullResult		hres;
HullLibrary		hlib;/*??*/ 
hdsc.mMaxVertices=nvertices;
hlib.CreateConvexHull(hdsc,hres);
btSoftBody*		psb=btSoftBody::Create(	isoftbody,
										(int)hres.mNumOutputVertices,
										hres.mOutputVertices,0);
for(int i=0;i<(int)hres.mNumFaces;++i)
	{
	const int idx[]={	hres.mIndices[i*3+0],
						hres.mIndices[i*3+1],
						hres.mIndices[i*3+2]};
	if(idx[0]<idx[1]) psb->AppendLink(	idx[0],idx[1],
										1,btSoftBody::eLType::Structural);
	if(idx[1]<idx[2]) psb->AppendLink(	idx[1],idx[2],
										1,btSoftBody::eLType::Structural);
	if(idx[2]<idx[0]) psb->AppendLink(	idx[2],idx[0],
										1,btSoftBody::eLType::Structural);
	psb->AppendFace(idx[0],idx[1],idx[2]);
	}
hlib.ReleaseResult(hres);
psb->RandomizeConstraints();
return(psb);
}


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




//
// ISoftBody implementation
//

//
void		SoftDemo::SoftBodyImpl::Attach(btSoftBody*)
{}

//
void		SoftDemo::SoftBodyImpl::Detach(btSoftBody*)
{}

//
void		SoftDemo::SoftBodyImpl::StartCollide(const btVector3&,const btVector3&)
{}

//
bool		SoftDemo::SoftBodyImpl::CheckContactPrecise(const btVector3& x,
														btSoftBody::ISoftBody::sCti& cti)
{
btScalar					maxdepth=0;
btGjkEpaSolver2::sResults	res;
btDynamicsWorld*			pdw=pdemo->m_dynamicsWorld;
btCollisionObjectArray&		coa=pdw->getCollisionObjectArray();
for(int i=0,ni=coa.size();i<ni;++i)
	{
	btRigidBody*			prb=(btRigidBody*)(coa[i]);
	btCollisionShape*		shp=prb->getCollisionShape();
	if(shp->isConvex())
		{
		btConvexShape*		csh=static_cast<btConvexShape*>(shp);
		const btTransform&	wtr=prb->getWorldTransform();
		const btScalar		dst=btGjkEpaSolver2::SignedDistance(x,0.1,csh,wtr,res);
		if(dst<maxdepth)
			{
			maxdepth		=	dst;
			cti.m_body		=	prb;
			cti.m_normal	=	res.normal;
			cti.m_offset	=	-dot(cti.m_normal,res.witnesses[0]);
			}
		}
	}
return(maxdepth<0);
}

//
bool		SoftDemo::SoftBodyImpl::CheckContact(	const btVector3& x,
													btSoftBody::ISoftBody::sCti& cti)
{
btScalar					maxdepth=0;
btGjkEpaSolver2::sResults	res;
btDynamicsWorld*			pdw=pdemo->m_dynamicsWorld;
btCollisionObjectArray&		coa=pdw->getCollisionObjectArray();
for(int i=0,ni=coa.size();i<ni;++i)
	{
	btVector3			nrm;
	btRigidBody*		prb=(btRigidBody*)(coa[i]);
	btCollisionShape*	shp=prb->getCollisionShape();
	btConvexShape*		csh=static_cast<btConvexShape*>(shp);
	const btTransform&	wtr=prb->getWorldTransform();
	btScalar			dst=pdemo->m_sparsesdf.Evaluate(wtr.invXform(x),csh,nrm);
	nrm=wtr.getBasis()*nrm;
	btVector3			wit=x-nrm*dst;
	if(dst<maxdepth)
		{
		maxdepth		=	dst;
		cti.m_body		=	prb;
		cti.m_normal	=	nrm;
		cti.m_offset	=	-dot(cti.m_normal,wit);
		}
	}
return(maxdepth<0);
}

//
void		SoftDemo::SoftBodyImpl::EndCollide()
{}

//
void		SoftDemo::SoftBodyImpl::EvaluateMedium(	const btVector3& x,
													btSoftBody::ISoftBody::sMedium& medium)
{
medium.m_velocity	=	btVector3(0,0,0);
medium.m_pressure	=	0;
medium.m_density	=	air_density;
if(water_density>0)
	{
	const btScalar	depth=-(dot(x,water_normal)+water_offset);
	if(depth>0)
		{
		medium.m_density	=	water_density;
		medium.m_pressure	=	depth			*
								water_density	*
								pdemo->m_dynamicsWorld->getGravity().length();
		}
	}
}



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
#define FIXED_STEP 0
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
		
		/* soft bodies	*/ 
		for(int ib=0;ib<m_softbodies.size();++ib)
			{
			btSoftBody*	psb=m_softbodies[ib];
			psb->AddVelocity(m_dynamicsWorld->getGravity()*dt);
			psb->Step(dt);
			}
		m_sparsesdf.GarbageCollect();
		
		//optional but useful: debug drawing
		
		m_dynamicsWorld->debugDrawWorld();		
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
btSoftBody*		psb=CreateFromConvexHull(&pdemo->m_softbodyimpl,c,8);
psb->GenerateBendingConstraints(2,1);
pdemo->m_softbodies.push_back(psb);
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
btSoftBody*		psb=CreateFromConvexHull(&pdemo->m_softbodyimpl,&pts[0],pts.size());
psb->GenerateBendingConstraints(2,1);
pdemo->m_softbodies.push_back(psb);
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
	btSoftBody*	psb=CreateRope(	&pdemo->m_softbodyimpl,
								btVector3(-10,0,i*0.25),
								btVector3(10,0,i*0.25),
								16,
								1+2);
	psb->m_cfg.iterations	=	4;
	psb->m_cfg.kLST			=	0.1+(i/(btScalar)(n-1))*0.9;
	psb->SetTotalMass(20);
	pdemo->m_softbodies.push_back(psb);
	}
}

//
// Rope attach
//
static void	Init_RopeAttach(SoftDemo* pdemo)
{
//TRACEDEMO
struct	Functors
	{
	static btSoftBody* CtorRope(SoftDemo* pdemo,const btVector3& p)
		{
		btSoftBody*	psb=CreateRope(&pdemo->m_softbodyimpl,
							p,p+btVector3(10,0,0),8,1);
		psb->m_cfg.kDF			=	0;
		psb->SetTotalMass(50);
		pdemo->m_softbodies.push_back(psb);
		return(psb);
		}
	};
btTransform startTransform;
startTransform.setIdentity();
startTransform.setOrigin(btVector3(12,8,0));
btRigidBody*		body=pdemo->localCreateRigidBody(50,startTransform,new btBoxShape(btVector3(2,6,2)));
btSoftBody*	psb0=Functors::CtorRope(pdemo,btVector3(0,8,-1));
btSoftBody*	psb1=Functors::CtorRope(pdemo,btVector3(0,8,+1));
psb0->AppendAnchor(psb0->m_nodes.size()-1,body);
psb1->AppendAnchor(psb1->m_nodes.size()-1,body);
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
btSoftBody*		psb=CreatePatch(&pdemo->m_softbodyimpl,
								btVector3(-s,h,-s),
								btVector3(+s,h,-s),
								btVector3(-s,h,+s),
								btVector3(+s,h,+s),r,r,4+8,true);
pdemo->m_softbodies.push_back(psb);
btTransform startTransform;
startTransform.setIdentity();
startTransform.setOrigin(btVector3(0,h,-(s+3.5)));
btRigidBody*		body=pdemo->localCreateRigidBody(20,startTransform,new btBoxShape(btVector3(s,1,3)));
psb->AppendAnchor(0,body);
psb->AppendAnchor(r-1,body);
}

//
// Impact
//
static void	Init_Impact(SoftDemo* pdemo)
{
//TRACEDEMO
btSoftBody*	psb=CreateRope(	&pdemo->m_softbodyimpl,
							btVector3(0,0,0),
							btVector3(0,-1,0),
							0,
							1);
pdemo->m_softbodies.push_back(psb);
btTransform startTransform;
startTransform.setIdentity();
startTransform.setOrigin(btVector3(0,20,0));
btRigidBody*		body=pdemo->localCreateRigidBody(10,startTransform,new btBoxShape(btVector3(2,2,2)));
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
	btSoftBody*		psb=CreatePatch(&pdemo->m_softbodyimpl,
									btVector3(-s,h,-s),
									btVector3(+s,h,-s),
									btVector3(-s,h,+s),
									btVector3(+s,h,+s),
									segments,segments,
									0,true);
	psb->GenerateBendingConstraints(2,1);
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
	psb->Transform(trs);
	psb->SetTotalMass(0.1);
	psb->AddForce(btVector3(0,2,0),0);
	pdemo->m_softbodies.push_back(psb);
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
	psb->AddVelocity(btVector3(0,0,-10));
	}
}

//
// Pressure
//
static void	Init_Pressure(SoftDemo* pdemo)
{
//TRACEDEMO
btSoftBody*	psb=CreateEllipsoid(&pdemo->m_softbodyimpl,
								btVector3(35,25,0),
								btVector3(1,1,1)*3,
								512);
psb->m_cfg.kLST		=	0.1;
psb->m_cfg.kDF		=	1;
psb->m_cfg.kPR		=	2500;
psb->SetTotalMass(30,true);
pdemo->m_softbodies.push_back(psb);
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
btSoftBody*	psb=CreateEllipsoid(&pdemo->m_softbodyimpl,
								btVector3(35,25,0),
								btVector3(1,1,1)*3,
								512);
psb->m_cfg.kLST		=	0.45;
psb->m_cfg.kVC		=	20;
psb->SetTotalMass(50,true);
psb->SetPose(true,false);
pdemo->m_softbodies.push_back(psb);
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
		btSoftBody*		psb=CreateRope(	&pdemo->m_softbodyimpl,
										org,
										org+btVector3(hg*0.001,hg,0),
										sg,
										1);
		psb->m_cfg.iterations	=	1;
		psb->m_cfg.kDP		=	0.005;
		psb->m_cfg.kCHR		=	0.1;
		for(int i=0;i<3;++i)
			{
			psb->GenerateBendingConstraints(2+i,1);
			}
		psb->SetMass(1,0);
		psb->SetTotalMass(0.01);
		pdemo->m_softbodies.push_back(psb);
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
btSoftBody*		psb=CreatePatch(&pdemo->m_softbodyimpl,
								btVector3(-s,0,-s),
								btVector3(+s,0,-s),
								btVector3(-s,0,+s),
								btVector3(+s,0,+s),
								31,31,
								1+2+4+8,true);
psb->GenerateBendingConstraints(2,1);
psb->m_cfg.kLST			=	0.4;
psb->SetTotalMass(150);
pdemo->m_softbodies.push_back(psb);
Ctor_RbUpStack(pdemo,10);
}

//
// 100kg Stanford's bunny
//
static void	Init_Bunny(SoftDemo* pdemo)
{
//TRACEDEMO
btSoftBody*	psb=CreateFromTriMesh(	&pdemo->m_softbodyimpl,
									gVerticesBunny,
									&gIndicesBunny[0][0],
									BUNNY_NUM_TRIANGLES);
psb->GenerateBendingConstraints(2,0.5);
psb->m_cfg.iterations	=	2;
psb->m_cfg.kDF			=	0.5;
psb->RandomizeConstraints();
psb->Scale(btVector3(6,6,6));
psb->SetTotalMass(100,true);
pdemo->m_softbodies.push_back(psb);	
}

//
// 100kg Stanford's bunny with pose matching
//
static void	Init_BunnyMatch(SoftDemo* pdemo)
{
//TRACEDEMO
btSoftBody*	psb=CreateFromTriMesh(	&pdemo->m_softbodyimpl,
									gVerticesBunny,
									&gIndicesBunny[0][0],
									BUNNY_NUM_TRIANGLES);
psb->GenerateBendingConstraints(2,0.5);
psb->m_cfg.kDF		=	0.5;
psb->m_cfg.kLST		=	0.1;
psb->m_cfg.kMT		=	0.01;
psb->RandomizeConstraints();
psb->Scale(btVector3(6,6,6));
psb->SetTotalMass(100,true);
psb->SetPose(true,true);
pdemo->m_softbodies.push_back(psb);	
}

//
// 50Kg Torus
//
static void	Init_Torus(SoftDemo* pdemo)
{
//TRACEDEMO
btSoftBody*	psb=CreateFromTriMesh(	&pdemo->m_softbodyimpl,
									gVertices,
									&gIndices[0][0],
									NUM_TRIANGLES);
psb->GenerateBendingConstraints(2,1);
psb->m_cfg.iterations=2;
psb->RandomizeConstraints();
btMatrix3x3	m;
m.setEulerZYX(SIMD_PI/2,0,0);
psb->Transform(btTransform(m,btVector3(0,4,0)));
psb->Scale(btVector3(2,2,2));
psb->SetTotalMass(50,true);
pdemo->m_softbodies.push_back(psb);
}

//
// 50Kg Torus with pose matching
//
static void	Init_TorusMatch(SoftDemo* pdemo)
{
//TRACEDEMO
btSoftBody*	psb=CreateFromTriMesh(	&pdemo->m_softbodyimpl,
									gVertices,
									&gIndices[0][0],
									NUM_TRIANGLES);
psb->GenerateBendingConstraints(2,1);
psb->m_cfg.kLST=0.1;
psb->m_cfg.kMT=0.05;
psb->RandomizeConstraints();
btMatrix3x3	m;
m.setEulerZYX(SIMD_PI/2,0,0);
psb->Transform(btTransform(m,btVector3(0,4,0)));
psb->Scale(btVector3(2,2,2));
psb->SetTotalMass(50,true);
psb->SetPose(true,true);
pdemo->m_softbodies.push_back(psb);
}

static unsigned	current_demo=0;

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
	m_dynamicsWorld->removeCollisionObject(obj);
	delete obj;
	}
for(int i=0;i<m_softbodies.size();++i)
	{
	delete m_softbodies[i];
	}
m_softbodies.clear();
m_sparsesdf.Reset();
/* Init		*/ 
void (*demofncs[])(SoftDemo*)=
	{
	Init_Cloth,
	Init_Pressure,
	Init_Volume,
	Init_Ropes,
	Init_RopeAttach,
	Init_ClothAttach,
	Init_Sticks,	
	Init_Impact,
	Init_Aero,
	Init_Friction,			
	Init_Torus,
	Init_TorusMatch,
	Init_Bunny,
	Init_BunnyMatch,
	};
current_demo=current_demo%(sizeof(demofncs)/sizeof(demofncs[0]));
m_softbodyimpl.air_density		=	(btScalar)1.2;
m_softbodyimpl.water_density	=	0;
m_softbodyimpl.water_offset		=	0;
m_softbodyimpl.water_normal		=	btVector3(0,0,0);
m_autocam						=	false;
demofncs[current_demo](this);
}

void	SoftDemo::renderme()
{
DemoApplication::renderme();
btIDebugDraw*	idraw=m_dynamicsWorld->getDebugDrawer();
/* Bodies		*/ 
btVector3	ps(0,0,0);
int			nps=0;
for(int ib=0;ib<m_softbodies.size();++ib)
	{
	btSoftBody*	psb=m_softbodies[ib];
	nps+=psb->m_nodes.size();
	for(int i=0;i<psb->m_nodes.size();++i)
		{
		ps+=psb->m_nodes[i].m_x;
		}
	DrawFrame(psb,idraw);
	Draw(psb,idraw,fDrawFlags::Std);
	}
ps/=nps;
if(m_autocam)
	m_cameraTargetPosition+=(ps-m_cameraTargetPosition)*0.01;
	else
	m_cameraTargetPosition=btVector3(0,0,0);
/* Water level	*/ 
static const btVector3	axis[]={btVector3(1,0,0),
								btVector3(0,1,0),
								btVector3(0,0,1)};
if(m_softbodyimpl.water_density>0)
	{
	const btVector3	c=	btVector3((btScalar)0.25,(btScalar)0.25,1);
	const btScalar	a=	(btScalar)0.5;
	const btVector3	n=	m_softbodyimpl.water_normal;
	const btVector3	o=	-n*m_softbodyimpl.water_offset;
	const btVector3	x=	cross(n,axis[n.minAxis()]).normalized();
	const btVector3	y=	cross(x,n).normalized();
	const btScalar	s=	25;
	idraw->drawTriangle(o-x*s-y*s,o+x*s-y*s,o+x*s+y*s,c,a);
	idraw->drawTriangle(o-x*s-y*s,o+x*s+y*s,o-x*s+y*s,c,a);
	}
}

void	SoftDemo::keyboardCallback(unsigned char key, int x, int y)
{
switch(key)
	{
	case	']':	++current_demo;clientResetScene();break;
	case	'[':	--current_demo;clientResetScene();break;
	case	';':	m_autocam=!m_autocam;break;
	default:		DemoApplication::keyboardCallback(key,x,y);
	}
}


void	SoftDemo::initPhysics()
{

//#define USE_GROUND_PLANE 1
#ifdef USE_GROUND_PLANE
	m_collisionShapes.push_back(new btStaticPlaneShape(btVector3(0,1,0),0.5));
#else

	///Please don't make the box sizes larger then 1000: the collision detection will be inaccurate.
	///See http://www.continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=346
	m_collisionShapes.push_back(new btBoxShape (btVector3(200,CUBE_HALF_EXTENTS,200)));
	//m_collisionShapes.push_back(new btCylinderShapeZ (btVector3(5,5,5)));
	//m_collisionShapes.push_back(new btSphereShape(5));
#endif

	m_collisionShapes.push_back(new btCylinderShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)));
	

	m_dispatcher=0;
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	
	
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);

	m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);
/// For large worlds or over 16384 objects, use the bt32BitAxisSweep3 broadphase
//	m_broadphase = new bt32BitAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);
/// When trying to debug broadphase issues, try to use the btSimpleBroadphase
//	m_broadphase = new btSimpleBroadphase;
	
	//box-box is in Extras/AlternativeCollisionAlgorithms:it requires inclusion of those files


	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	m_solver = solver;
	//default solverMode is SOLVER_RANDMIZE_ORDER. Warmstarting seems not to improve convergence, see 
	//solver->setSolverMode(0);//btSequentialImpulseConstraintSolver::SOLVER_USE_WARMSTARTING | btSequentialImpulseConstraintSolver::SOLVER_RANDMIZE_ORDER);

	btDiscreteDynamicsWorld* world = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	m_dynamicsWorld = world;


	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	


	int i;

	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0,-20,0));

	

	btRigidBody* body = localCreateRigidBody(0.f,tr,m_collisionShapes[0]);

	
//	clientResetScene();

m_softbodyimpl.pdemo=this;
m_sparsesdf.Initialize();
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


