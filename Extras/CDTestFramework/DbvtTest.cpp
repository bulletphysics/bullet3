/*
Bounding Volume Hierarchy Test
Copyright (c) 2008 Nathanael Presson, as part of Bullet Physics Library

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "stdafx.h"
#include "DbvtTest.h"
#include "RenderingHelpers.h"
#include "GLFontRenderer.h"
#include "btBulletCollisionCommon.h"
#include "btDbvt.h"

namespace	_0E5F0A41_1DC2_4e14_9ABC_1CBE13DF25C8_
{

//
class	btDbvtProxy : public btBroadphaseProxy
{
public:
AABB			aabb;
btDbvt::Node*	leaf;
bool			contact;
};

//
struct	btDbvtPair
{
btDbvtProxy*	proxies[2];
};

//
class	btDbvtBroadphase : public btBroadphaseInterface
{
public:
										btDbvtBroadphase();
virtual									~btDbvtBroadphase()	{}
virtual btBroadphaseProxy*				createProxy(const btVector3& aabbMin,const btVector3& aabbMax,int shapeType,void* userPtr,short int collisionFilterGroup,short int collisionFilterMask, btDispatcher* dispatcher,void* multiSapProxy);
virtual void							destroyProxy(btBroadphaseProxy* proxy,btDispatcher* dispatcher);
virtual void							setAabb(btBroadphaseProxy* proxy,const btVector3& aabbMin,const btVector3& aabbMax, btDispatcher* dispatcher);
virtual void							calculateOverlappingPairs(btDispatcher* dispatcher) {}
virtual	btOverlappingPairCache*			getOverlappingPairCache() { return(0); }
virtual	const btOverlappingPairCache*	getOverlappingPairCache() const { return(0); }
virtual void							getBroadphaseAabb(btVector3& aabbMin,btVector3& aabbMax) const {}
virtual void							printStats() {}
btAlignedObjectArray<btDbvtProxy*>		m_proxies;
btAlignedObjectArray<btDbvtPair>		m_pairs;
btDbvt*									m_dbvt;
};

//
										btDbvtBroadphase::btDbvtBroadphase()
{
while(m_proxies.size()) destroyProxy(m_proxies[0],0);
}
//
btBroadphaseProxy*						btDbvtBroadphase::createProxy(const btVector3& aabbMin,const btVector3& aabbMax,int shapeType,void* userPtr,short int collisionFilterGroup,short int collisionFilterMask, btDispatcher* dispatcher,void* multiSapProxy)
{
btDbvtProxy*	pp=new btDbvtProxy();
pp->aabb.SetMinMax(	Point(aabbMin.x(),aabbMin.y(),aabbMin.z()),
					Point(aabbMax.x(),aabbMax.y(),aabbMax.z()));
pp->leaf=m_dbvt->Insert((aabbMin+aabbMax)/2,(aabbMax-aabbMin)/2,pp);
m_proxies.push_back(pp);
return(pp);
}

//
void									btDbvtBroadphase::destroyProxy(btBroadphaseProxy* proxy,btDispatcher* dispatcher)
{
btDbvtProxy*	pp=(btDbvtProxy*)proxy;
m_proxies.remove(pp);
m_dbvt->Remove(pp->leaf);
delete proxy;
}

//
void									btDbvtBroadphase::setAabb(btBroadphaseProxy* proxy,const btVector3& aabbMin,const btVector3& aabbMax, btDispatcher* dispatcher)
{
AABB			box;
btDbvtProxy*	pp=(btDbvtProxy*)proxy;
box.SetMinMax(	Point(aabbMin.x(),aabbMin.y(),aabbMin.z()),
				Point(aabbMax.x(),aabbMax.y(),aabbMax.z()));
pp->aabb=box;
}

}

using namespace _0E5F0A41_1DC2_4e14_9ABC_1CBE13DF25C8_;

DbvtTest::DbvtTest(int numBoxes)
{
m_bfirsttime	=	true;
m_nbox			=	numBoxes;
m_broadphase	=	new btDbvtBroadphase();
m_margin		=	0.5;
m_bar			=	0;
m_speed			=	0.005;
m_amp			=	100;
}

DbvtTest::~DbvtTest()
{
}

void DbvtTest::Init()
{
SRand(0);
((btDbvtBroadphase*)m_broadphase)->m_dbvt=btDbvt::Create();
m_times.resize(m_nbox);
for(int i=0;i<m_nbox;i++)
	{
	Point Center, Extents;
	Center.x = (UnitRandomFloat()-0.5f) * 100.0f;
	Center.y = (UnitRandomFloat()-0.5f) * 10.0f;
	Center.z = (UnitRandomFloat()-0.5f) * 100.0f;
	Extents.x = 2.0f + UnitRandomFloat() * 2.0f;
	Extents.y = 2.0f + UnitRandomFloat() * 2.0f;
	Extents.z = 2.0f + UnitRandomFloat() * 2.0f;
	btVector3	aabbMin(Center.x-Extents.x,Center.y-Extents.y,Center.z-Extents.z);
	btVector3	aabbMax(Center.x+Extents.x,Center.y+Extents.y,Center.z+Extents.z);
	m_broadphase->createProxy(aabbMin,aabbMax,0,0,1,1,0,0);
	m_times[i]=2000.0f*UnitRandomFloat();
	}
}

void DbvtTest::Release()
{
}

void DbvtTest::Select()
{
m_bar=TwNewBar("Dbvt");
TwAddVarRW(m_bar,"Margin",TW_TYPE_FLOAT,&m_margin," min=0.0 max=10.0 step=0.5");
TwAddVarRW(m_bar,"Speed", TW_TYPE_FLOAT, &m_speed, " min=0.0 max=0.01 step=0.00001");
TwAddVarRW(m_bar,"Amplitude", TW_TYPE_FLOAT, &m_amp, " min=10.0 max=200.0 step=0.1");
}

void DbvtTest::Deselect()
{
if(m_bar)
	{
	TwDeleteBar(m_bar);
	m_bar=0;
	}
}

static inline bool Intersect(	const btDbvt::Aabb& a,
								const btDbvt::Aabb& b)
{
return(	(a.mi.x()<=b.mx.x())&&
		(a.mx.x()>=b.mi.x())&&
		(a.mi.y()<=b.mx.y())&&
		(a.mx.y()>=b.mi.y())&&
		(a.mi.z()<=b.mx.z())&&
		(a.mx.z()>=b.mi.z()));
}

static void	TreeTreeParse(	const btDbvt::Node* a,
							const btDbvt::Node* b,
							btAlignedObjectArray<btDbvtPair>& pairs)
{
if(a==b)
	{
	if(a->isinternal())
		{
		TreeTreeParse(a->childs[0],a->childs[0],pairs);
		TreeTreeParse(a->childs[1],a->childs[1],pairs);
		TreeTreeParse(a->childs[0],a->childs[1],pairs);
		}
	}
else if(Intersect(a->box,b->box))
	{
	if(a->isinternal())
		{
		if(b->isinternal())
			{
			TreeTreeParse(a->childs[0],b->childs[0],pairs);
			TreeTreeParse(a->childs[1],b->childs[0],pairs);
			TreeTreeParse(a->childs[0],b->childs[1],pairs);
			TreeTreeParse(a->childs[1],b->childs[1],pairs);
			}
			else
			{
			TreeTreeParse(a->childs[0],b,pairs);
			TreeTreeParse(a->childs[1],b,pairs);
			}
		}
		else
		{
		if(b->isinternal())
			{
			TreeTreeParse(a,b->childs[0],pairs);
			TreeTreeParse(a,b->childs[1],pairs);
			}
			else
			{
			btDbvtPair	p;
			p.proxies[0]=(btDbvtProxy*)a->data;
			p.proxies[1]=(btDbvtProxy*)b->data;
			if(p.proxies[0]->aabb.Intersect(p.proxies[1]->aabb))
				{
				pairs.push_back(p);
				}
			}
		}
	}
}

void DbvtTest::PerformTest()
{
btDbvtBroadphase*	pb=(btDbvtBroadphase*)m_broadphase;
const btScalar		speed=m_speed;
const btScalar		ampl=m_amp;
const int			nupd=m_bfirsttime?	pb->m_proxies.size():
										(pb->m_proxies.size()*10)/100;
/* Move boxes		*/ 
m_profiler.Start();
for(int i=0;i<nupd;i++)
	{
	const btScalar	time=(m_times[i]+=speed);
	Point			center(	cosf(time*2.17)*ampl+sinf(time)*ampl*0.5,
							cosf(time*1.38)*ampl+sinf(time*ampl),
							sinf(time*0.777)*ampl);
	Point			extent;	
	pb->m_proxies[i]->aabb.GetExtents(extent);
	pb->m_proxies[i]->aabb.SetCenterExtents(center,extent);
	pb->m_dbvt->Update(	pb->m_proxies[i]->leaf,
						btVector3(center.x,center.y,center.z),
						btVector3(extent.x,extent.y,extent.z),
						m_margin);
	}
if(m_bfirsttime)
	{
	pb->m_dbvt->OptimizeTopDown();
	}
/* Overlap			*/ 
pb->m_pairs.resize(0);
TreeTreeParse(	pb->m_dbvt->m_root,
				pb->m_dbvt->m_root,
				pb->m_pairs);
m_profiler.End();
m_profiler.Accum();
#if 0
int				check=0;
for(int i=0,ni=pb->m_proxies.size();i<ni;++i)
	{
	AABB	boxa=pb->m_proxies[i]->aabb;
	for(int j=i+1;j<ni;++j)
		{
		if(boxa.Intersect(pb->m_proxies[j]->aabb))
			{
			++check;
			}
		}
	}
if(check!=pb->m_pairs.size())
	{
	printf("Check failed %u => %u\r\n",check,pb->m_pairs.size());
	}
#endif
/* Render boxes		*/ 
OBB CurrentBox;
CurrentBox.mRot.Identity();
for(int i=0,ni=pb->m_proxies.size();i<ni;i++)
	{
	pb->m_proxies[i]->contact=false;
	}
for(int i=0,ni=pb->m_pairs.size();i<ni;i++)
	{
	pb->m_pairs[i].proxies[0]->contact=true;
	pb->m_pairs[i].proxies[1]->contact=true;
	}
for(int i=0,ni=pb->m_proxies.size();i<ni;i++)
	{
	if(pb->m_proxies[i]->contact)	glColor3f(1.0f, 0.0f, 0.0f);
	else							glColor3f(0.0f, 1.0f, 0.0f);
	pb->m_proxies[i]->aabb.GetCenter(CurrentBox.mCenter);
	pb->m_proxies[i]->aabb.GetExtents(CurrentBox.mExtents);
	DrawOBB(CurrentBox);
	}
char Buffer[4096];
sprintf(Buffer,"Dbvt: %5.1f us (%d cycles) : %d pairs\n",m_profiler.mMsTime,m_profiler.mCycles,pb->m_pairs.size());
GLFontRenderer::print(10.0f, 10.0f, 0.02f, Buffer);
m_bfirsttime=false;
}

void DbvtTest::KeyboardCallback(unsigned char key, int x, int y)
{
}

void DbvtTest::MouseCallback(int button, int state, int x, int y)
{
}

void DbvtTest::MotionCallback(int x, int y)
{
}
