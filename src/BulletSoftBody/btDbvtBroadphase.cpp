/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
///btDbvtBroadphase implementation by Nathanael Presson

#include "btDbvtBroadphase.h"

//
// Profiling
//

#if DBVT_BP_PROFILE
#include <stdio.h>
struct	ProfileScope
	{
	ProfileScope(btClock& clock,unsigned long& value)
		{
		m_clock=&clock;
		m_value=&value;
		m_base=clock.getTimeMicroseconds();
		}
	~ProfileScope()
		{
		(*m_value)+=m_clock->getTimeMicroseconds()-m_base;
		}
	btClock*		m_clock;
	unsigned long*	m_value;
	unsigned long	m_base;
	};
#define	SPC(_value_)	ProfileScope	spc_scope(m_clock,_value_)
#else
#define	SPC(_value_)
#endif

//
// Helpers
//

//
static inline int	hash(unsigned int i,unsigned int j)
{
int key=((unsigned int)i)|(((unsigned int)j)<<16);
key+=~(key<<15);
key^= (key>>10);
key+= (key<<3);
key^= (key>>6);
key+=~(key<<11);
key^= (key>>16);
return(key);
}

//
template <typename T>
static inline void	listappend(T* item,T*& list)
{
item->links[0]=0;
item->links[1]=list;
if(list) list->links[0]=item;
list=item;
}

//
template <typename T>
static inline void	listremove(T* item,T*& list)
{
if(item->links[0]) item->links[0]->links[1]=item->links[1]; else list=item->links[1];
if(item->links[1]) item->links[1]->links[0]=item->links[0];
}

//
template <typename T>
static inline int	listcount(T* root)
{
int	n=0;
while(root) { ++n;root=root->links[1]; }
return(n);
}

//
template <typename T>
static inline void	clear(T& value)
{
static const T zerodummy;
value=zerodummy;
}

//
// Collider
//
struct	btDbvtBroadphaseCollider : btDbvt::ICollide
{
btDbvtBroadphase*	pbp;
int					pid;
		btDbvtBroadphaseCollider(btDbvtBroadphase* p,int id) : pbp(p),pid(id) {}
void	Process(const btDbvt::Node* na,const btDbvt::Node* nb)
	{
	btDbvtProxy*	pa=(btDbvtProxy*)na->data;
	btDbvtProxy*	pb=(btDbvtProxy*)nb->data;
	#if DBVT_BP_DISCRETPAIRS
	if(Intersect(pa->aabb,pb->aabb))
	#endif
		{
		btBroadphasePair*	pp=pbp->m_paircache->addOverlappingPair(pa,pb);
		if(pp) pp->m_userInfo=*(void**)&pid;
		}
	}
};

//
// btDbvtBroadphase
//

//
btDbvtBroadphase::btDbvtBroadphase()
{
m_fcursor			=	0;
m_dcursor			=	0;
m_stageCurrent		=	0;
m_fupdates			=	1;
m_dupdates			=	0;
m_paircache			=	new btHashedOverlappingPairCache();
m_gid				=	0;
m_pid				=	0;
for(int i=0;i<=STAGECOUNT;++i)
	{
	m_stageRoots[i]=0;
	}
#if DBVT_BP_PROFILE
clear(m_profiling);
#endif
}

//
btDbvtBroadphase::~btDbvtBroadphase()
{
delete m_paircache;
}

//
btBroadphaseProxy*				btDbvtBroadphase::createProxy(	const btVector3& aabbMin,
																const btVector3& aabbMax,
																int shapeType,
																void* userPtr,
																short int collisionFilterGroup,
																short int collisionFilterMask,
																btDispatcher* dispatcher,
																void* multiSapProxy)
{
btDbvtProxy*	proxy=new btDbvtProxy(userPtr,collisionFilterGroup,collisionFilterMask);
proxy->aabb			=	btDbvtAabbMm::FromMM(aabbMin,aabbMax);
proxy->leaf			=	m_sets[0].insert(proxy->aabb,proxy);
proxy->stage		=	m_stageCurrent;
proxy->m_uniqueId	=	++m_gid;
listappend(proxy,m_stageRoots[m_stageCurrent]);
return(proxy);
}

//
void							btDbvtBroadphase::destroyProxy(	btBroadphaseProxy* absproxy,
																btDispatcher* dispatcher)
{
btDbvtProxy*	proxy=(btDbvtProxy*)absproxy;
if(proxy->stage==STAGECOUNT)
	m_sets[1].remove(proxy->leaf);
	else
	m_sets[0].remove(proxy->leaf);
listremove(proxy,m_stageRoots[proxy->stage]);
m_paircache->removeOverlappingPairsContainingProxy(proxy,dispatcher);
delete proxy;
}

//
void							btDbvtBroadphase::setAabb(		btBroadphaseProxy* absproxy,
																const btVector3& aabbMin,
																const btVector3& aabbMax,
																btDispatcher* dispatcher)
{
btDbvtProxy*	proxy=(btDbvtProxy*)absproxy;
btDbvtAabbMm	aabb=btDbvtAabbMm::FromMM(aabbMin,aabbMax);
if(proxy->stage==STAGECOUNT)
	{/* fixed -> dynamic set	*/ 
	m_sets[1].remove(proxy->leaf);
	proxy->leaf=m_sets[0].insert(aabb,proxy);
	m_fcursor=0;
	}
	else
	{/* dynamic set				*/ 
	const btVector3	delta=(aabbMin+aabbMax)/2-proxy->aabb.Center();
	m_sets[0].update(proxy->leaf,aabb,delta*PREDICTED_FRAMES,DBVT_BP_MARGIN);		
	}
listremove(proxy,m_stageRoots[proxy->stage]);
proxy->aabb		=	aabb;
proxy->stage	=	m_stageCurrent;
listappend(proxy,m_stageRoots[m_stageCurrent]);
}

//
void							btDbvtBroadphase::calculateOverlappingPairs(btDispatcher* dispatcher)
{
collide(dispatcher);
#if DBVT_BP_PROFILE
if(0==(m_pid%DBVT_BP_PROFILING_RATE))
	{	
	printf("fixed(%u) dynamics(%u) pairs(%u)\r\n",m_sets[1].m_leafs,m_sets[0].m_leafs,m_paircache->getNumOverlappingPairs());
	unsigned int	total=m_profiling.m_total;
	if(total<=0) total=1;
	printf("ddcollide: %u%% (%uus)\r\n",(50+m_profiling.m_ddcollide*100)/total,m_profiling.m_ddcollide/DBVT_BP_PROFILING_RATE);
	printf("fdcollide: %u%% (%uus)\r\n",(50+m_profiling.m_fdcollide*100)/total,m_profiling.m_fdcollide/DBVT_BP_PROFILING_RATE);
	printf("cleanup:   %u%% (%uus)\r\n",(50+m_profiling.m_cleanup*100)/total,m_profiling.m_cleanup/DBVT_BP_PROFILING_RATE);
	printf("total:     %uus\r\n",total/DBVT_BP_PROFILING_RATE);
	const unsigned long	sum=m_profiling.m_ddcollide+
							m_profiling.m_fdcollide+
							m_profiling.m_cleanup;
	printf("leaked: %u%% (%uus)\r\n",100-((50+sum*100)/total),(total-sum)/DBVT_BP_PROFILING_RATE);
	clear(m_profiling);
	m_clock.reset();
	}
#endif
}

//
void							btDbvtBroadphase::collide(btDispatcher* dispatcher)
{
SPC(m_profiling.m_total);
/* refine dynamic		*/ 
if(m_stageRoots[m_stageCurrent]&&(m_dupdates>0))
	{
	const int	count=1+(m_sets[0].m_leafs*m_dupdates)/100;
	for(int i=0;i<count;++i)
		{
		if(!m_dcursor) m_dcursor=m_stageRoots[m_stageCurrent];
		m_sets[0].update(m_dcursor->leaf);
		m_dcursor=m_dcursor->links[1];
		}
	}
/* dynamic -> fixed set	*/ 
m_stageCurrent=(m_stageCurrent+1)%STAGECOUNT;
btDbvtProxy*	current=m_stageRoots[m_stageCurrent];
if(current)
	{
	btDbvtBroadphaseCollider	collider(this,m_pid);
	do	{
		btDbvtProxy*	next=current->links[1];
		if(m_dcursor==current) m_dcursor=0;
		listremove(current,m_stageRoots[current->stage]);
		listappend(current,m_stageRoots[STAGECOUNT]);
		m_sets[1].collideGeneric(current->leaf,collider);
		m_sets[0].remove(current->leaf);
		current->leaf	=	m_sets[1].insert(current->aabb,current);
		current->stage	=	STAGECOUNT;	
		current			=	next;
		} while(current);
	}
/* refine fixed			*/ 
if(m_stageRoots[STAGECOUNT]&&(m_fupdates>0))
	{
	const int	count=1+(m_sets[1].m_leafs*m_fupdates)/100;
	for(int i=0;i<count;++i)
		{
		if(!m_fcursor) m_fcursor=m_stageRoots[STAGECOUNT];
		m_sets[1].update(m_fcursor->leaf);
		m_fcursor=m_fcursor->links[1];		
		}
	}
/* collide dynamics		*/ 
btDbvtBroadphaseCollider	collider(this,m_pid);
	{
	SPC(m_profiling.m_fdcollide);
	m_sets[0].collideGeneric(&m_sets[1],collider);
	}
	{
	SPC(m_profiling.m_ddcollide);
	m_sets[0].collideGeneric(&m_sets[0],collider);
	}
/* clean up				*/ 
	{
	SPC(m_profiling.m_cleanup);
	btBroadphasePairArray&	pairs=m_paircache->getOverlappingPairArray();
	for(int i=0,ni=pairs.size();i<ni;++i)
		{
		btBroadphasePair&	p=pairs[i];
		if(m_pid!=(*(int*)&p.m_userInfo))
			{
			btDbvtProxy*	pa=(btDbvtProxy*)p.m_pProxy0;
			btDbvtProxy*	pb=(btDbvtProxy*)p.m_pProxy1;
			if(!Intersect(pa->aabb,pb->aabb))
				{
				m_paircache->removeOverlappingPair(pa,pb,dispatcher);
				--ni;--i;
				}
			}
		}
	}
++m_pid;
}

//
btOverlappingPairCache*			btDbvtBroadphase::getOverlappingPairCache()
{
return(m_paircache);
}

//
const btOverlappingPairCache*	btDbvtBroadphase::getOverlappingPairCache() const
{
return(m_paircache);
}

//
void							btDbvtBroadphase::getBroadphaseAabb(btVector3& aabbMin,btVector3& aabbMax) const
{
btDbvtAabbMm	bounds;
if(!m_sets[0].empty())
	if(!m_sets[1].empty())	Merge(	m_sets[0].m_root->volume,
									m_sets[1].m_root->volume,bounds);
							else
							bounds=m_sets[0].m_root->volume;
else if(!m_sets[1].empty())	bounds=m_sets[1].m_root->volume;
							else
							bounds=btDbvtAabbMm::FromCR(btVector3(0,0,0),0);
aabbMin=bounds.Mins();
aabbMax=bounds.Maxs();
}

//
void							btDbvtBroadphase::printStats()
{}

#if DBVT_BP_PROFILE
#undef	SPC
#endif
