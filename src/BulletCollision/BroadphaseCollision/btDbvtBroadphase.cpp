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
static const struct ZeroDummy : T {} zerodummy;
value=zerodummy;
}

//
// Colliders
//

/* Leaf collider	*/ 
struct	btDbvtLeafCollider : btDbvt::ICollide
{
btDbvtBroadphase*	pbp;
btDbvtProxy*		ppx;
		btDbvtLeafCollider(btDbvtBroadphase* p,btDbvtProxy* px) : pbp(p),ppx(px) {}
void	Process(const btDbvt::Node* na)
	{
	const btDbvt::Node*	nb=ppx->leaf;
	if(nb!=na)
		{
		btDbvtProxy*		pa=(btDbvtProxy*)na->data;
		btDbvtProxy*		pb=(btDbvtProxy*)nb->data;
		#if DBVT_BP_DISCRETPAIRS
		if(Intersect(pa->aabb,pb->aabb))
		#endif
			{
			if(pa>pb) btSwap(pa,pb);
			pbp->m_paircache->addOverlappingPair(pa,pb);
			}
		}
	}
};

/* Tree collider	*/ 
struct	btDbvtTreeCollider : btDbvt::ICollide
{
btDbvtBroadphase*	pbp;
		btDbvtTreeCollider(btDbvtBroadphase* p) : pbp(p) {}
void	Process(const btDbvt::Node* na,const btDbvt::Node* nb)
	{
	btDbvtProxy*	pa=(btDbvtProxy*)na->data;
	btDbvtProxy*	pb=(btDbvtProxy*)nb->data;
	#if DBVT_BP_DISCRETPAIRS
	if(Intersect(pa->aabb,pb->aabb))
	#endif
		{
		if(pa>pb) btSwap(pa,pb);
		pbp->m_paircache->addOverlappingPair(pa,pb);
		}
	}
};

//
// btDbvtBroadphase
//

//
btDbvtBroadphase::btDbvtBroadphase(btOverlappingPairCache* paircache)
{
m_releasepaircache	=	(paircache!=0)?false:true;
m_stageCurrent		=	0;
m_fupdates			=	1;
m_dupdates			=	1;
m_paircache			=	paircache?
							paircache	:
							new(btAlignedAlloc(sizeof(btHashedOverlappingPairCache),16)) btHashedOverlappingPairCache();
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
if(m_releasepaircache) btAlignedFree(m_paircache);
}

//
btBroadphaseProxy*				btDbvtBroadphase::createProxy(	const btVector3& aabbMin,
																const btVector3& aabbMax,
																int /*shapeType*/,
																void* userPtr,
																short int collisionFilterGroup,
																short int collisionFilterMask,
																btDispatcher* /*dispatcher*/,
																void* /*multiSapProxy*/)
{
btDbvtProxy*	proxy=new(btAlignedAlloc(sizeof(btDbvtProxy),16)) btDbvtProxy(	userPtr,
																				collisionFilterGroup,
																				collisionFilterMask);
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
btAlignedFree(proxy);
}

//
void							btDbvtBroadphase::setAabb(		btBroadphaseProxy* absproxy,
																const btVector3& aabbMin,
																const btVector3& aabbMax,
																btDispatcher* /*dispatcher*/)
{
btDbvtProxy*	proxy=(btDbvtProxy*)absproxy;
btDbvtAabbMm	aabb=btDbvtAabbMm::FromMM(aabbMin,aabbMax);
if(proxy->stage==STAGECOUNT)
	{/* fixed -> dynamic set	*/ 
	m_sets[1].remove(proxy->leaf);
	proxy->leaf=m_sets[0].insert(aabb,proxy);
	}
	else
	{/* dynamic set				*/ 
	if(Intersect(proxy->leaf->volume,aabb))
		{/* Moving				*/ 
		const btVector3	delta=(aabbMin+aabbMax)/2-proxy->aabb.Center();
		#ifdef DBVT_BP_MARGIN
		m_sets[0].update(proxy->leaf,aabb,delta*PREDICTED_FRAMES,DBVT_BP_MARGIN);
		#else
		m_sets[0].update(proxy->leaf,aabb,delta*PREDICTED_FRAMES);
		#endif
		}
		else
		{/* Teleporting			*/ 
		m_sets[0].update(proxy->leaf,aabb);		
		}	
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
	printf("mode:    %s\r\n",m_mode==MODE_FULL?"full":"incremental");
	printf("cleanup: %s\r\n",m_cleanupmode==CLEANUP_FULL?"full":"incremental");
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
	printf("job counts: %u%%\r\n",(m_profiling.m_jobcount*100)/((m_sets[0].m_leafs+m_sets[1].m_leafs)*DBVT_BP_PROFILING_RATE));
	clear(m_profiling);
	m_clock.reset();
	}
#endif
}

//
void							btDbvtBroadphase::collide(btDispatcher* dispatcher)
{
SPC(m_profiling.m_total);
/* optimize				*/ 
m_sets[0].optimizeIncremental(1+(m_sets[0].m_leafs*m_dupdates)/100);
m_sets[1].optimizeIncremental(1+(m_sets[1].m_leafs*m_fupdates)/100);
/* dynamic -> fixed set	*/ 
m_stageCurrent=(m_stageCurrent+1)%STAGECOUNT;
btDbvtProxy*	current=m_stageRoots[m_stageCurrent];
if(current)
	{
	btDbvtTreeCollider	collider(this);
	do	{
		btDbvtProxy*	next=current->links[1];
		listremove(current,m_stageRoots[current->stage]);
		listappend(current,m_stageRoots[STAGECOUNT]);
		btDbvt::collideTT(m_sets[1].m_root,current->leaf,collider);
		m_sets[0].remove(current->leaf);
		current->leaf	=	m_sets[1].insert(current->aabb,current);
		current->stage	=	STAGECOUNT;	
		current			=	next;
		} while(current);
	}
/* collide dynamics		*/ 
	{
	btDbvtTreeCollider	collider(this);
		{
		SPC(m_profiling.m_fdcollide);
		btDbvt::collideTT(m_sets[0].m_root,m_sets[1].m_root,collider);
		}
		{
		SPC(m_profiling.m_ddcollide);
		btDbvt::collideTT(m_sets[0].m_root,m_sets[0].m_root,collider);
		}
	}
/* clean up				*/ 
	{
	SPC(m_profiling.m_cleanup);
	btBroadphasePairArray&	pairs=m_paircache->getOverlappingPairArray();
	if(pairs.size()>0)
		{
		for(int i=0,ni=pairs.size();i<ni;++i)
			{
			btBroadphasePair&	p=pairs[i];
			btDbvtProxy*	pa=(btDbvtProxy*)p.m_pProxy0;
			btDbvtProxy*	pb=(btDbvtProxy*)p.m_pProxy1;
			if(!Intersect(pa->aabb,pb->aabb))
				{
				if(pa>pb) btSwap(pa,pb);
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
