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
///btDbvt implementation by Nathanael Presson

#include "btDbvt.h"

//
typedef btAlignedObjectArray<btDbvt::Node*>			tNodeArray;
typedef btAlignedObjectArray<const btDbvt::Node*>	tConstNodeArray;

//
struct btDbvtNodeEnumerator : btDbvt::ICollide
{
tConstNodeArray	nodes;
void Process(const btDbvt::Node* n) { nodes.push_back(n); }
};

//
static inline int				indexof(const btDbvt::Node* node)
{
return(node->parent->childs[1]==node);
}

//
static inline btDbvt::Volume	merge(		const btDbvt::Volume& a,
											const btDbvt::Volume& b)
{
btDbvt::Volume	res;
Merge(a,b,res);
return(res);
}

// volume+edge lengths
static inline btScalar			size(const btDbvt::Volume& a)
{
const btVector3	edges=a.Lengths();
return(	edges.x()*edges.y()*edges.z()+
		edges.x()+edges.y()+edges.z());
}

//
static inline void				deletenode(	btDbvt* pdbvt,
											btDbvt::Node* node)
{
btAlignedFree(pdbvt->m_free);
pdbvt->m_free=node;
}
	
//
static inline void				recursedeletenode(	btDbvt* pdbvt,
													btDbvt::Node* node)
{
if(!node->isleaf())
	{
	recursedeletenode(pdbvt,node->childs[0]);
	recursedeletenode(pdbvt,node->childs[1]);
	}
if(node==pdbvt->m_root) pdbvt->m_root=0;
deletenode(pdbvt,node);
}

//
static inline btDbvt::Node*		createnode(	btDbvt* pdbvt,
											btDbvt::Node* parent,
											const btDbvt::Volume& volume,
											void* data)
{
btDbvt::Node*	node;
if(pdbvt->m_free)
	{ node=pdbvt->m_free;pdbvt->m_free=0; }
	else
	{ node=new(btAlignedAlloc(sizeof(btDbvt::Node),16)) btDbvt::Node(); }
node->parent	=	parent;
node->volume	=	volume;
node->data		=	data;
node->childs[1]	=	0;
return(node);
}

//
static inline void				insertleaf(	btDbvt* pdbvt,
											btDbvt::Node* root,
											btDbvt::Node* leaf)
{
if(!pdbvt->m_root)
	{
	pdbvt->m_root	=	leaf;
	leaf->parent	=	0;
	}
	else
	{
	if(!root->isleaf())
		{
		do	{
			if(	Proximity(root->childs[0]->volume,leaf->volume)<
				Proximity(root->childs[1]->volume,leaf->volume))
				root=root->childs[0];
				else
				root=root->childs[1];
			} while(!root->isleaf());
		}
	btDbvt::Node*	prev=root->parent;
	btDbvt::Node*	node=createnode(pdbvt,prev,merge(leaf->volume,root->volume),0);
	if(prev)
		{
		prev->childs[indexof(root)]	=	node;
		node->childs[0]				=	root;root->parent=node;
		node->childs[1]				=	leaf;leaf->parent=node;
		do	{
			if(!prev->volume.Contain(node->volume))
				Merge(prev->childs[0]->volume,prev->childs[1]->volume,prev->volume);
				else
				break;
			node=prev;
			} while(0!=(prev=node->parent));
		}
		else
		{
		node->childs[0]	=	root;root->parent=node;
		node->childs[1]	=	leaf;leaf->parent=node;
		pdbvt->m_root	=	node;
		}
	}
}
	
//
static inline btDbvt::Node*		removeleaf(	btDbvt* pdbvt,
											btDbvt::Node* leaf)
{
if(leaf==pdbvt->m_root)
	{
	pdbvt->m_root=0;
	return(0);
	}
	else
	{
	btDbvt::Node*	parent=leaf->parent;
	btDbvt::Node*	prev=parent->parent;
	btDbvt::Node*	sibling=parent->childs[1-indexof(leaf)];			
	if(prev)
		{
		prev->childs[indexof(parent)]=sibling;
		sibling->parent=prev;
		deletenode(pdbvt,parent);
		while(prev)
			{
			const btDbvt::Volume	pb=prev->volume;
			Merge(prev->childs[0]->volume,prev->childs[1]->volume,prev->volume);
			if(NotEqual(pb,prev->volume))
				{
				prev=prev->parent;
				} else break;
			}
		return(prev?prev:pdbvt->m_root);
		}
		else
		{								
		pdbvt->m_root=sibling;
		sibling->parent=0;
		deletenode(pdbvt,parent);
		return(pdbvt->m_root);
		}			
	}
}

//
static void						fetchleaves(btDbvt* pdbvt,
											btDbvt::Node* root,
											tNodeArray& leaves,
											int depth=-1)
{
if(root->isinternal()&&depth)
	{
	fetchleaves(pdbvt,root->childs[0],leaves,depth-1);
	fetchleaves(pdbvt,root->childs[1],leaves,depth-1);
	deletenode(pdbvt,root);
	}
	else
	{
	leaves.push_back(root);
	}
}

//
static void						split(	const tNodeArray& leaves,
										tNodeArray& left,
										tNodeArray& right,
										const btVector3& org,
										const btVector3& axis)
{
left.resize(0);
right.resize(0);
for(int i=0,ni=leaves.size();i<ni;++i)
	{
	if(dot(axis,leaves[i]->volume.Center()-org)<0)
		left.push_back(leaves[i]);
		else
		right.push_back(leaves[i]);
	}
}

//
static btDbvt::Volume			bounds(	const tNodeArray& leaves)
{
btDbvt::Volume	volume=leaves[0]->volume;
for(int i=1,ni=leaves.size();i<ni;++i)
	{
	volume=merge(volume,leaves[i]->volume);
	}
return(volume);
}

//
static void						bottomup(	btDbvt* pdbvt,
											tNodeArray& leaves)
{
while(leaves.size()>1)
	{
	btScalar	minsize=SIMD_INFINITY;
	int			minidx[2]={-1,-1};
	for(int i=0;i<leaves.size();++i)
		{
		for(int j=i+1;j<leaves.size();++j)
			{
			const btScalar	sz=size(merge(leaves[i]->volume,leaves[j]->volume));
			if(sz<minsize)
				{
				minsize		=	sz;
				minidx[0]	=	i;
				minidx[1]	=	j;
				}
			}
		}
	btDbvt::Node*	n[]	=	{leaves[minidx[0]],leaves[minidx[1]]};
	btDbvt::Node*	p	=	createnode(pdbvt,0,merge(n[0]->volume,n[1]->volume),0);
	p->childs[0]		=	n[0];
	p->childs[1]		=	n[1];
	n[0]->parent		=	p;
	n[1]->parent		=	p;
	leaves[minidx[0]]	=	p;
	leaves.swap(minidx[1],leaves.size()-1);
	leaves.pop_back();
	}
}

//
static btDbvt::Node*			topdown(btDbvt* pdbvt,
										tNodeArray& leaves,
										int bu_treshold)
{
static const btVector3	axis[]={btVector3(1,0,0),
								btVector3(0,1,0),
								btVector3(0,0,1)};
if(leaves.size()>1)
	{
	if(leaves.size()>bu_treshold)
		{
		const btDbvt::Volume	vol=bounds(leaves);
		const btVector3			org=vol.Center();
		tNodeArray				sets[2];
		int						bestaxis=-1;
		int						bestmidp=leaves.size();
		int						splitcount[3][2]={{0,0},{0,0},{0,0}};
		for(int i=0;i<leaves.size();++i)
			{
			const btVector3	x=leaves[i]->volume.Center()-org;
			for(int j=0;j<3;++j)
				{
				++splitcount[j][dot(x,axis[j])>0?1:0];
				}
			}
		for(int i=0;i<3;++i)
			{
			if((splitcount[i][0]>0)&&(splitcount[i][1]>0))
				{
				const int	midp=(int)btFabs(btScalar(splitcount[i][0]-splitcount[i][1]));
				if(midp<bestmidp)
					{
					bestaxis=i;
					bestmidp=midp;
					}
				}
			}
		if(bestaxis>=0)
			{
			sets[0].reserve(splitcount[bestaxis][0]);
			sets[1].reserve(splitcount[bestaxis][1]);
			split(leaves,sets[0],sets[1],org,axis[bestaxis]);
			}
			else
			{
			sets[0].reserve(leaves.size()/2+1);
			sets[1].reserve(leaves.size()/2);
			for(int i=0,ni=leaves.size();i<ni;++i)
				{
				sets[i&1].push_back(leaves[i]);
				}
			}
		btDbvt::Node*	node=createnode(pdbvt,0,vol,0);
		node->childs[0]=topdown(pdbvt,sets[0],bu_treshold);
		node->childs[1]=topdown(pdbvt,sets[1],bu_treshold);
		node->childs[0]->parent=node;
		node->childs[1]->parent=node;
		return(node);
		}
		else
		{
		bottomup(pdbvt,leaves);
		return(leaves[0]);
		}
	}
return(leaves[0]);
}

//
static inline btDbvt::Node*		sort(btDbvt::Node* n,btDbvt::Node*& r)
{
btDbvt::Node*	p=n->parent;
btAssert(n->isinternal());
if(p>n)
	{
	const int		i=indexof(n);
	const int		j=1-i;
	btDbvt::Node*	s=p->childs[j];
	btDbvt::Node*	q=p->parent;
	btAssert(n==p->childs[i]);
	if(q) q->childs[indexof(p)]=n; else r=n;
	s->parent=n;
	p->parent=n;
	n->parent=q;
	p->childs[0]=n->childs[0];
	p->childs[1]=n->childs[1];
	n->childs[0]->parent=p;
	n->childs[1]->parent=p;
	n->childs[i]=p;
	n->childs[j]=s;
	btSwap(p->volume,n->volume);
	return(p);
	}
return(n);
}

//
static inline btDbvt::Node*		walkup(btDbvt::Node* n,int count)
{
while(n&&(count--)) n=n->parent;
return(n);
}

//
// Api
//

//
				btDbvt::btDbvt()
{
m_root		=	0;
m_free		=	0;
m_lkhd		=	-1;
m_leaves	=	0;
m_opath		=	0;
}

//
				btDbvt::~btDbvt()
{
clear();
}

//
void			btDbvt::clear()
{
if(m_root)	recursedeletenode(this,m_root);
btAlignedFree(m_free);
m_free=0;
}

//
void			btDbvt::optimizeBottomUp()
{
if(m_root)
	{
	tNodeArray leaves;
	leaves.reserve(m_leaves);
	fetchleaves(this,m_root,leaves);
	bottomup(this,leaves);
	m_root=leaves[0];
	}
}

//
void			btDbvt::optimizeTopDown(int bu_treshold)
{
if(m_root)
	{
	tNodeArray	leaves;
	leaves.reserve(m_leaves);
	fetchleaves(this,m_root,leaves);
	m_root=topdown(this,leaves,bu_treshold);
	}
}

//
void			btDbvt::optimizeIncremental(int passes)
{
if(passes<0) passes=m_leaves;
if(m_root&&(passes>0))
	{
	do	{
		Node*		node=m_root;
		unsigned	bit=0;
		while(node->isinternal())
			{
			node=sort(node,m_root)->childs[(m_opath>>bit)&1];
			bit=(bit+1)&(sizeof(unsigned)*8-1);
			}
		update(node);
		++m_opath;
		} while(--passes);
	}
}

//
btDbvt::Node*	btDbvt::insert(const Volume& volume,void* data)
{
Node*	leaf=createnode(this,0,volume,data);
insertleaf(this,m_root,leaf);
++m_leaves;
return(leaf);
}

//
void			btDbvt::update(Node* leaf,int lookahead)
{
Node*	root=removeleaf(this,leaf);
if(root)
	{
	if(lookahead>=0)
		{
		for(int i=0;(i<lookahead)&&root->parent;++i)
			{
			root=root->parent;
			}
		} else root=m_root;
	}
insertleaf(this,root,leaf);
}

//
void			btDbvt::update(Node* leaf,const Volume& volume)
{
Node*	root=removeleaf(this,leaf);
if(root)
	{
	if(m_lkhd>=0)
		{
		for(int i=0;(i<m_lkhd)&&root->parent;++i)
			{
			root=root->parent;
			}
		} else root=m_root;
	}
leaf->volume=volume;
insertleaf(this,root,leaf);
}

//
bool			btDbvt::update(Node* leaf,Volume volume,const btVector3& velocity,btScalar margin)
{
if(leaf->volume.Contain(volume)) return(false);
volume.Expand(btVector3(margin,margin,margin));
volume.SignedExpand(velocity);
update(leaf,volume);
return(true);
}

//
bool			btDbvt::update(Node* leaf,Volume volume,const btVector3& velocity)
{
if(leaf->volume.Contain(volume)) return(false);
volume.SignedExpand(velocity);
update(leaf,volume);
return(true);
}

//
bool			btDbvt::update(Node* leaf,Volume volume,btScalar margin)
{
if(leaf->volume.Contain(volume)) return(false);
volume.Expand(btVector3(margin,margin,margin));
update(leaf,volume);
return(true);
}

//
void			btDbvt::remove(Node* leaf)
{
removeleaf(this,leaf);
deletenode(this,leaf);
--m_leaves;
}

//
void			btDbvt::write(IWriter* iwriter) const
{
btDbvtNodeEnumerator	nodes;
nodes.nodes.reserve(m_leaves*2);
enumNodes(m_root,nodes);
iwriter->Prepare(m_root,nodes.nodes.size());
for(int i=0;i<nodes.nodes.size();++i)
	{
	const Node* n=nodes.nodes[i];
	int			p=-1;
	if(n->parent) p=nodes.nodes.findLinearSearch(n->parent);
	if(n->isinternal())
		{
		const int	c0=nodes.nodes.findLinearSearch(n->childs[0]);
		const int	c1=nodes.nodes.findLinearSearch(n->childs[1]);
		iwriter->WriteNode(n,i,p,c0,c1);
		}
		else
		{
		iwriter->WriteLeaf(n,i,p);
		}	
	}
}

//
void			btDbvt::clone(btDbvt& dest,IClone* iclone) const
{
dest.clear();
if(m_root!=0)
	{	
	btAlignedObjectArray<sStkCLN>	stack;
	stack.reserve(m_leaves);
	stack.push_back(sStkCLN(m_root,0));
	do	{
		const int		i=stack.size()-1;
		const sStkCLN	e=stack[i];
		Node*			n=createnode(&dest,e.parent,e.node->volume,e.node->data);
		stack.pop_back();
		if(e.parent!=0)
			e.parent->childs[i&1]=n;
			else
			dest.m_root=n;
		if(e.node->isinternal())
			{
			stack.push_back(sStkCLN(e.node->childs[0],n));
			stack.push_back(sStkCLN(e.node->childs[1],n));
			}
			else
			{
			iclone->CloneLeaf(n);
			}
		} while(stack.size()>0);
	}
}

//
int				btDbvt::countLeaves(const Node* node)
{
if(node->isinternal())
	return(countLeaves(node->childs[0])+countLeaves(node->childs[1]));
	else
	return(1);
}

//
void			btDbvt::extractLeaves(const Node* node,btAlignedObjectArray<const Node*>& leaves)
{
if(node->isinternal())
	{
	extractLeaves(node->childs[0],leaves);
	extractLeaves(node->childs[1],leaves);
	}
	else
	{
	leaves.push_back(node);
	}	
}

//
#if DBVT_ENABLE_BENCHMARK

#include <stdio.h>
#include <stdlib.h>
#include "LinearMath/btQuickProf.h"

/*
q6600,2.4ghz

/Ox /Ob2 /Oi /Ot /I "." /I "..\.." /I "..\..\src" /D "NDEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"
/GF /FD /MT /GS- /Gy /arch:SSE2 /Zc:wchar_t- /Fp"..\..\out\release8\build\libbulletcollision\libbulletcollision.pch"
/Fo"..\..\out\release8\build\libbulletcollision\\"
/Fd"..\..\out\release8\build\libbulletcollision\bulletcollision.pdb"
/W3 /nologo /c /Wp64 /Zi /errorReport:prompt

Benchmarking dbvt...
        World scale: 100.000000
        Extents base: 1.000000
        Extents range: 4.000000
        Leaves: 8192
[1] btDbvt::Volume intersections: 3986 ms (0%)
[2] btDbvt::Volume merges: 5815 ms (-1%)
[3] btDbvt::collideTT: 3267 ms (0%)
[4] btDbvt::collideTT self: 1657 ms (0%)
[5] btDbvt::collideTT xform: 7201 ms (0%)
[6] btDbvt::collideTT xform,self: 7382 ms (0%)
[7] btDbvt::collideRAY: 8855 ms (-1%),(236832 r/s)
[8] insert/remove: 3574 ms (0%),(586780 ir/s)
[9] updates (teleport): 3281 ms (0%),(639180 u/s)
[10] updates (jitter): 2658 ms (0%),(788996 u/s)
[11] optimize (incremental): 5091 ms (0%),(823000 o/s)
[12] btDbvt::Volume notequal: 4151 ms (0%)
[13] culling(OCL): 2486 ms (0%),(411 t/s)
[14] culling(OCL+qsort): 599 ms (-2%),(1709 t/s)
[15] culling(KDOP+qsort): 306 ms (0%),(3346 t/s)
*/

struct btDbvtBenchmark
{
struct NilPolicy : btDbvt::ICollide
	{
	NilPolicy() : m_pcount(0),m_depth(-SIMD_INFINITY),m_checksort(true)		{}
	void	Process(const btDbvt::Node*,const btDbvt::Node*)				{ ++m_pcount; }
	void	Process(const btDbvt::Node*)									{ ++m_pcount; }
	void	Process(const btDbvt::Node*,btScalar depth)
		{
		++m_pcount;
		if(m_checksort)
			{ if(depth>=m_depth) m_depth=depth; else printf("wrong depth: %f\r\n",depth); }
		}
	int			m_pcount;
	btScalar	m_depth;
	bool		m_checksort;
	};
struct P14 : btDbvt::ICollide
	{
	struct Node
		{
		const btDbvt::Node*	leaf;
		btScalar			depth;
		};
	void Process(const btDbvt::Node* leaf,btScalar depth)
		{
		Node	n;
		n.leaf	=	leaf;
		n.depth	=	depth;
		}
	static int sortfnc(const Node& a,const Node& b)
		{
		if(a.depth<b.depth) return(+1);
		if(a.depth>b.depth) return(-1);
		return(0);
		}
	btAlignedObjectArray<Node>		m_nodes;
	};
struct P15 : btDbvt::ICollide
	{
	struct Node
		{
		const btDbvt::Node*	leaf;
		btScalar			depth;
		};
	void Process(const btDbvt::Node* leaf)
		{
		Node	n;
		n.leaf	=	leaf;
		n.depth	=	dot(leaf->volume.Center(),m_axis);
		}
	static int sortfnc(const Node& a,const Node& b)
		{
		if(a.depth<b.depth) return(+1);
		if(a.depth>b.depth) return(-1);
		return(0);
		}
	btAlignedObjectArray<Node>		m_nodes;
	btVector3						m_axis;
	};
static btScalar			RandUnit()
	{
	return(rand()/(btScalar)RAND_MAX);
	}
static btVector3		RandVector3()
	{
	return(btVector3(RandUnit(),RandUnit(),RandUnit()));
	}
static btVector3		RandVector3(btScalar cs)
	{
	return(RandVector3()*cs-btVector3(cs,cs,cs)/2);
	}
static btDbvt::Volume	RandVolume(btScalar cs,btScalar eb,btScalar es)
	{
	return(btDbvt::Volume::FromCE(RandVector3(cs),btVector3(eb,eb,eb)+RandVector3()*es));
	}
static btTransform		RandTransform(btScalar cs)
	{
	btTransform	t;
	t.setOrigin(RandVector3(cs));
	t.setRotation(btQuaternion(RandUnit()*SIMD_PI*2,RandUnit()*SIMD_PI*2,RandUnit()*SIMD_PI*2).normalized());
	return(t);
	}
static void				RandTree(btScalar cs,btScalar eb,btScalar es,int leaves,btDbvt& dbvt)
	{
	dbvt.clear();
	for(int i=0;i<leaves;++i)
		{
		dbvt.insert(RandVolume(cs,eb,es),0);
		}
	}
};

void			btDbvt::benchmark()
{
static const btScalar	cfgVolumeCenterScale		=	100;
static const btScalar	cfgVolumeExentsBase			=	1;
static const btScalar	cfgVolumeExentsScale		=	4;
static const int		cfgLeaves					=	8192;
static const bool		cfgEnable					=	true;

//[1] btDbvt::Volume intersections
bool					cfgBenchmark1_Enable		=	cfgEnable;
static const int		cfgBenchmark1_Iterations	=	8;
static const int		cfgBenchmark1_Reference		=	3980;
//[2] btDbvt::Volume merges
bool					cfgBenchmark2_Enable		=	cfgEnable;
static const int		cfgBenchmark2_Iterations	=	4;
static const int		cfgBenchmark2_Reference		=	5924;
//[3] btDbvt::collideTT
bool					cfgBenchmark3_Enable		=	cfgEnable;
static const int		cfgBenchmark3_Iterations	=	256;
static const int		cfgBenchmark3_Reference		=	3288;
//[4] btDbvt::collideTT self
bool					cfgBenchmark4_Enable		=	cfgEnable;
static const int		cfgBenchmark4_Iterations	=	256;
static const int		cfgBenchmark4_Reference		=	1655;
//[5] btDbvt::collideTT xform
bool					cfgBenchmark5_Enable		=	cfgEnable;
static const int		cfgBenchmark5_Iterations	=	256;
static const btScalar	cfgBenchmark5_OffsetScale	=	2;
static const int		cfgBenchmark5_Reference		=	7201;
//[6] btDbvt::collideTT xform,self
bool					cfgBenchmark6_Enable		=	cfgEnable;
static const int		cfgBenchmark6_Iterations	=	256;
static const btScalar	cfgBenchmark6_OffsetScale	=	2;
static const int		cfgBenchmark6_Reference		=	7382;
//[7] btDbvt::collideRAY
bool					cfgBenchmark7_Enable		=	cfgEnable;
static const int		cfgBenchmark7_Passes		=	32;
static const int		cfgBenchmark7_Iterations	=	65536;
static const int		cfgBenchmark7_Reference		=	8954;
//[8] insert/remove
bool					cfgBenchmark8_Enable		=	cfgEnable;
static const int		cfgBenchmark8_Passes		=	32;
static const int		cfgBenchmark8_Iterations	=	65536;
static const int		cfgBenchmark8_Reference		=	3597;
//[9] updates (teleport)
bool					cfgBenchmark9_Enable		=	cfgEnable;
static const int		cfgBenchmark9_Passes		=	32;
static const int		cfgBenchmark9_Iterations	=	65536;
static const int		cfgBenchmark9_Reference		=	3282;
//[10] updates (jitter)
bool					cfgBenchmark10_Enable		=	cfgEnable;
static const btScalar	cfgBenchmark10_Scale		=	cfgVolumeCenterScale/10000;
static const int		cfgBenchmark10_Passes		=	32;
static const int		cfgBenchmark10_Iterations	=	65536;
static const int		cfgBenchmark10_Reference	=	2659;
//[11] optimize (incremental)
bool					cfgBenchmark11_Enable		=	cfgEnable;
static const int		cfgBenchmark11_Passes		=	64;
static const int		cfgBenchmark11_Iterations	=	65536;
static const int		cfgBenchmark11_Reference	=	5075;
//[12] btDbvt::Volume notequal
bool					cfgBenchmark12_Enable		=	cfgEnable;
static const int		cfgBenchmark12_Iterations	=	32;
static const int		cfgBenchmark12_Reference	=	4118;
//[13] culling(OCL+fullsort)
bool					cfgBenchmark13_Enable		=	cfgEnable;
static const int		cfgBenchmark13_Iterations	=	1024;
static const int		cfgBenchmark13_Reference	=	2483;
//[14] culling(OCL+qsort)
bool					cfgBenchmark14_Enable		=	cfgEnable;
static const int		cfgBenchmark14_Iterations	=	1024;
static const int		cfgBenchmark14_Reference	=	614;
//[15] culling(KDOP+qsort)
bool					cfgBenchmark15_Enable		=	cfgEnable;
static const int		cfgBenchmark15_Iterations	=	1024;
static const int		cfgBenchmark15_Reference	=	305;

btClock					wallclock;
printf("Benchmarking dbvt...\r\n");
printf("\tWorld scale: %f\r\n",cfgVolumeCenterScale);
printf("\tExtents base: %f\r\n",cfgVolumeExentsBase);
printf("\tExtents range: %f\r\n",cfgVolumeExentsScale);
printf("\tLeaves: %u\r\n",cfgLeaves);
if(cfgBenchmark1_Enable)
	{// Benchmark 1	
	srand(380843);
	btAlignedObjectArray<btDbvt::Volume>	volumes;
	btAlignedObjectArray<bool>				results;
	volumes.resize(cfgLeaves);
	results.resize(cfgLeaves);
	for(int i=0;i<cfgLeaves;++i)
		{
		volumes[i]=btDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale);
		}
	printf("[1] btDbvt::Volume intersections: ");
	wallclock.reset();
	for(int i=0;i<cfgBenchmark1_Iterations;++i)
		{
		for(int j=0;j<cfgLeaves;++j)
			{
			for(int k=0;k<cfgLeaves;++k)
				{
				results[k]=Intersect(volumes[j],volumes[k]);
				}
			}
		}
	const int time=(int)wallclock.getTimeMilliseconds();
	printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark1_Reference)*100/time);
	}
if(cfgBenchmark2_Enable)
	{// Benchmark 2	
	srand(380843);
	btAlignedObjectArray<btDbvt::Volume>	volumes;
	btAlignedObjectArray<btDbvt::Volume>	results;
	volumes.resize(cfgLeaves);
	results.resize(cfgLeaves);
	for(int i=0;i<cfgLeaves;++i)
		{
		volumes[i]=btDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale);
		}
	printf("[2] btDbvt::Volume merges: ");
	wallclock.reset();
	for(int i=0;i<cfgBenchmark2_Iterations;++i)
		{
		for(int j=0;j<cfgLeaves;++j)
			{
			for(int k=0;k<cfgLeaves;++k)
				{
				Merge(volumes[j],volumes[k],results[k]);
				}
			}
		}
	const int time=(int)wallclock.getTimeMilliseconds();
	printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark2_Reference)*100/time);
	}
if(cfgBenchmark3_Enable)
	{// Benchmark 3	
	srand(380843);
	btDbvt						dbvt[2];
	btDbvtBenchmark::NilPolicy	policy;
	btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt[0]);
	btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt[1]);
	dbvt[0].optimizeTopDown();
	dbvt[1].optimizeTopDown();
	printf("[3] btDbvt::collideTT: ");
	wallclock.reset();
	for(int i=0;i<cfgBenchmark3_Iterations;++i)
		{
		btDbvt::collideTT(dbvt[0].m_root,dbvt[1].m_root,policy);
		}
	const int time=(int)wallclock.getTimeMilliseconds();
	printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark3_Reference)*100/time);
	}
if(cfgBenchmark4_Enable)
	{// Benchmark 4
	srand(380843);
	btDbvt						dbvt;
	btDbvtBenchmark::NilPolicy	policy;
	btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
	dbvt.optimizeTopDown();
	printf("[4] btDbvt::collideTT self: ");
	wallclock.reset();
	for(int i=0;i<cfgBenchmark4_Iterations;++i)
		{
		btDbvt::collideTT(dbvt.m_root,dbvt.m_root,policy);
		}
	const int time=(int)wallclock.getTimeMilliseconds();
	printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark4_Reference)*100/time);
	}
if(cfgBenchmark5_Enable)
	{// Benchmark 5	
	srand(380843);
	btDbvt								dbvt[2];
	btAlignedObjectArray<btTransform>	transforms;
	btDbvtBenchmark::NilPolicy			policy;
	transforms.resize(cfgBenchmark5_Iterations);
	for(int i=0;i<transforms.size();++i)
		{
		transforms[i]=btDbvtBenchmark::RandTransform(cfgVolumeCenterScale*cfgBenchmark5_OffsetScale);
		}
	btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt[0]);
	btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt[1]);
	dbvt[0].optimizeTopDown();
	dbvt[1].optimizeTopDown();
	printf("[5] btDbvt::collideTT xform: ");
	wallclock.reset();
	for(int i=0;i<cfgBenchmark5_Iterations;++i)
		{
		btDbvt::collideTT(dbvt[0].m_root,dbvt[1].m_root,transforms[i],policy);
		}
	const int time=(int)wallclock.getTimeMilliseconds();
	printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark5_Reference)*100/time);
	}
if(cfgBenchmark6_Enable)
	{// Benchmark 6	
	srand(380843);
	btDbvt								dbvt;
	btAlignedObjectArray<btTransform>	transforms;
	btDbvtBenchmark::NilPolicy			policy;
	transforms.resize(cfgBenchmark6_Iterations);
	for(int i=0;i<transforms.size();++i)
		{
		transforms[i]=btDbvtBenchmark::RandTransform(cfgVolumeCenterScale*cfgBenchmark6_OffsetScale);
		}
	btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
	dbvt.optimizeTopDown();
	printf("[6] btDbvt::collideTT xform,self: ");
	wallclock.reset();
	for(int i=0;i<cfgBenchmark6_Iterations;++i)
		{
		btDbvt::collideTT(dbvt.m_root,dbvt.m_root,transforms[i],policy);		
		}
	const int time=(int)wallclock.getTimeMilliseconds();
	printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark6_Reference)*100/time);
	}
if(cfgBenchmark7_Enable)
	{// Benchmark 7	
	srand(380843);
	btDbvt								dbvt;
	btAlignedObjectArray<btVector3>		rayorg;
	btAlignedObjectArray<btVector3>		raydir;
	btDbvtBenchmark::NilPolicy			policy;
	rayorg.resize(cfgBenchmark7_Iterations);
	raydir.resize(cfgBenchmark7_Iterations);
	for(int i=0;i<rayorg.size();++i)
		{
		rayorg[i]=btDbvtBenchmark::RandVector3(cfgVolumeCenterScale*2);
		raydir[i]=btDbvtBenchmark::RandVector3(cfgVolumeCenterScale*2);
		}
	btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
	dbvt.optimizeTopDown();
	printf("[7] btDbvt::collideRAY: ");
	wallclock.reset();
	for(int i=0;i<cfgBenchmark7_Passes;++i)
		{
		for(int j=0;j<cfgBenchmark7_Iterations;++j)
			{
			btDbvt::collideRAY(dbvt.m_root,rayorg[j],raydir[j],policy);
			}
		}
	const int	time=(int)wallclock.getTimeMilliseconds();
	unsigned	rays=cfgBenchmark7_Passes*cfgBenchmark7_Iterations;
	printf("%u ms (%i%%),(%u r/s)\r\n",time,(time-cfgBenchmark7_Reference)*100/time,(rays*1000)/time);
	}
if(cfgBenchmark8_Enable)
	{// Benchmark 8	
	srand(380843);
	btDbvt								dbvt;
	btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
	dbvt.optimizeTopDown();
	printf("[8] insert/remove: ");
	wallclock.reset();
	for(int i=0;i<cfgBenchmark8_Passes;++i)
		{
		for(int j=0;j<cfgBenchmark8_Iterations;++j)
			{
			dbvt.remove(dbvt.insert(btDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale),0));
			}
		}
	const int	time=(int)wallclock.getTimeMilliseconds();
	const int	ir=cfgBenchmark8_Passes*cfgBenchmark8_Iterations;
	printf("%u ms (%i%%),(%u ir/s)\r\n",time,(time-cfgBenchmark8_Reference)*100/time,ir*1000/time);
	}
if(cfgBenchmark9_Enable)
	{// Benchmark 9	
	srand(380843);
	btDbvt										dbvt;
	btAlignedObjectArray<const btDbvt::Node*>	leaves;
	btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
	dbvt.optimizeTopDown();
	dbvt.extractLeaves(dbvt.m_root,leaves);
	printf("[9] updates (teleport): ");
	wallclock.reset();
	for(int i=0;i<cfgBenchmark9_Passes;++i)
		{
		for(int j=0;j<cfgBenchmark9_Iterations;++j)
			{
			dbvt.update(const_cast<btDbvt::Node*>(leaves[rand()%cfgLeaves]),
						btDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale));
			}
		}
	const int	time=(int)wallclock.getTimeMilliseconds();
	const int	up=cfgBenchmark9_Passes*cfgBenchmark9_Iterations;
	printf("%u ms (%i%%),(%u u/s)\r\n",time,(time-cfgBenchmark9_Reference)*100/time,up*1000/time);
	}
if(cfgBenchmark10_Enable)
	{// Benchmark 10	
	srand(380843);
	btDbvt										dbvt;
	btAlignedObjectArray<const btDbvt::Node*>	leaves;
	btAlignedObjectArray<btVector3>				vectors;
	vectors.resize(cfgBenchmark10_Iterations);
	for(int i=0;i<vectors.size();++i)
		{
		vectors[i]=(btDbvtBenchmark::RandVector3()*2-btVector3(1,1,1))*cfgBenchmark10_Scale;
		}
	btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
	dbvt.optimizeTopDown();
	dbvt.extractLeaves(dbvt.m_root,leaves);
	printf("[10] updates (jitter): ");
	wallclock.reset();
	
	for(int i=0;i<cfgBenchmark10_Passes;++i)
		{
		for(int j=0;j<cfgBenchmark10_Iterations;++j)
			{			
			const btVector3&	d=vectors[j];
			btDbvt::Node*		l=const_cast<btDbvt::Node*>(leaves[rand()%cfgLeaves]);
			btDbvt::Volume		v=btDbvt::Volume::FromMM(l->volume.Mins()+d,l->volume.Maxs()+d);
			dbvt.update(l,v);
			}
		}
	const int	time=(int)wallclock.getTimeMilliseconds();
	const int	up=cfgBenchmark10_Passes*cfgBenchmark10_Iterations;
	printf("%u ms (%i%%),(%u u/s)\r\n",time,(time-cfgBenchmark10_Reference)*100/time,up*1000/time);
	}
if(cfgBenchmark11_Enable)
	{// Benchmark 11	
	srand(380843);
	btDbvt										dbvt;
	btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
	dbvt.optimizeTopDown();
	printf("[11] optimize (incremental): ");
	wallclock.reset();	
	for(int i=0;i<cfgBenchmark11_Passes;++i)
		{
		dbvt.optimizeIncremental(cfgBenchmark11_Iterations);
		}
	const int	time=(int)wallclock.getTimeMilliseconds();
	const int	op=cfgBenchmark11_Passes*cfgBenchmark11_Iterations;
	printf("%u ms (%i%%),(%u o/s)\r\n",time,(time-cfgBenchmark11_Reference)*100/time,op/time*1000);
	}
if(cfgBenchmark12_Enable)
	{// Benchmark 12	
	srand(380843);
	btAlignedObjectArray<btDbvt::Volume>	volumes;
	btAlignedObjectArray<bool>				results;
	volumes.resize(cfgLeaves);
	results.resize(cfgLeaves);
	for(int i=0;i<cfgLeaves;++i)
		{
		volumes[i]=btDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale);
		}
	printf("[12] btDbvt::Volume notequal: ");
	wallclock.reset();
	for(int i=0;i<cfgBenchmark12_Iterations;++i)
		{
		for(int j=0;j<cfgLeaves;++j)
			{
			for(int k=0;k<cfgLeaves;++k)
				{
				results[k]=NotEqual(volumes[j],volumes[k]);
				}
			}
		}
	const int time=(int)wallclock.getTimeMilliseconds();
	printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark12_Reference)*100/time);
	}
if(cfgBenchmark13_Enable)
	{// Benchmark 13	
	srand(380843);
	btDbvt								dbvt;
	btAlignedObjectArray<btVector3>		vectors;
	btDbvtBenchmark::NilPolicy			policy;
	vectors.resize(cfgBenchmark13_Iterations);
	for(int i=0;i<vectors.size();++i)
		{
		vectors[i]=(btDbvtBenchmark::RandVector3()*2-btVector3(1,1,1)).normalized();
		}
	btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
	dbvt.optimizeTopDown();
	printf("[13] culling(OCL+fullsort): ");
	wallclock.reset();	
	for(int i=0;i<cfgBenchmark13_Iterations;++i)
		{
		static const btScalar	offset=0;
		policy.m_depth=-SIMD_INFINITY;
		dbvt.collideOCL(dbvt.m_root,&vectors[i],&offset,vectors[i],1,policy);
		}
	const int	time=(int)wallclock.getTimeMilliseconds();
	const int	t=cfgBenchmark13_Iterations;
	printf("%u ms (%i%%),(%u t/s)\r\n",time,(time-cfgBenchmark13_Reference)*100/time,(t*1000)/time);
	}
if(cfgBenchmark14_Enable)
	{// Benchmark 14	
	srand(380843);
	btDbvt								dbvt;
	btAlignedObjectArray<btVector3>		vectors;
	btDbvtBenchmark::P14				policy;
	vectors.resize(cfgBenchmark14_Iterations);
	for(int i=0;i<vectors.size();++i)
		{
		vectors[i]=(btDbvtBenchmark::RandVector3()*2-btVector3(1,1,1)).normalized();
		}
	btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
	dbvt.optimizeTopDown();
	policy.m_nodes.reserve(cfgLeaves);
	printf("[14] culling(OCL+qsort): ");
	wallclock.reset();	
	for(int i=0;i<cfgBenchmark14_Iterations;++i)
		{
		static const btScalar	offset=0;
		policy.m_nodes.resize(0);
		dbvt.collideOCL(dbvt.m_root,&vectors[i],&offset,vectors[i],1,policy,false);
		policy.m_nodes.quickSort(btDbvtBenchmark::P14::sortfnc);
		}
	const int	time=(int)wallclock.getTimeMilliseconds();
	const int	t=cfgBenchmark14_Iterations;
	printf("%u ms (%i%%),(%u t/s)\r\n",time,(time-cfgBenchmark14_Reference)*100/time,(t*1000)/time);
	}
if(cfgBenchmark15_Enable)
	{// Benchmark 15	
	srand(380843);
	btDbvt								dbvt;
	btAlignedObjectArray<btVector3>		vectors;
	btDbvtBenchmark::P15				policy;
	vectors.resize(cfgBenchmark15_Iterations);
	for(int i=0;i<vectors.size();++i)
		{
		vectors[i]=(btDbvtBenchmark::RandVector3()*2-btVector3(1,1,1)).normalized();
		}
	btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
	dbvt.optimizeTopDown();
	policy.m_nodes.reserve(cfgLeaves);
	printf("[15] culling(KDOP+qsort): ");
	wallclock.reset();	
	for(int i=0;i<cfgBenchmark15_Iterations;++i)
		{
		static const btScalar	offset=0;
		policy.m_nodes.resize(0);
		policy.m_axis=vectors[i];
		dbvt.collideKDOP(dbvt.m_root,&vectors[i],&offset,1,policy);
		policy.m_nodes.quickSort(btDbvtBenchmark::P15::sortfnc);
		}
	const int	time=(int)wallclock.getTimeMilliseconds();
	const int	t=cfgBenchmark15_Iterations;
	printf("%u ms (%i%%),(%u t/s)\r\n",time,(time-cfgBenchmark15_Reference)*100/time,(t*1000)/time);
	}
printf("\r\n\r\n");
}
#endif