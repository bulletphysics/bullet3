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
				sibling		=	prev;
				prev		=	prev->parent;
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
static void						fetchleafs(	btDbvt* pdbvt,
											btDbvt::Node* root,
											tNodeArray& leafs,
											int depth=-1)
{
if(root->isinternal()&&depth)
	{
	fetchleafs(pdbvt,root->childs[0],leafs,depth-1);
	fetchleafs(pdbvt,root->childs[1],leafs,depth-1);
	deletenode(pdbvt,root);
	}
	else
	{
	leafs.push_back(root);
	}
}

//
static void						split(	const tNodeArray& leafs,
										tNodeArray& left,
										tNodeArray& right,
										const btVector3& org,
										const btVector3& axis)
{
left.resize(0);
right.resize(0);
for(int i=0,ni=leafs.size();i<ni;++i)
	{
	if(dot(axis,leafs[i]->volume.Center()-org)<0)
		left.push_back(leafs[i]);
		else
		right.push_back(leafs[i]);
	}
}

//
static btDbvt::Volume			bounds(	const tNodeArray& leafs)
{
btDbvt::Volume	volume=leafs[0]->volume;
for(int i=1,ni=leafs.size();i<ni;++i)
	{
	volume=merge(volume,leafs[i]->volume);
	}
return(volume);
}

//
static void						bottomup(	btDbvt* pdbvt,
											tNodeArray& leafs)
{
while(leafs.size()>1)
	{
	btScalar	minsize=SIMD_INFINITY;
	int			minidx[2]={-1,-1};
	for(int i=0;i<leafs.size();++i)
		{
		for(int j=i+1;j<leafs.size();++j)
			{
			const btScalar	sz=size(merge(leafs[i]->volume,leafs[j]->volume));
			if(sz<minsize)
				{
				minsize		=	sz;
				minidx[0]	=	i;
				minidx[1]	=	j;
				}
			}
		}
	btDbvt::Node*	n[]	=	{leafs[minidx[0]],leafs[minidx[1]]};
	btDbvt::Node*	p	=	createnode(pdbvt,0,merge(n[0]->volume,n[1]->volume),0);
	p->childs[0]		=	n[0];
	p->childs[1]		=	n[1];
	n[0]->parent		=	p;
	n[1]->parent		=	p;
	leafs[minidx[0]]	=	p;
	leafs.swap(minidx[1],leafs.size()-1);
	leafs.pop_back();
	}
}

//
static btDbvt::Node*			topdown(btDbvt* pdbvt,
										tNodeArray& leafs,
										int bu_treshold)
{
static const btVector3	axis[]={btVector3(1,0,0),
								btVector3(0,1,0),
								btVector3(0,0,1)};
if(leafs.size()>1)
	{
	if(leafs.size()>bu_treshold)
		{
		const btDbvt::Volume	vol=bounds(leafs);
		const btVector3			org=vol.Center();
		tNodeArray				sets[2];
		int						bestaxis=-1;
		int						bestmidp=leafs.size();
		int						splitcount[3][2]={{0,0},{0,0},{0,0}};
		for(int i=0;i<leafs.size();++i)
			{
			const btVector3	x=leafs[i]->volume.Center()-org;
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
			split(leafs,sets[0],sets[1],org,axis[bestaxis]);
			}
			else
			{
			sets[0].reserve(leafs.size()/2+1);
			sets[1].reserve(leafs.size()/2);
			for(int i=0,ni=leafs.size();i<ni;++i)
				{
				sets[i&1].push_back(leafs[i]);
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
		bottomup(pdbvt,leafs);
		return(leafs[0]);
		}
	}
return(leafs[0]);
}

//
static inline btDbvt::Node*		refit(	btDbvt* pdbvt,
										btDbvt::Node* node)
{
btDbvt::Node*	parent=node->parent;
if(parent)
	{
	const int		idx=indexof(node);
	tNodeArray		leafs;
	leafs.reserve(64);
	fetchleafs(pdbvt,node,leafs,3);
	if(leafs.size()>=2)
		{
		bottomup(pdbvt,leafs);
		node=leafs[0];
		node->parent=parent;
		parent->childs[idx]=node;
		}
	}
return(node);
}

//
// Api
//

//
				btDbvt::btDbvt()
{
m_root	=	0;
m_free	=	0;
m_lkhd	=	-1;
m_leafs	=	0;
m_opath	=	0;
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
	tNodeArray leafs;
	leafs.reserve(m_leafs);
	fetchleafs(this,m_root,leafs);
	bottomup(this,leafs);
	m_root=leafs[0];
	}
}

//
void			btDbvt::optimizeTopDown(int bu_treshold)
{
if(m_root)
	{
	tNodeArray	leafs;
	leafs.reserve(m_leafs);
	fetchleafs(this,m_root,leafs);
	m_root=topdown(this,leafs,bu_treshold);
	}
}

//
void			btDbvt::optimizeIncremental(int passes)
{
if(m_root&&(passes>0))
	{
	do	{
		Node*		node=m_root;
		unsigned	bit=0;
		while(node->isinternal())
			{
			node=node->childs[(m_opath>>bit)&1];
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
++m_leafs;
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
--m_leafs;
}

//
void			btDbvt::write(IWriter* iwriter) const
{
btDbvtNodeEnumerator	nodes;
nodes.nodes.reserve(m_leafs*2);
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
