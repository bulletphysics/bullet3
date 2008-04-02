/*
Bounding Volume Hierarchy,  btDbvt.cpp
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
#include "btDbvt.h"

#include <stdio.h>
#include <string.h>

namespace btdbvt_internals
{

//
typedef btAlignedObjectArray<btDbvt::Node*>	tNodeArray;

//
static inline int				indexof(const btDbvt::Node* node)
{
return(node->parent->childs[1]==node);
}

//
static inline btDbvt::Aabb		merge(		const btDbvt::Aabb& a,
											const btDbvt::Aabb& b)
{
btDbvt::Aabb	res=a;
res.mi.setMin(b.mi);
res.mx.setMax(b.mx);
return(res);
}

// volume+edge lengths
static inline btScalar			size(const btDbvt::Aabb& a)
{
const btVector3	edges=a.mx-a.mi;
return(	edges.x()*edges.y()*edges.z()+
		edges.x()+edges.y()+edges.z());
}

// Using manhattan distance heuristic
static inline btScalar			proximity(	const btDbvt::Aabb& a,
											const btDbvt::Aabb& b)
{
const btVector3	d=(a.mi+a.mx)-(b.mi+b.mx);
return(btFabs(d.x())+btFabs(d.y())+btFabs(d.z()));
}

//
static inline bool				contain(	const btDbvt::Aabb& a,
											const btDbvt::Aabb& b)
{
return(	(a.mi.x()<=b.mi.x())&&
		(a.mx.x()>=b.mx.x())&&
		(a.mi.y()<=b.mi.y())&&
		(a.mx.y()>=b.mx.y())&&
		(a.mi.z()<=b.mi.z())&&
		(a.mx.z()>=b.mx.z()));
}
	
//
static inline void				deletenode(	btDbvt* pdbvt,
											btDbvt::Node* node)
{
if(pdbvt->m_stock) delete pdbvt->m_stock;
pdbvt->m_stock=node;
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
											const btDbvt::Aabb& box,
											void* data)
{
btDbvt::Node*	node;
if(pdbvt->m_stock)
	{ node=pdbvt->m_stock;pdbvt->m_stock=0; }
	else
	{ node=new btDbvt::Node(); }
node->parent	=	parent;
node->box		=	box;
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
			if(	proximity(root->childs[0]->box,leaf->box)<
				proximity(root->childs[1]->box,leaf->box))
				root=root->childs[0];
				else
				root=root->childs[1];
			} while(!root->isleaf());
		}
	btDbvt::Node*	prev=root->parent;
	btDbvt::Node*	node=createnode(pdbvt,prev,merge(leaf->box,root->box),0);
	if(prev)
		{
		prev->childs[indexof(root)]	=	node;
		node->childs[0]				=	root;root->parent=node;
		node->childs[1]				=	leaf;leaf->parent=node;
		do	{
			if(contain(prev->box,node->box))
				break;
				else
				prev->box=merge(prev->childs[0]->box,prev->childs[1]->box);
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
++pdbvt->m_nleafs;
}
	
//
static inline btDbvt::Node*		removeleaf(	btDbvt* pdbvt,
											btDbvt::Node* leaf)
{
--pdbvt->m_nleafs;
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
			const btDbvt::Aabb	pb=prev->box;
			prev->box	=	merge(prev->childs[0]->box,prev->childs[1]->box);
			if(0==memcmp(&pb,&prev->box,sizeof(pb))) break;
			sibling		=	prev;
			prev		=	prev->parent;
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
static void						fetchleafs(	btDbvt::Node* root,
											tNodeArray& leafs)
{
if(root->isinternal())
	{
	fetchleafs(root->childs[0],leafs);
	fetchleafs(root->childs[1],leafs);
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
	if(dot(axis,leafs[i]->box.Center()-org)<0)
		left.push_back(leafs[i]);
		else
		right.push_back(leafs[i]);
	}
}

//
static btDbvt::Aabb				bounds(	const tNodeArray& leafs)
{
btDbvt::Aabb	box=leafs[0]->box;
for(int i=1,ni=leafs.size();i<ni;++i)
	{
	box=merge(box,leafs[i]->box);
	}
return(box);
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
			const btScalar	sz=size(merge(leafs[i]->box,leafs[j]->box));
			if(sz<minsize)
				{
				minsize		=	sz;
				minidx[0]	=	i;
				minidx[1]	=	j;
				}
			}
		}
	btDbvt::Node*	n[]	=	{leafs[minidx[0]],leafs[minidx[1]]};
	btDbvt::Node*	p	=	createnode(pdbvt,0,merge(n[0]->box,n[1]->box),0);
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
										tNodeArray& leafs)
{
static const btVector3	axis[]={btVector3(1,0,0),
								btVector3(0,1,0),
								btVector3(0,0,1)};
if(leafs.size()>1)
	{	
	const btDbvt::Aabb	box=bounds(leafs);
	const btVector3		org=box.Center();
	tNodeArray			sets[2];
	int					bestaxis=-1;
	int					bestmidp=leafs.size();
	sets[0].reserve(leafs.size());
	sets[1].reserve(leafs.size());
	for(int i=0;i<3;++i)
		{
		split(leafs,sets[0],sets[1],org,axis[i]);
		if((sets[0].size()>0)&&(sets[1].size()>0))
			{
			const int	midp=abs(sets[0].size()-sets[1].size());
			if(midp<bestmidp)
				{
				bestaxis=i;
				bestmidp=midp;
				}
			}
		}
	if(bestaxis>=0)
		{
		split(leafs,sets[0],sets[1],org,axis[bestaxis]);
		}
		else
		{
		sets[0].resize(0);
		sets[1].resize(0);
		for(int i=0,ni=leafs.size();i<ni;++i)
			{
			sets[i&1].push_back(leafs[i]);
			}
		}
	btDbvt::Node*	node=createnode(pdbvt,0,box,0);
	node->childs[0]=topdown(pdbvt,sets[0]);
	node->childs[1]=topdown(pdbvt,sets[1]);
	node->childs[0]->parent=node;
	node->childs[1]->parent=node;
	return(node);
	}
return(leafs[0]);
}
									
										

}

using namespace btdbvt_internals;

//
// Api
//

//
btDbvt*			btDbvt::Create()
{
btDbvt*	pdbvt	=	new btDbvt();
pdbvt->m_nleafs	=	0;
pdbvt->m_root	=	0;
pdbvt->m_stock	=	0;
return(pdbvt);
}

//
void			btDbvt::Delete()
{
if(m_root)	recursedeletenode(this,m_root);
if(m_stock)	delete m_stock;
delete this;
}

//
void			btDbvt::OptimizeBottomUp()
{
if(m_root)
	{
	tNodeArray leafs;
	leafs.reserve(m_nleafs);
	fetchleafs(m_root,leafs);
	for(int i=0,ni=leafs.size();i<ni;++i)
		{
		removeleaf(this,leafs[i]);
		}
	m_nleafs	=	leafs.size();
	bottomup(this,leafs);
	m_root		=	leafs[0];
	}
}

//
void			btDbvt::OptimizeTopDown()
{
if(m_root)
	{
	tNodeArray	leafs;
	leafs.reserve(m_nleafs);
	fetchleafs(m_root,leafs);
	for(int i=0,ni=leafs.size();i<ni;++i)
		{
		removeleaf(this,leafs[i]);
		}
	m_nleafs	=	leafs.size();
	m_root		=	topdown(this,leafs);
	}
}

//
btDbvt::Node*	btDbvt::Insert(	const btVector3& center,
								const btVector3& extent,
								void* data)
{
Node*	leaf=createnode(this,0,Aabb::FromCE(center,extent),data);
insertleaf(this,m_root,leaf);
return(leaf);
}

//
bool			btDbvt::Update(	Node* leaf,
								const btVector3& center,
								const btVector3& extent,
								btScalar margin)
{
Aabb	box=Aabb::FromCE(center,extent);
if(margin>0)
	{
	if(contain(leaf->box,box)) return(false);
	const btVector3	vm(margin,margin,margin);
	box.mi-=vm;box.mx+=vm;
	}
Node*	root=removeleaf(this,leaf);
leaf->box=box;
insertleaf(this,root,leaf);
return(true);
}

//
void			btDbvt::Remove(	Node* leaf)
{
removeleaf(this,leaf);
deletenode(this,leaf);
}