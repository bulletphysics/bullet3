#include "btDbvt.h"

#include <stdio.h>

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
btDbvt::Aabb	res;
Merge(a,b,res);
return(res);
}


// volume+edge lengths
static inline btScalar			size(const btDbvt::Aabb& a)
{
const btVector3	edges=a.Lengths();
return(	edges.x()*edges.y()*edges.z()+
		edges.x()+edges.y()+edges.z());
}

//
static inline void				deletenode(	btDbvt* pdbvt,
											btDbvt::Node* node)
{
delete pdbvt->m_free;
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
											const btDbvt::Aabb& box,
											void* data)
{
btDbvt::Node*	node;
if(pdbvt->m_free)
	{ node=pdbvt->m_free;pdbvt->m_free=0; }
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
			if(	Proximity(root->childs[0]->box,leaf->box)<
				Proximity(root->childs[1]->box,leaf->box))
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
			if(prev->box.Contain(node->box))
				break;
				else
				Merge(prev->childs[0]->box,prev->childs[1]->box,prev->box);
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
			const btDbvt::Aabb	pb=prev->box;
			Merge(prev->childs[0]->box,prev->childs[1]->box,prev->box);
			if(NotEqual(pb,prev->box))
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
static int						leafcount(btDbvt::Node* root)
{
if(root->isinternal())
	{
	return(	leafcount(root->childs[0])+
			leafcount(root->childs[1]));
	}
return(1);
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
										

}

using namespace btdbvt_internals;

//
// Api
//

//
				btDbvt::btDbvt()
{
m_root	=	0;
m_free	=	0;
m_lkhd	=	2;
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
delete m_free;
m_free=0;
}

//
int				btDbvt::leafCount() const
{
if(m_root)	return(leafcount(m_root));
			else
			return(0);
}

//
void			btDbvt::optimizeBottomUp()
{
if(m_root)
	{
	tNodeArray leafs;
	leafs.reserve(leafCount());
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
	leafs.reserve(leafCount());
	fetchleafs(this,m_root,leafs);
	m_root=topdown(this,leafs,bu_treshold);
	}
}

//
btDbvt::Node*	btDbvt::insert(const Aabb& box,void* data)
{
Node*	leaf=createnode(this,0,box,data);
insertleaf(this,m_root,leaf);
return(leaf);
}

//
void			btDbvt::update(Node* leaf,const Aabb& box)
{
Node*	root=removeleaf(this,leaf);
if(root)
	{
	for(int i=0;(i<m_lkhd)&&root->parent;++i)
		{
		root=root->parent;
		}
	}
leaf->box=box;
insertleaf(this,root,leaf);
}

//
bool			btDbvt::update(Node* leaf,Aabb box,const btVector3& velocity,btScalar margin)
{
if(leaf->box.Contain(box)) return(false);
if(margin>0)
	box.Expand(btVector3(margin,margin,margin));
if(velocity.length2()>0)
	box.SignedExpand(velocity);
update(leaf,box);
return(true);
}

//
void			btDbvt::remove(Node* leaf)
{
removeleaf(this,leaf);
deletenode(this,leaf);
}

//
void			btDbvt::collide(btDbvt* tree,
								ICollide* icollide) const
{
if(tree->m_root&&m_root)
	{
	struct	sStkElm
		{
		const Node*	a;
		const Node*	b;
		sStkElm(const Node* na,const Node* nb) : a(na),b(nb) {}
		};
	btAlignedObjectArray<sStkElm>	stack;
	stack.reserve(128);
	stack.push_back(sStkElm(m_root,tree->m_root));
	do	{
		sStkElm	p=stack[stack.size()-1];
		stack.pop_back();
		if(p.a==p.b)
			{
			if(p.a->isinternal())
				{
				stack.push_back(sStkElm(p.a->childs[0],p.a->childs[0]));
				stack.push_back(sStkElm(p.a->childs[1],p.a->childs[1]));
				stack.push_back(sStkElm(p.a->childs[0],p.a->childs[1]));
				}
			}
		else if(Intersect(p.a->box,p.b->box))
			{
			if(p.a->isinternal())
				{
				if(p.b->isinternal())
					{
					stack.push_back(sStkElm(p.a->childs[0],p.b->childs[0]));
					stack.push_back(sStkElm(p.a->childs[1],p.b->childs[0]));
					stack.push_back(sStkElm(p.a->childs[0],p.b->childs[1]));
					stack.push_back(sStkElm(p.a->childs[1],p.b->childs[1]));
					}
					else
					{
					stack.push_back(sStkElm(p.a->childs[0],p.b));
					stack.push_back(sStkElm(p.a->childs[1],p.b));
					}
				}
				else
				{
				if(p.b->isinternal())
					{
					stack.push_back(sStkElm(p.a,p.b->childs[0]));
					stack.push_back(sStkElm(p.a,p.b->childs[1]));
					}
					else
					{
					icollide->Process(p.a,p.b);
					}
				}
			}
		} while(stack.size()>0);
	}
}
							
//
void			btDbvt::collide(const Aabb& box,
								ICollide* icollide) const
{
if(m_root)
	{
	btAlignedObjectArray<const Node*>	stack;
	stack.reserve(64);
	stack.push_back(m_root);
	do	{
		const Node*	n=stack[stack.size()-1];
		stack.pop_back();
		if(Intersect(n->box,box))
			{
			if(n->isinternal())
				{
				stack.push_back(n->childs[0]);
				stack.push_back(n->childs[1]);
				}
				else
				{
				icollide->Process(n);
				}
			}
		} while(stack.size()>0);
	}
}
							
//
void			btDbvt::collide(const btVector3& org,
								const btVector3& dir,
								ICollide* icollide) const
{
/* not implemented	*/ 
}
