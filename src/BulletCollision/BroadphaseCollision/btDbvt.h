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
///btDbvt implementation by Nathanael Presson

#ifndef BT_DYNAMIC_BOUNDING_VOLUME_TREE_H
#define BT_DYNAMIC_BOUNDING_VOLUME_TREE_H

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"

//
// Compile time configuration
//

#ifdef WIN32
#define	DBVT_USE_TEMPLATE		1	// Enable template for ICollide
#else
#define	DBVT_USE_TEMPLATE		0	// Enable template for ICollide
#endif

#define DBVT_USE_MEMMOVE		1	// Enable memmove (collideOCL)
#define	DBVT_ENABLE_BENCHMARK	0	// Enable benchmarking code

//
// Auto config and checks
//

#if DBVT_USE_TEMPLATE
#define	DBVT_VIRTUAL
#define DBVT_VIRTUAL_DTOR(a)
#define DBVT_PREFIX					template <typename T>
#define DBVT_IPOLICY				T& policy
#define DBVT_CHECKTYPE				static const ICollide&	typechecker=*(T*)0;
#else
#define	DBVT_VIRTUAL_DTOR(a)		virtual ~a() {}
#define DBVT_VIRTUAL				virtual
#define DBVT_PREFIX
#define DBVT_IPOLICY				ICollide& policy
#define DBVT_CHECKTYPE
#endif

#if DBVT_USE_MEMMOVE
#include <memory.h>
#include <string.h>
#endif

#ifndef DBVT_USE_TEMPLATE
#error "DBVT_USE_TEMPLATE undefined"
#endif

#ifndef DBVT_USE_MEMMOVE
#error "DBVT_USE_MEMMOVE undefined"
#endif

#ifndef DBVT_ENABLE_BENCHMARK
#error "DBVT_ENABLE_BENCHMARK undefined"
#endif

//
// Defaults volumes
//

/* btDbvtAabbMm			*/ 
struct	btDbvtAabbMm
{
inline btVector3			Center() const	{ return((mi+mx)/2); }
inline btVector3			Lengths() const	{ return(mx-mi); }
inline btVector3			Extents() const	{ return((mx-mi)/2); }
inline const btVector3&		Mins() const	{ return(mi); }
inline const btVector3&		Maxs() const	{ return(mx); }
static inline btDbvtAabbMm	FromCE(const btVector3& c,const btVector3& e);
static inline btDbvtAabbMm	FromCR(const btVector3& c,btScalar r);
static inline btDbvtAabbMm	FromMM(const btVector3& mi,const btVector3& mx);
static inline btDbvtAabbMm	FromPoints(const btVector3* pts,int n);
static inline btDbvtAabbMm	FromPoints(const btVector3** ppts,int n);
inline void					Expand(const btVector3 e);
inline void					SignedExpand(const btVector3 e);
inline bool					Contain(const btDbvtAabbMm& a) const;
inline int					Classify(const btVector3& n,btScalar o,int s) const;
inline btScalar				ProjectMinimum(const btVector3& v,unsigned signs) const;
inline friend bool			Intersect(	const btDbvtAabbMm& a,
										const btDbvtAabbMm& b);
inline friend bool			Intersect(	const btDbvtAabbMm& a,
										const btDbvtAabbMm& b,
										const btTransform& xform);
inline friend bool			Intersect(	const btDbvtAabbMm& a,
										const btVector3& b);
inline friend bool			Intersect(	const btDbvtAabbMm& a,
										const btVector3& org,
										const btVector3& invdir,
										const unsigned* signs);
inline friend btScalar		Proximity(	const btDbvtAabbMm& a,
										const btDbvtAabbMm& b);
inline friend void			Merge(	const btDbvtAabbMm& a,
									const btDbvtAabbMm& b,
									btDbvtAabbMm& r);
inline friend bool			NotEqual(	const btDbvtAabbMm& a,
										const btDbvtAabbMm& b);
private:
inline void					AddSpan(const btVector3& d,btScalar& smi,btScalar& smx) const;
private:
btVector3	mi,mx;
};

//
// Dynamic bounding volume tree
//
struct	btDbvt
	{
	// Types	
	typedef	btDbvtAabbMm	Volume;
	/* Node				*/ 
	struct	Node
		{
		Volume	volume;
		Node*	parent;
		bool	isleaf() const		{ return(childs[1]==0); }
		bool	isinternal() const	{ return(!isleaf()); }
		union	{
				Node*	childs[2];
				void*	data;
				};
		};
	/* Stack element	*/ 
	struct	sStkNN
		{
		const Node*	a;
		const Node*	b;
		sStkNN(const Node* na,const Node* nb) : a(na),b(nb) {}
		};
	struct	sStkNP
		{
		const Node*	node;
		int			mask;
		sStkNP(const Node* n,unsigned m) : node(n),mask(m) {}
		};
	struct	sStkNPS
		{
		const Node*	node;
		int			mask;
		btScalar	value;
		sStkNPS() {}
		sStkNPS(const Node* n,unsigned m,btScalar v) : node(n),mask(m),value(v) {}
		};
	struct	sStkCLN
		{
		const Node*	node;
		Node*		parent;
		sStkCLN(const Node* n,Node* p) : node(n),parent(p) {}
		};
	// Policies/Interfaces
			
	/* ICollide	*/ 
	struct	ICollide
		{		
		DBVT_VIRTUAL_DTOR(ICollide)
		DBVT_VIRTUAL void	Process(const Node*,const Node*)		{}
		DBVT_VIRTUAL void	Process(const Node*)					{}
		DBVT_VIRTUAL void	Process(const Node* n,btScalar)			{ Process(n); }
		DBVT_VIRTUAL bool	Descent(const Node*)					{ return(true); }
		DBVT_VIRTUAL bool	AllLeaves(const Node*)					{ return(true); }
		};
	/* IWriter	*/ 
	struct	IWriter
		{
		virtual ~IWriter() {}
		virtual void		Prepare(const Node* root,int numnodes)=0;
		virtual void		WriteNode(const Node*,int index,int parent,int child0,int child1)=0;
		virtual void		WriteLeaf(const Node*,int index,int parent)=0;
		};
	/* IClone	*/ 
	struct	IClone
		{
		virtual ~IClone()	{}
		virtual void		CloneLeaf(Node*) {}
		};
		
	// Constants
	enum	{
			SIMPLE_STACKSIZE	=	64,
			DOUBLE_STACKSIZE	=	SIMPLE_STACKSIZE*2
			};
		
	// Fields
	Node*			m_root;
	Node*			m_free;
	int				m_lkhd;
	int				m_leaves;
	unsigned		m_opath;
	// Methods
					btDbvt();
					~btDbvt();
	void			clear();
	bool			empty() const { return(0==m_root); }
	void			optimizeBottomUp();
	void			optimizeTopDown(int bu_treshold=128);
	void			optimizeIncremental(int passes);
	Node*			insert(const Volume& box,void* data);
	void			update(Node* leaf,int lookahead=-1);
	void			update(Node* leaf,const Volume& volume);
	bool			update(Node* leaf,Volume volume,const btVector3& velocity,btScalar margin);
	bool			update(Node* leaf,Volume volume,const btVector3& velocity);
	bool			update(Node* leaf,Volume volume,btScalar margin);	
	void			remove(Node* leaf);
	void			write(IWriter* iwriter) const;
	void			clone(btDbvt& dest,IClone* iclone=0) const;	
	static int		countLeaves(const Node* node);
	static void		extractLeaves(const Node* node,btAlignedObjectArray<const Node*>& leaves);
	#if DBVT_ENABLE_BENCHMARK
	static void		benchmark();
	#else
	static void		benchmark(){}
	#endif
	// DBVT_IPOLICY must support ICollide policy/interface
	DBVT_PREFIX
	static void		enumNodes(	const Node* root,
								DBVT_IPOLICY);
	DBVT_PREFIX
	static void		enumLeaves(	const Node* root,
								DBVT_IPOLICY);
	DBVT_PREFIX
	static void		collideTT(	const Node* root0,
								const Node* root1,
								DBVT_IPOLICY);
	DBVT_PREFIX
	static void		collideTT(	const Node* root0,
								const Node* root1,
								const btTransform& xform,
								DBVT_IPOLICY);
	DBVT_PREFIX
	static void		collideTT(	const Node* root0,
								const btTransform& xform0,
								const Node* root1,
								const btTransform& xform1,
								DBVT_IPOLICY);
	DBVT_PREFIX
	static void		collideTV(	const Node* root,
								const Volume& volume,
								DBVT_IPOLICY);
	DBVT_PREFIX
	static void		collideRAY(	const Node* root,
								const btVector3& origin,
								const btVector3& direction,
								DBVT_IPOLICY);
	DBVT_PREFIX
	static void		collideKDOP(const Node* root,
								const btVector3* normals,
								const btScalar* offsets,
								int count,
								DBVT_IPOLICY);
	DBVT_PREFIX
	static void		collideOCL(	const Node* root,
								const btVector3* normals,
								const btScalar* offsets,
								const btVector3& sortaxis,
								int count,								
								DBVT_IPOLICY,
								bool fullsort=true);
	DBVT_PREFIX
	static void		collideTU(	const Node* root,
								DBVT_IPOLICY);
	// Helpers	
	static inline int		nearest(const int* i,const btDbvt::sStkNPS* a,btScalar v,int l,int h)
		{
		int	m=0;
		while(l<h)
			{
			m=(l+h)>>1;
			if(a[i[m]].value>=v) l=m+1; else h=m;
			}
		return(h);
		}
	static inline int		allocate(	btAlignedObjectArray<int>& ifree,
										btAlignedObjectArray<btDbvt::sStkNPS>& stock,
										const btDbvt::sStkNPS& value)
		{
		int	i;
		if(ifree.size()>0)
			{ i=ifree[ifree.size()-1];ifree.pop_back();stock[i]=value; }
			else
			{ i=stock.size();stock.push_back(value); }
		return(i); 
		}
	//
	private:
					btDbvt(const btDbvt&)	{}	
	};

//
// Inline's
//

//
inline btDbvtAabbMm		btDbvtAabbMm::FromCE(const btVector3& c,const btVector3& e)
{
btDbvtAabbMm box;
box.mi=c-e;box.mx=c+e;
return(box);
}
	
//
inline btDbvtAabbMm		btDbvtAabbMm::FromCR(const btVector3& c,btScalar r)
{
return(FromCE(c,btVector3(r,r,r)));
}
	
//
inline btDbvtAabbMm		btDbvtAabbMm::FromMM(const btVector3& mi,const btVector3& mx)
{
btDbvtAabbMm box;
box.mi=mi;box.mx=mx;
return(box);
}
	
//
inline btDbvtAabbMm		btDbvtAabbMm::FromPoints(const btVector3* pts,int n)
{
btDbvtAabbMm box;
box.mi=box.mx=pts[0];
for(int i=1;i<n;++i)
	{
	box.mi.setMin(pts[i]);
	box.mx.setMax(pts[i]);
	}
return(box);
}

//
inline btDbvtAabbMm		btDbvtAabbMm::FromPoints(const btVector3** ppts,int n)
{
btDbvtAabbMm box;
box.mi=box.mx=*ppts[0];
for(int i=1;i<n;++i)
	{
	box.mi.setMin(*ppts[i]);
	box.mx.setMax(*ppts[i]);
	}
return(box);
}

//
inline void				btDbvtAabbMm::Expand(const btVector3 e)
{
mi-=e;mx+=e;
}
	
//
inline void				btDbvtAabbMm::SignedExpand(const btVector3 e)
{
if(e.x()>0) mx.setX(mx.x()+e.x()); else mi.setX(mi.x()+e.x());
if(e.y()>0) mx.setY(mx.y()+e.y()); else mi.setY(mi.y()+e.y());
if(e.z()>0) mx.setZ(mx.z()+e.z()); else mi.setZ(mi.z()+e.z());
}
	
//
inline bool				btDbvtAabbMm::Contain(const btDbvtAabbMm& a) const
{
return(	(mi.x()<=a.mi.x())&&
		(mi.y()<=a.mi.y())&&
		(mi.z()<=a.mi.z())&&
		(mx.x()>=a.mx.x())&&
		(mx.y()>=a.mx.y())&&
		(mx.z()>=a.mx.z()));
}

//
inline int				btDbvtAabbMm::Classify(const btVector3& n,btScalar o,int s) const
{
btVector3			pi,px;
switch(s)
	{
	case	(0+0+0):	px=btVector3(mi.x(),mi.y(),mi.z());
						pi=btVector3(mx.x(),mx.y(),mx.z());break;
	case	(1+0+0):	px=btVector3(mx.x(),mi.y(),mi.z());
						pi=btVector3(mi.x(),mx.y(),mx.z());break;
	case	(0+2+0):	px=btVector3(mi.x(),mx.y(),mi.z());
						pi=btVector3(mx.x(),mi.y(),mx.z());break;
	case	(1+2+0):	px=btVector3(mx.x(),mx.y(),mi.z());
						pi=btVector3(mi.x(),mi.y(),mx.z());break;
	case	(0+0+4):	px=btVector3(mi.x(),mi.y(),mx.z());
						pi=btVector3(mx.x(),mx.y(),mi.z());break;
	case	(1+0+4):	px=btVector3(mx.x(),mi.y(),mx.z());
						pi=btVector3(mi.x(),mx.y(),mi.z());break;
	case	(0+2+4):	px=btVector3(mi.x(),mx.y(),mx.z());
						pi=btVector3(mx.x(),mi.y(),mi.z());break;
	case	(1+2+4):	px=btVector3(mx.x(),mx.y(),mx.z());
						pi=btVector3(mi.x(),mi.y(),mi.z());break;
	}
if((dot(n,px)+o)<0)		return(-1);
if((dot(n,pi)+o)>=0)	return(+1);
return(0);
}

//
inline btScalar			btDbvtAabbMm::ProjectMinimum(const btVector3& v,unsigned signs) const
{
const btVector3*	b[]={&mx,&mi};
const btVector3		p(	b[(signs>>0)&1]->x(),
						b[(signs>>1)&1]->y(),
						b[(signs>>2)&1]->z());
return(dot(p,v));
}

//
inline void				btDbvtAabbMm::AddSpan(const btVector3& d,btScalar& smi,btScalar& smx) const
{
for(int i=0;i<3;++i)
	{
	if(d[i]<0)
		{ smi+=mx[i]*d[i];smx+=mi[i]*d[i]; }
		else
		{ smi+=mi[i]*d[i];smx+=mx[i]*d[i]; }
	}
}
	
//
inline bool				Intersect(	const btDbvtAabbMm& a,
									const btDbvtAabbMm& b)
{
return(	(a.mi.x()<=b.mx.x())&&
		(a.mx.x()>=b.mi.x())&&
		(a.mi.y()<=b.mx.y())&&
		(a.mx.y()>=b.mi.y())&&
		(a.mi.z()<=b.mx.z())&&		
		(a.mx.z()>=b.mi.z()));
}

//
inline bool				Intersect(	const btDbvtAabbMm& a,
									const btDbvtAabbMm& b,
									const btTransform& xform)
{
const btVector3		d0=xform*b.Center()-a.Center();
const btVector3		d1=d0*xform.getBasis();
btScalar			s0[2]={0,0};
btScalar			s1[2]={dot(xform.getOrigin(),d0),s1[0]};
a.AddSpan(d0,s0[0],s0[1]);
b.AddSpan(d1,s1[0],s1[1]);
if(s0[0]>(s1[1])) return(false);
if(s0[1]<(s1[0])) return(false);
return(true);
}

//
inline bool				Intersect(	const btDbvtAabbMm& a,
									const btVector3& b)
{
return(	(b.x()>=a.mi.x())&&
		(b.y()>=a.mi.y())&&
		(b.z()>=a.mi.z())&&
		(b.x()<=a.mx.x())&&
		(b.y()<=a.mx.y())&&
		(b.z()<=a.mx.z()));
}

//
inline bool				Intersect(	const btDbvtAabbMm& a,
									const btVector3& org,
									const btVector3& invdir,
									const unsigned* signs)
{
const btVector3*	bounds[2]={&a.mi,&a.mx};
btScalar			txmin=(bounds[  signs[0]]->x()-org[0])*invdir[0];
btScalar			txmax=(bounds[1-signs[0]]->x()-org[0])*invdir[0];
const btScalar		tymin=(bounds[  signs[1]]->y()-org[1])*invdir[1];
const btScalar		tymax=(bounds[1-signs[1]]->y()-org[1])*invdir[1];
if((txmin>tymax)||(tymin>txmax)) return(false);
if(tymin>txmin) txmin=tymin;
if(tymax<txmax) txmax=tymax;
const btScalar		tzmin=(bounds[  signs[2]]->z()-org[2])*invdir[2];
const btScalar		tzmax=(bounds[1-signs[2]]->z()-org[2])*invdir[2];
if((txmin>tzmax)||(tzmin>txmax)) return(false);
if(tzmin>txmin) txmin=tzmin;
if(tzmax<txmax) txmax=tzmax;
return(txmax>0);
}
	
//
inline btScalar			Proximity(	const btDbvtAabbMm& a,
									const btDbvtAabbMm& b)
{
const btVector3	d=(a.mi+a.mx)-(b.mi+b.mx);
return(btFabs(d.x())+btFabs(d.y())+btFabs(d.z()));
}

//
inline void				Merge(	const btDbvtAabbMm& a,
								const btDbvtAabbMm& b,
								btDbvtAabbMm& r)
{
for(int i=0;i<3;++i)
	{
	if(a.mi[i]<b.mi[i]) r.mi[i]=a.mi[i]; else r.mi[i]=b.mi[i];
	if(a.mx[i]>b.mx[i]) r.mx[i]=a.mx[i]; else r.mx[i]=b.mx[i];
	}
}

//
inline bool				NotEqual(	const btDbvtAabbMm& a,
									const btDbvtAabbMm& b)
{
return(	(a.mi.x()!=b.mi.x())||
		(a.mi.y()!=b.mi.y())||
		(a.mi.z()!=b.mi.z())||
		(a.mx.x()!=b.mx.x())||
		(a.mx.y()!=b.mx.y())||
		(a.mx.z()!=b.mx.z()));
}

//
// Inline's
//

//
DBVT_PREFIX
inline void		btDbvt::enumNodes(	const Node* root,
									DBVT_IPOLICY)
{
DBVT_CHECKTYPE
policy.Process(root);
if(root->isinternal())
	{
	enumNodes(root->childs[0],policy);
	enumNodes(root->childs[1],policy);
	}
}

//
DBVT_PREFIX
inline void		btDbvt::enumLeaves(	const Node* root,
									DBVT_IPOLICY)
{
DBVT_CHECKTYPE
if(root->isinternal())
	{
	enumLeaves(root->childs[0],policy);
	enumLeaves(root->childs[1],policy);
	}
	else
	{
	policy.Process(root);
	}
}

//
DBVT_PREFIX
inline void		btDbvt::collideTT(	const Node* root0,
									const Node* root1,
									DBVT_IPOLICY)
{
DBVT_CHECKTYPE
if(root0&&root1)
	{
	btAlignedObjectArray<sStkNN>	stack;
	stack.reserve(DOUBLE_STACKSIZE);
	stack.push_back(sStkNN(root0,root1));
	do	{
		sStkNN	p=stack[stack.size()-1];
		stack.pop_back();
		if(p.a==p.b)
			{
			if(p.a->isinternal())
				{
				stack.push_back(sStkNN(p.a->childs[0],p.a->childs[0]));
				stack.push_back(sStkNN(p.a->childs[1],p.a->childs[1]));
				stack.push_back(sStkNN(p.a->childs[0],p.a->childs[1]));
				}
			}
		else if(Intersect(p.a->volume,p.b->volume))
			{
			if(p.a->isinternal())
				{
				if(p.b->isinternal())
					{
					stack.push_back(sStkNN(p.a->childs[0],p.b->childs[0]));
					stack.push_back(sStkNN(p.a->childs[1],p.b->childs[0]));
					stack.push_back(sStkNN(p.a->childs[0],p.b->childs[1]));
					stack.push_back(sStkNN(p.a->childs[1],p.b->childs[1]));
					}
					else
					{
					stack.push_back(sStkNN(p.a->childs[0],p.b));
					stack.push_back(sStkNN(p.a->childs[1],p.b));
					}
				}
				else
				{
				if(p.b->isinternal())
					{
					stack.push_back(sStkNN(p.a,p.b->childs[0]));
					stack.push_back(sStkNN(p.a,p.b->childs[1]));
					}
					else
					{
					policy.Process(p.a,p.b);
					}
				}
			}
		} while(stack.size()>0);
	}
}

//
DBVT_PREFIX
inline void		btDbvt::collideTT(	const Node* root0,
									const Node* root1,
									const btTransform& xform,
									DBVT_IPOLICY)
{
DBVT_CHECKTYPE
if(root0&&root1)
	{
	btAlignedObjectArray<sStkNN>	stack;
	stack.reserve(DOUBLE_STACKSIZE);
	stack.push_back(sStkNN(root0,root1));
	do	{
		sStkNN	p=stack[stack.size()-1];
		stack.pop_back();
		if(Intersect(p.a->volume,p.b->volume,xform))
			{
			if(p.a->isinternal())
				{
				if(p.b->isinternal())
					{
					stack.push_back(sStkNN(p.a->childs[0],p.b->childs[0]));
					stack.push_back(sStkNN(p.a->childs[1],p.b->childs[0]));
					stack.push_back(sStkNN(p.a->childs[0],p.b->childs[1]));
					stack.push_back(sStkNN(p.a->childs[1],p.b->childs[1]));
					}
					else
					{
					stack.push_back(sStkNN(p.a->childs[0],p.b));
					stack.push_back(sStkNN(p.a->childs[1],p.b));
					}
				}
				else
				{
				if(p.b->isinternal())
					{
					stack.push_back(sStkNN(p.a,p.b->childs[0]));
					stack.push_back(sStkNN(p.a,p.b->childs[1]));
					}
					else
					{
					policy.Process(p.a,p.b);
					}
				}
			}
		} while(stack.size()>0);
	}
}

//
DBVT_PREFIX
inline void		btDbvt::collideTT(	const Node* root0,
									const btTransform& xform0,
									const Node* root1,
									const btTransform& xform1,
									DBVT_IPOLICY)
{
const btTransform	xform=xform0.inverse()*xform1;
collideTT(root0,root1,xform,policy);
}

//
DBVT_PREFIX
inline void		btDbvt::collideTV(	const Node* root,
									const Volume& volume,
									DBVT_IPOLICY)
{
DBVT_CHECKTYPE
if(root)
	{
	btAlignedObjectArray<const Node*>	stack;
	stack.reserve(SIMPLE_STACKSIZE);
	stack.push_back(root);
	do	{
		const Node*	n=stack[stack.size()-1];
		stack.pop_back();
		if(Intersect(n->volume,volume))
			{
			if(n->isinternal())
				{
				stack.push_back(n->childs[0]);
				stack.push_back(n->childs[1]);
				}
				else
				{
				policy.Process(n);
				}
			}
		} while(stack.size()>0);
	}
}

//
DBVT_PREFIX
inline void		btDbvt::collideRAY(	const Node* root,
									const btVector3& origin,
									const btVector3& direction,
									DBVT_IPOLICY)
{
DBVT_CHECKTYPE
if(root)
	{
	const btVector3	normal=direction.normalized();
	const btVector3	invdir(	1/normal.x(),
							1/normal.y(),
							1/normal.z());
	const unsigned	signs[]={	direction.x()<0,
								direction.y()<0,
								direction.z()<0};
	btAlignedObjectArray<const Node*>	stack;
	stack.reserve(SIMPLE_STACKSIZE);
	stack.push_back(root);
	do	{
		const Node*	node=stack[stack.size()-1];
		stack.pop_back();
		if(Intersect(node->volume,origin,invdir,signs))
			{
			if(node->isinternal())
				{
				stack.push_back(node->childs[0]);
				stack.push_back(node->childs[1]);
				}
				else
				{
				policy.Process(node);
				}
			}
		} while(stack.size());
	}
}

//
DBVT_PREFIX
inline void		btDbvt::collideKDOP(const Node* root,
									const btVector3* normals,
									const btScalar* offsets,
									int count,
									DBVT_IPOLICY)
{
DBVT_CHECKTYPE
if(root)
	{
	const int						inside=(1<<count)-1;
	btAlignedObjectArray<sStkNP>	stack;
	int								signs[sizeof(unsigned)*8];
	btAssert(count<int (sizeof(signs)/sizeof(signs[0])));
	for(int i=0;i<count;++i)
		{
		signs[i]=	((normals[i].x()>=0)?1:0)+
					((normals[i].y()>=0)?2:0)+
					((normals[i].z()>=0)?4:0);
		}
	stack.reserve(SIMPLE_STACKSIZE);
	stack.push_back(sStkNP(root,0));
	do	{
		sStkNP	se=stack[stack.size()-1];
		bool	out=false;
		stack.pop_back();
		for(int i=0,j=1;(!out)&&(i<count);++i,j<<=1)
			{
			if(0==(se.mask&j))
				{
				const int	side=se.node->volume.Classify(normals[i],offsets[i],signs[i]);
				switch(side)
					{
					case	-1:	out=true;break;
					case	+1:	se.mask|=j;break;
					}
				}
			}
		if(!out)
			{
			if((se.mask!=inside)&&(se.node->isinternal()))
				{
				stack.push_back(sStkNP(se.node->childs[0],se.mask));
				stack.push_back(sStkNP(se.node->childs[1],se.mask));
				}
				else
				{
				if(policy.AllLeaves(se.node)) enumLeaves(se.node,policy);
				}
			}
		} while(stack.size());
	}
}

//
DBVT_PREFIX
inline void		btDbvt::collideOCL(	const Node* root,
									const btVector3* normals,
									const btScalar* offsets,
									const btVector3& sortaxis,
									int count,
									DBVT_IPOLICY,
									bool fsort)
{
DBVT_CHECKTYPE
if(root)
	{
	const unsigned					srtsgns=(sortaxis[0]>=0?1:0)+
											(sortaxis[1]>=0?2:0)+
											(sortaxis[2]>=0?4:0);
	const int						inside=(1<<count)-1;
	btAlignedObjectArray<sStkNPS>	stock;
	btAlignedObjectArray<int>		ifree;
	btAlignedObjectArray<int>		stack;
	int								signs[sizeof(unsigned)*8];
	btAssert(count<int (sizeof(signs)/sizeof(signs[0])));
	for(int i=0;i<count;++i)
		{
		signs[i]=	((normals[i].x()>=0)?1:0)+
					((normals[i].y()>=0)?2:0)+
					((normals[i].z()>=0)?4:0);
		}
	stock.reserve(SIMPLE_STACKSIZE);
	stack.reserve(SIMPLE_STACKSIZE);
	ifree.reserve(SIMPLE_STACKSIZE);
	stack.push_back(allocate(ifree,stock,sStkNPS(root,0,root->volume.ProjectMinimum(sortaxis,srtsgns))));
	do	{
		const int	id=stack[stack.size()-1];
		sStkNPS		se=stock[id];
		stack.pop_back();ifree.push_back(id);
		if(se.mask!=inside)
			{
			bool	out=false;
			for(int i=0,j=1;(!out)&&(i<count);++i,j<<=1)
				{
				if(0==(se.mask&j))
					{
					const int	side=se.node->volume.Classify(normals[i],offsets[i],signs[i]);
					switch(side)
						{
						case	-1:	out=true;break;
						case	+1:	se.mask|=j;break;
						}
					}
				}
			if(out) continue;
			}
		if(policy.Descent(se.node))
			{
			if(se.node->isinternal())
				{
				const Node* pns[]={	se.node->childs[0],se.node->childs[1]};
				sStkNPS		nes[]={	sStkNPS(pns[0],se.mask,pns[0]->volume.ProjectMinimum(sortaxis,srtsgns)),
									sStkNPS(pns[1],se.mask,pns[1]->volume.ProjectMinimum(sortaxis,srtsgns))};
				const int	q=nes[0].value<nes[1].value?1:0;				
				int			j=stack.size();
				if(fsort&&(j>0))
					{
					/* Insert 0	*/ 
					j=nearest(&stack[0],&stock[0],nes[q].value,0,stack.size());
					stack.push_back(0);
					#if DBVT_USE_MEMMOVE
					memmove(&stack[j+1],&stack[j],sizeof(int)*(stack.size()-j-1));
					#else
					for(int k=stack.size()-1;k>j;--k) stack[k]=stack[k-1];
					#endif
					stack[j]=allocate(ifree,stock,nes[q]);
					/* Insert 1	*/ 
					j=nearest(&stack[0],&stock[0],nes[1-q].value,j,stack.size());
					stack.push_back(0);
					#if DBVT_USE_MEMMOVE
					memmove(&stack[j+1],&stack[j],sizeof(int)*(stack.size()-j-1));
					#else
					for(int k=stack.size()-1;k>j;--k) stack[k]=stack[k-1];
					#endif
					stack[j]=allocate(ifree,stock,nes[1-q]);
					}
					else
					{
					stack.push_back(allocate(ifree,stock,nes[q]));
					stack.push_back(allocate(ifree,stock,nes[1-q]));
					}
				}
				else
				{
				policy.Process(se.node,se.value);
				}
			}
		} while(stack.size());
	}
}

//
DBVT_PREFIX
inline void		btDbvt::collideTU(	const Node* root,
									DBVT_IPOLICY)
{
DBVT_CHECKTYPE
if(root)
	{
	btAlignedObjectArray<const Node*>	stack;
	stack.reserve(SIMPLE_STACKSIZE);
	stack.push_back(root);
	do	{
		const Node*	n=stack[stack.size()-1];
		stack.pop_back();
		if(policy.Descent(n))
			{
			if(n->isinternal())
				{ stack.push_back(n->childs[0]);stack.push_back(n->childs[1]); }
				else
				{ policy.Process(n); }
			}
		} while(stack.size()>0);
	}
}

//
// PP Cleanup
//

#undef DBVT_USE_MEMMOVE
#undef DBVT_USE_TEMPLATE
#undef DBVT_VIRTUAL_DTOR
#undef DBVT_VIRTUAL
#undef DBVT_PREFIX
#undef DBVT_IPOLICY
#undef DBVT_CHECKTYPE

#endif
