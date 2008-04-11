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
///btSoftBody implementation by Nathanael Presson

#ifndef BT_DYNAMIC_BOUNDING_VOLUME_TREE_H
#define BT_DYNAMIC_BOUNDING_VOLUME_TREE_H

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btVector3.h"

//
// Dynamic bounding volume tree
//
struct	btDbvt
	{
	// Types

	/* Aabb		*/ 
	struct	Aabb
		{
		inline btVector3		Center() const	{ return((mi+mx)/2); }
		inline btVector3		Extent() const	{ return((mx-mi)/2); }
		inline const btVector3&	Mins() const	{ return(mi); }
		inline const btVector3&	Maxs() const	{ return(mx); }
		inline btVector3		Lengths() const	{ return(mx-mi); }
		static inline Aabb		FromCE(const btVector3& c,const btVector3& e);
		static inline Aabb		FromCR(const btVector3& c,btScalar r);
		static inline Aabb		FromMM(const btVector3& mi,const btVector3& mx);
		static inline Aabb		FromPoints(const btVector3* pts,int n);
		static inline Aabb		FromPoints(const btVector3** ppts,int n);
		inline void				Expand(const btVector3 e);
		inline void				SignedExpand(const btVector3 e);
		inline bool				Contain(const Aabb& a) const;
		inline friend bool		Intersect(	const Aabb& a,
											const Aabb& b);
		inline friend btScalar	Proximity(	const Aabb& a,
											const Aabb& b);
		inline friend void		Merge(	const Aabb& a,
										const Aabb& b,
										Aabb& r);
		inline friend bool		NotEqual(	const Aabb& a,
											const Aabb& b);
		btVector3	mi,mx;
		};
	/* Node			*/ 
	struct	Node
		{
		Aabb	box;
		Node*	parent;
		bool	isleaf() const		{ return(childs[1]==0); }
		bool	isinternal() const	{ return(!isleaf()); }
		union	{
				Node*	childs[2];
				void*	data;
				};
		};
	// Interfaces
	
	struct	sStkElm
		{
		const Node*	a;
		const Node*	b;
		sStkElm(const Node* na,const Node* nb) : a(na),b(nb) {}
		};
		
	/* ICollide	*/ 
	struct	ICollide
		{
		virtual void	Process(const Node* leaf0,const Node* leaf1) {};
		virtual void	Process(const Node* leaf) {};
		};
		
	// Fields
	Node*			m_root;
	Node*			m_free;
	int				m_lkhd;
	// Methods
					btDbvt();
					~btDbvt();
	void			clear();
	int				leafCount() const;
	void			optimizeBottomUp();
	void			optimizeTopDown(int bu_treshold=128);
	Node*			insert(const Aabb& box,void* data);
	void			update(Node* leaf,const Aabb& box);
	bool			update(Node* leaf,Aabb box,const btVector3& velocity,btScalar margin);
	void			remove(Node* leaf);
	void			collide(btDbvt* tree,
							ICollide* icollide) const;
	void			collide(const Aabb& box,
							ICollide* icollide) const;
	void			collide(const btVector3& org,
							const btVector3& dir,
							ICollide* icollide) const;
	//
	private:
	btDbvt(const btDbvt&)	{}
	};

//
// Inline's
//

//
inline btDbvt::Aabb		btDbvt::Aabb::FromCE(const btVector3& c,const btVector3& e)
{
Aabb box;
box.mi=c-e;box.mx=c+e;
return(box);
}
	
//
inline btDbvt::Aabb		btDbvt::Aabb::FromCR(const btVector3& c,btScalar r)
{
return(FromCE(c,btVector3(r,r,r)));
}
	
//
inline btDbvt::Aabb		btDbvt::Aabb::FromMM(const btVector3& mi,const btVector3& mx)
{
Aabb box;
box.mi=mi;box.mx=mx;
return(box);
}
	
//
inline btDbvt::Aabb		btDbvt::Aabb::FromPoints(const btVector3* pts,int n)
{
Aabb box;
box.mi=box.mx=pts[0];
for(int i=1;i<n;++i)
	{
	box.mi.setMin(pts[i]);
	box.mx.setMax(pts[i]);
	}
return(box);
}

//
inline btDbvt::Aabb		btDbvt::Aabb::FromPoints(const btVector3** ppts,int n)
{
Aabb box;
box.mi=box.mx=*ppts[0];
for(int i=1;i<n;++i)
	{
	box.mi.setMin(*ppts[i]);
	box.mx.setMax(*ppts[i]);
	}
return(box);
}

//
inline void				btDbvt::Aabb::Expand(const btVector3 e)
{
mi-=e;mx+=e;
}
	
//
inline void				btDbvt::Aabb::SignedExpand(const btVector3 e)
{
if(e.x()>0) mx.setX(mx.x()+e.x()); else mi.setX(mi.x()+e.x());
if(e.y()>0) mx.setY(mx.y()+e.y()); else mi.setY(mi.y()+e.y());
if(e.z()>0) mx.setZ(mx.z()+e.z()); else mi.setZ(mi.z()+e.z());
}
	
//
inline bool				btDbvt::Aabb::Contain(const Aabb& a) const
{
return(	(mi.x()<=a.mi.x())&&
		(mi.y()<=a.mi.y())&&
		(mi.z()<=a.mi.z())&&
		(mx.x()>=a.mx.x())&&
		(mx.y()>=a.mx.y())&&
		(mx.z()>=a.mx.z()));
}
	
//
inline bool				Intersect(	const btDbvt::Aabb& a,
									const btDbvt::Aabb& b)
{
return(	(a.mi.x()<=b.mx.x())&&
		(a.mi.y()<=b.mx.y())&&
		(a.mi.z()<=b.mx.z())&&
		(a.mx.x()>=b.mi.x())&&
		(a.mx.y()>=b.mi.y())&&
		(a.mx.z()>=b.mi.z()));
}
	
//
inline btScalar			Proximity(	const btDbvt::Aabb& a,
									const btDbvt::Aabb& b)
{
const btVector3	d=(a.mi+a.mx)-(b.mi+b.mx);
return(btFabs(d.x())+btFabs(d.y())+btFabs(d.z()));
}

//
inline void				Merge(	const btDbvt::Aabb& a,
								const btDbvt::Aabb& b,
								btDbvt::Aabb& r)
{
r=a;
r.mi.setMin(b.mi);
r.mx.setMax(b.mx);
}

//
inline bool				NotEqual(	const btDbvt::Aabb& a,
									const btDbvt::Aabb& b)
{
return(	(a.mi.x()!=b.mi.x())||
		(a.mi.y()!=b.mi.y())||
		(a.mi.z()!=b.mi.z())||
		(a.mx.x()!=b.mx.x())||
		(a.mx.y()!=b.mx.y())||
		(a.mx.z()!=b.mx.z()));
}

#endif
