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

//
// Defaults volumes
//

/* btDbvtAabbMm			*/ 
struct	btDbvtAabbMm
{
	inline btVector3			Center() const	{ return((mi+mx)/2); }
	inline btVector3			Extent() const	{ return((mx-mi)/2); }
	inline const btVector3&		Mins() const	{ return(mi); }
	inline const btVector3&		Maxs() const	{ return(mx); }
	inline btVector3			Lengths() const	{ return(mx-mi); }
	static inline btDbvtAabbMm	FromCE(const btVector3& c,const btVector3& e);
	static inline btDbvtAabbMm	FromCR(const btVector3& c,btScalar r);
	static inline btDbvtAabbMm	FromMM(const btVector3& mi,const btVector3& mx);
	static inline btDbvtAabbMm	FromPoints(const btVector3* pts,int n);
	static inline btDbvtAabbMm	FromPoints(const btVector3** ppts,int n);
	inline void					Expand(const btVector3 e);
	inline void					SignedExpand(const btVector3 e);
	inline bool					Contain(const btDbvtAabbMm& a) const;
	inline friend bool			Intersect(	const btDbvtAabbMm& a,
		const btDbvtAabbMm& b);
	inline friend bool			Intersect(	const btDbvtAabbMm& a,
		const btVector3& b);
	inline friend btScalar		Proximity(	const btDbvtAabbMm& a,
		const btDbvtAabbMm& b);
	inline friend void			Merge(	const btDbvtAabbMm& a,
		const btDbvtAabbMm& b,
		btDbvtAabbMm& r);
	inline friend bool			NotEqual(	const btDbvtAabbMm& a,
		const btDbvtAabbMm& b);
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
	struct	sStkElm
	{
		const Node*	a;
		const Node*	b;
		sStkElm(const Node* na,const Node* nb) : a(na),b(nb) {}
	};

	// Interfaces

	/* ICollide	*/ 
	struct	ICollide
	{
		virtual void	Process(const Node*,const Node*)=0;
		virtual void	Process(const Node*)=0;
		virtual bool	Descent(const Node*)=0;
	};
	
	// Constants
	enum	{
		TREETREE_STACKSIZE		=	128,
		VOLUMETREE_STACKSIZE	=	64,
	};

	// Fields
	Node*			m_root;
	Node*			m_free;
	int				m_lkhd;
	int				m_leafs;
	// Methods
	btDbvt();
	~btDbvt();
	void			clear();
	bool			empty() const { return(0==m_root); }
	void			optimizeBottomUp();
	void			optimizeTopDown(int bu_treshold=128);
	Node*			insert(const Volume& box,void* data);
	void			update(Node* leaf,int lookahead=1);
	void			update(Node* leaf,const Volume& volume);
	bool			update(Node* leaf,Volume volume,const btVector3& velocity,btScalar margin);
	bool			update(Node* leaf,Volume volume,const btVector3& velocity);
	bool			update(Node* leaf,Volume volume,btScalar margin);
	void			remove(Node* leaf);
	void			collide(btDbvt* tree,
		ICollide* icollide) const;
	void			collide(btDbvt::Node* node,
		ICollide* icollide) const;
	void			collide(const Volume& volume,
		ICollide* icollide) const;
	void			collide(const btVector3& org,
		const btVector3& dir,
		ICollide* icollide) const;
	void			collide(ICollide* icollide) const;
	// Generics : T must implement ICollide

	void			collideGeneric(	btDbvt* tree,ICollide* policy) const;

	void			collideGeneric(	btDbvt::Node* node,ICollide*  policy) const;

	void			collideGeneric(const Volume& volume,ICollide*  policy) const;

	void			collideGeneric(ICollide*  policy) const;
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
inline bool				Intersect(	const btDbvtAabbMm& a,
								  const btDbvtAabbMm& b)
{
#if 0
	const btScalar	mi[]={	b.mx.x()-a.mi.x(),
		b.mx.y()-a.mi.y(),
		b.mx.z()-a.mi.z()};
	const unsigned*	imi=(const unsigned*)mi;
	if((imi[0]|imi[1]|imi[2])&0x80000000) return(false);
	const btScalar	mx[]={	a.mx.x()-b.mi.x(),
		a.mx.y()-b.mi.y(),
		a.mx.z()-b.mi.z()};
	const unsigned*	imx=(const unsigned*)mx;
	if((imx[0]|imx[1]|imx[2])&0x80000000) return(false);
	return(true);
#else
	return(	(a.mi.x()<=b.mx.x())&&
		(a.mi.y()<=b.mx.y())&&
		(a.mi.z()<=b.mx.z())&&
		(a.mx.x()>=b.mi.x())&&
		(a.mx.y()>=b.mi.y())&&
		(a.mx.z()>=b.mi.z()));
#endif
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
	r=a;
	r.mi.setMin(b.mi);
	r.mx.setMax(b.mx);
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
// Generic's
//

//
inline void			btDbvt::collideGeneric(	btDbvt::Node* node,ICollide*  policy) const
{
	if(m_root&&node)
	{
		btAlignedObjectArray<sStkElm>	stack;
		stack.reserve(TREETREE_STACKSIZE);
		stack.push_back(sStkElm(m_root,node));
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
			else if(Intersect(p.a->volume,p.b->volume))
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
						policy->Process(p.a,p.b);
					}
				}
			}
		} while(stack.size()>0);
	}
}

//
inline void			btDbvt::collideGeneric(	btDbvt* tree,ICollide* policy) const
{
	collideGeneric(tree->m_root,policy);
}

//
inline void			btDbvt::collideGeneric(const Volume& volume,ICollide* policy) const
{
	if(m_root)
	{
		btAlignedObjectArray<const Node*>	stack;
		stack.reserve(VOLUMETREE_STACKSIZE);
		stack.push_back(m_root);
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
					policy->Process(n);
				}
			}
		} while(stack.size()>0);
	}
}

//

inline void			btDbvt::collideGeneric(ICollide* policy) const
{
	if(m_root)
	{
		btAlignedObjectArray<const Node*>	stack;
		stack.reserve(VOLUMETREE_STACKSIZE);
		stack.push_back(m_root);
		do	{
			const Node*	n=stack[stack.size()-1];
			stack.pop_back();
			if(policy->Descent(n))
			{
				if(n->isinternal())
				{ stack.push_back(n->childs[0]);stack.push_back(n->childs[1]); }
				else
				{ policy->Process(n); }
			}
		} while(stack.size()>0);
	}
}

#endif
