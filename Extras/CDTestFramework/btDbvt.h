/*
Bounding Volume Hierarchy,  btDbvt.h
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

#ifndef _6095FD2A_2B3C_4c47_AB85_D56C1DD1A210_
#define _6095FD2A_2B3C_4c47_AB85_D56C1DD1A210_

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btPoint3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btIDebugDraw.h"

//
// Dynamic bounding volume tree
//
struct	btDbvt
	{
	// Types
	
	/* AabbCe		*/ 
	struct	Aabb
		{
		btVector3			mi,mx;
		inline btVector3	Center() const { return((mi+mx)/2); }
		inline btVector3	Extent() const { return((mx-mi)/2); }
		static inline Aabb	FromCE(const btVector3& c,const btVector3& e)
			{
			Aabb box;
			box.mi=c-e;box.mx=c+e;
			return(box);
			}
		inline friend bool	operator==(const Aabb& a,const Aabb& b)
			{
			return(	((a.mi-b.mi).length2()==0)&&
					((a.mx-b.mx).length2()==0));
			}
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
	// Fields
	Node*						m_root;
	Node*						m_stock;
	int							m_nleafs;
	// Methods
	static btDbvt*	Create();
	void			Delete();
	void			OptimizeBottomUp();
	void			OptimizeTopDown();
	Node*			Insert(	const btVector3& center,
							const btVector3& extent,
							void* data);
	bool			Update(	Node* leaf,
							const btVector3& center,
							const btVector3& extent,
							btScalar margin=0);
	void			Remove(	Node* leaf);
	// Inline's
	inline Node*	Insert(	const btVector3& center,
							btScalar radius,
							void* data)
		{
		return(Insert(center,btVector3(radius,radius,radius),data));
		}
	bool			Update(	Node* leaf,
							const btVector3& center,
							btScalar radius,
							btScalar margin=0)
		{
		return(Update(leaf,center,btVector3(radius,radius,radius),margin));
		}
	};

#endif