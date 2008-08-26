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

#ifndef OPTIMIZED_BVH_H
#define OPTIMIZED_BVH_H

#include "BulletCollision/BroadphaseCollision/btQuantizedBvh.h"

class btStridingMeshInterface;


///OptimizedBvh extends the btQuantizedBvh to create AABB tree for triangle meshes, through the btStridingMeshInterface.
ATTRIBUTE_ALIGNED16(class) btOptimizedBvh : public btQuantizedBvh
{
	
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

protected:

public:

	btOptimizedBvh();

	virtual ~btOptimizedBvh();

	void	build(btStridingMeshInterface* triangles,bool useQuantizedAabbCompression, const btVector3& bvhAabbMin, const btVector3& bvhAabbMax);

	void	refit(btStridingMeshInterface* triangles,const btVector3& aabbMin,const btVector3& aabbMax);

	void	refitPartial(btStridingMeshInterface* triangles,const btVector3& aabbMin, const btVector3& aabbMax);

	void	updateBvhNodes(btStridingMeshInterface* meshInterface,int firstNode,int endNode,int index);

};


#endif //OPTIMIZED_BVH_H


