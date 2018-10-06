#ifndef COLLISION_SHAPE_2_GRAPHICS_H
#define COLLISION_SHAPE_2_GRAPHICS_H

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
class btCollisionShape;

void CollisionShape2TriangleMesh(btCollisionShape* collisionShape, const btTransform& parentTransform, btAlignedObjectArray<btVector3>& vertexPositions, btAlignedObjectArray<btVector3>& vertexNormals, btAlignedObjectArray<int>& indicesOut);

#endif  //COLLISION_SHAPE_2_GRAPHICS_H
