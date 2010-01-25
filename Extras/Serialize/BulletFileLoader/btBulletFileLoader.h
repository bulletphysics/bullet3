/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2010 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef BULLET_FILE_LOADER_H
#define BULLET_FILE_LOADER_H

#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btHashMap.h"

class btBulletFile;
class btCollisionShape;
class btRigidBody;
class btTypedConstraint;
class btDynamicsWorld;
struct ConstraintInput;
class btRigidBodyColladaInfo;

namespace bParse
{
	class btBulletFile;
};


class btBulletFileLoader
{
	btDynamicsWorld* m_dynamicsWorld;

	bool m_verboseDumpAllTypes;

public:
	
	btBulletFileLoader(btDynamicsWorld* world);

	bool	loadFileFromMemory(char* fileName);

	bool	loadFileFromMemory(char *memoryBuffer, int len);

	bool	loadFileFromMemory(bParse::btBulletFile* file);

	void	setVerboseMode(bool verboseDumpAllTypes)
	{
		m_verboseDumpAllTypes = verboseDumpAllTypes;
	}

	bool getVerboseMode() const
	{
		return m_verboseDumpAllTypes;
	}
	
	///those virtuals are called by load
	virtual btTypedConstraint*			createUniversalD6Constraint(
		class btRigidBody* body0,class btRigidBody* otherBody,
			btTransform& localAttachmentFrameRef,
			btTransform& localAttachmentOther,
			const btVector3& linearMinLimits,
			const btVector3& linearMaxLimits,
			const btVector3& angularMinLimits,
			const btVector3& angularMaxLimits,
			bool disableCollisionsBetweenLinkedBodies
			);
	
	virtual btRigidBody*  createRigidBody(bool isDynamic, 
		btScalar mass, 
		const btTransform& startTransform,
		btCollisionShape* shape);

	virtual btCollisionShape* createPlaneShape(const btVector3& planeNormal,btScalar planeConstant);
	virtual btCollisionShape* createBoxShape(const btVector3& halfExtents);
	virtual btCollisionShape* createSphereShape(btScalar radius);
	virtual btCollisionShape* createCapsuleShape(btScalar radius, btScalar height);
	virtual btCollisionShape* createCylinderShapeY(btScalar radius,btScalar height);
	virtual class btTriangleMesh*	createTriangleMeshContainer();
	virtual	btCollisionShape* createBvhTriangleMeshShape(btTriangleMesh* trimesh);
	virtual btCollisionShape* createConvexTriangleMeshShape(btTriangleMesh* trimesh);
	virtual btCollisionShape* createGimpactShape(btTriangleMesh* trimesh);
	virtual class btConvexHullShape* createConvexHullShape();
	virtual class btCompoundShape* createCompoundShape();

};

#endif //BULLET_FILE_LOADER_H