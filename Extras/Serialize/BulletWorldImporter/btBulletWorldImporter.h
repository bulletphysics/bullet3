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


#ifndef BULLET_WORLD_IMPORTER_H
#define BULLET_WORLD_IMPORTER_H

#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btHashMap.h"

class btBulletFile;
class btCollisionShape;
class btCollisionObject;
class btRigidBody;
class btTypedConstraint;
class btDynamicsWorld;
struct ConstraintInput;
class btRigidBodyColladaInfo;
struct btCollisionShapeData;
class btTriangleIndexVertexArray;
class btStridingMeshInterface;
struct btStridingMeshInterfaceData;
class btGImpactMeshShape;

namespace bParse
{
	class btBulletFile;
	
};


class btBulletWorldImporter
{
protected:

	btDynamicsWorld* m_dynamicsWorld;

	bool m_verboseDumpAllTypes;

	btCollisionShape* convertCollisionShape(  btCollisionShapeData* shapeData  );

	btAlignedObjectArray<btCollisionShape*> m_allocatedCollisionShapes;

	btTriangleIndexVertexArray* createMeshInterface(btStridingMeshInterfaceData& meshData);

	static btRigidBody& getFixedBody();
	
public:
	
	btBulletWorldImporter(btDynamicsWorld* world);

	virtual ~btBulletWorldImporter();

	bool	loadFile(const char* fileName);

	///the memoryBuffer might be modified (for example if endian swaps are necessary)
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

	virtual btCollisionObject*  createCollisionObject(	const btTransform& startTransform,	btCollisionShape* shape);


	virtual btCollisionShape* createPlaneShape(const btVector3& planeNormal,btScalar planeConstant);
	virtual btCollisionShape* createBoxShape(const btVector3& halfExtents);
	virtual btCollisionShape* createSphereShape(btScalar radius);
	virtual btCollisionShape* createCapsuleShapeX(btScalar radius, btScalar height);
	virtual btCollisionShape* createCapsuleShapeY(btScalar radius, btScalar height);
	virtual btCollisionShape* createCapsuleShapeZ(btScalar radius, btScalar height);
	
	virtual btCollisionShape* createCylinderShapeX(btScalar radius,btScalar height);
	virtual btCollisionShape* createCylinderShapeY(btScalar radius,btScalar height);
	virtual btCollisionShape* createCylinderShapeZ(btScalar radius,btScalar height);
	virtual class btTriangleIndexVertexArray*	createTriangleMeshContainer();
	virtual	btCollisionShape* createBvhTriangleMeshShape(btStridingMeshInterface* trimesh);
	virtual btCollisionShape* createConvexTriangleMeshShape(btStridingMeshInterface* trimesh);
	virtual btGImpactMeshShape* createGimpactShape(btStridingMeshInterface* trimesh);
	virtual class btConvexHullShape* createConvexHullShape();
	virtual class btCompoundShape* createCompoundShape();

};

#endif //BULLET_WORLD_IMPORTER_H

