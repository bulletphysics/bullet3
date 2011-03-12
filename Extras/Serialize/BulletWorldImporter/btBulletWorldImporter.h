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
class btOptimizedBvh;
struct btTriangleInfoMap;
class btBvhTriangleMeshShape;
class btPoint2PointConstraint;
class btHingeConstraint;
class btConeTwistConstraint;
class btGeneric6DofConstraint;
class btGeneric6DofSpringConstraint;
class btSliderConstraint;



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

	btAlignedObjectArray<btCollisionShape*>  m_allocatedCollisionShapes;
	btAlignedObjectArray<btCollisionObject*> m_allocatedRigidBodies;
	btAlignedObjectArray<btTypedConstraint*> m_allocatedConstraints;
	btAlignedObjectArray<btOptimizedBvh*>	 m_allocatedBvhs;
	btAlignedObjectArray<btTriangleInfoMap*> m_allocatedTriangleInfoMaps;
	btAlignedObjectArray<btTriangleIndexVertexArray*> m_allocatedTriangleIndexArrays;
	btAlignedObjectArray<btStridingMeshInterfaceData*> m_allocatedbtStridingMeshInterfaceDatas;

	btAlignedObjectArray<char*>				m_allocatedNames;

	btAlignedObjectArray<int*>				m_indexArrays;
	btAlignedObjectArray<short int*>		m_shortIndexArrays;
	btAlignedObjectArray<btVector3FloatData*>	m_floatVertexArrays;
	btAlignedObjectArray<btVector3DoubleData*>	m_doubleVertexArrays;


	btHashMap<btHashPtr,btOptimizedBvh*>	m_bvhMap;
	btHashMap<btHashPtr,btTriangleInfoMap*>	m_timMap;
	
	btHashMap<btHashString,btCollisionShape*>	m_nameShapeMap;
	btHashMap<btHashString,btRigidBody*>	m_nameBodyMap;
	btHashMap<btHashString,btTypedConstraint*>	m_nameConstraintMap;
	btHashMap<btHashPtr,const char*>	m_objectNameMap;

	btHashMap<btHashPtr,btCollisionShape*>	m_shapeMap;
	btHashMap<btHashPtr,btCollisionObject*>	m_bodyMap;


	//methods

	

	static btRigidBody& getFixedBody();

	char*	duplicateName(const char* name);

public:
	
	btBulletWorldImporter(btDynamicsWorld* world=0);

	virtual ~btBulletWorldImporter();

	///delete all memory collision shapes, rigid bodies, constraints etc. allocated during the load.
	///make sure you don't use the dynamics world containing objects after you call this method
	virtual void deleteAllData();

	bool	loadFile(const char* fileName);

	///the memoryBuffer might be modified (for example if endian swaps are necessary)
	bool	loadFileFromMemory(char *memoryBuffer, int len);

	bool	loadFileFromMemory(bParse::btBulletFile* file);

	//call make sure bulletFile2 has been parsed, either using btBulletFile::parse or btBulletWorldImporter::loadFileFromMemory
	virtual	bool	convertAllObjects(bParse::btBulletFile* file);

	void	setVerboseMode(bool verboseDumpAllTypes)
	{
		m_verboseDumpAllTypes = verboseDumpAllTypes;
	}

	bool getVerboseMode() const
	{
		return m_verboseDumpAllTypes;
	}

	// query for data
	int	getNumCollisionShapes() const;
	btCollisionShape* getCollisionShapeByIndex(int index);
	int getNumRigidBodies() const;
	btCollisionObject* getRigidBodyByIndex(int index) const;
	int getNumConstraints() const;
	btTypedConstraint* getConstraintByIndex(int index) const;
	int getNumBvhs() const;
	btOptimizedBvh*  getBvhByIndex(int index) const;
	int getNumTriangleInfoMaps() const;
	btTriangleInfoMap* getTriangleInfoMapByIndex(int index) const;
	
	// queris involving named objects
	btCollisionShape* getCollisionShapeByName(const char* name);
	btRigidBody* getRigidBodyByName(const char* name);
	btTypedConstraint* getConstraintByName(const char* name);
	const char*	getNameForPointer(const void* ptr) const;

	///those virtuals are called by load and can be overridden by the user

	//bodies
	virtual btRigidBody*  createRigidBody(bool isDynamic, 	btScalar mass, 	const btTransform& startTransform,	btCollisionShape* shape,const char* bodyName);
	virtual btCollisionObject*  createCollisionObject(	const btTransform& startTransform,	btCollisionShape* shape,const char* bodyName);

	///shapes

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
	virtual	btBvhTriangleMeshShape* createBvhTriangleMeshShape(btStridingMeshInterface* trimesh, btOptimizedBvh* bvh);
	virtual btCollisionShape* createConvexTriangleMeshShape(btStridingMeshInterface* trimesh);
	virtual btGImpactMeshShape* createGimpactShape(btStridingMeshInterface* trimesh);
	virtual btStridingMeshInterfaceData* createStridingMeshInterfaceData(btStridingMeshInterfaceData* interfaceData);

	virtual class btConvexHullShape* createConvexHullShape();
	virtual class btCompoundShape* createCompoundShape();
	virtual class btScaledBvhTriangleMeshShape* createScaledTrangleMeshShape(btBvhTriangleMeshShape* meshShape,const btVector3& localScalingbtBvhTriangleMeshShape);

	virtual btTriangleIndexVertexArray* createMeshInterface(btStridingMeshInterfaceData& meshData);

	///acceleration and connectivity structures
	virtual btOptimizedBvh*	createOptimizedBvh();
	virtual btTriangleInfoMap* createTriangleInfoMap();

	///constraints
	virtual btPoint2PointConstraint* createPoint2PointConstraint(btRigidBody& rbA,btRigidBody& rbB, const btVector3& pivotInA,const btVector3& pivotInB);
	virtual btPoint2PointConstraint* createPoint2PointConstraint(btRigidBody& rbA,const btVector3& pivotInA);
	virtual btHingeConstraint* createHingeConstraint(btRigidBody& rbA,btRigidBody& rbB, const btTransform& rbAFrame, const btTransform& rbBFrame, bool useReferenceFrameA=false);
	virtual btHingeConstraint* createHingeConstraint(btRigidBody& rbA,const btTransform& rbAFrame, bool useReferenceFrameA=false);
	virtual btConeTwistConstraint* createConeTwistConstraint(btRigidBody& rbA,btRigidBody& rbB,const btTransform& rbAFrame, const btTransform& rbBFrame);
	virtual btConeTwistConstraint* createConeTwistConstraint(btRigidBody& rbA,const btTransform& rbAFrame);
	virtual btGeneric6DofConstraint* createGeneric6DofConstraint(btRigidBody& rbA, btRigidBody& rbB, const btTransform& frameInA, const btTransform& frameInB ,bool useLinearReferenceFrameA);
    virtual btGeneric6DofConstraint* createGeneric6DofConstraint(btRigidBody& rbB, const btTransform& frameInB, bool useLinearReferenceFrameB);
	virtual btGeneric6DofSpringConstraint* createGeneric6DofSpringConstraint(btRigidBody& rbA, btRigidBody& rbB, const btTransform& frameInA, const btTransform& frameInB ,bool useLinearReferenceFrameA);
	virtual btSliderConstraint* createSliderConstraint(btRigidBody& rbA, btRigidBody& rbB, const btTransform& frameInA, const btTransform& frameInB ,bool useLinearReferenceFrameA);
    virtual btSliderConstraint* createSliderConstraint(btRigidBody& rbB, const btTransform& frameInB, bool useLinearReferenceFrameA);

};

#endif //BULLET_WORLD_IMPORTER_H

