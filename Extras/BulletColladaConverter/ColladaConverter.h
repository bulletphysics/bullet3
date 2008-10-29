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



#ifndef COLLADA_CONVERTER_H
#define COLLADA_CONVERTER_H

#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btHashMap.h"

class btCollisionShape;
class btRigidBody;
class btTypedConstraint;
class btDynamicsWorld;
struct ConstraintInput;
class btRigidBodyColladaInfo;

class btRigidBodyColladaInfo
{
public:
	class domNode* m_node;
	class domRigid_body* m_domRigidBody;
	class domInstance_rigid_body* m_instanceRigidBody;
	btRigidBody*	m_body;

	int	getUid();

	btRigidBodyColladaInfo (btRigidBody* body, domNode* node, domRigid_body* rigidBody, domInstance_rigid_body* instanceRigidBody)
	{
		m_body = body;
		m_node = node;
		m_domRigidBody = rigidBody;
		m_instanceRigidBody = instanceRigidBody;
	}
};




class btRigidConstraintColladaInfo
{
public:
	btTypedConstraint* m_typedConstraint;
	class domRigid_constraint* m_domRigidConstraint;
	class domInstance_rigid_constraint* m_domInstanceRigidConstraint;

	int getUid();

	btRigidConstraintColladaInfo (btTypedConstraint* constraint, domRigid_constraint* rigidConstraint, domInstance_rigid_constraint* domInstanceRigidConstraint) 
	{
		m_typedConstraint = constraint;
		m_domRigidConstraint = rigidConstraint;
		m_domInstanceRigidConstraint = domInstanceRigidConstraint;
	}


};





///ColladaConverter helps converting the physics assets from COLLADA DOM into physics objects
class ColladaConverter
{
	char m_cleaned_filename[513];
	
	
protected:

	btDynamicsWorld* m_dynamicsWorld;

	btAlignedObjectArray<class btCollisionShape*>	m_allocatedCollisionShapes;

	
	btHashMap<btHashKeyPtr<btRigidBodyColladaInfo*>,btRigidBodyColladaInfo*>	m_rbUserInfoHashMap;
	btHashMap<btHashKeyPtr<btRigidConstraintColladaInfo*>,btRigidConstraintColladaInfo*> m_constraintUserInfoHashMap;
	
	class DAE* m_collada;
	class domCOLLADA* m_dom;
	char* m_filename;

	float m_unitMeterScaling;
	bool	m_use32bitIndices;
	bool	m_use4componentVertices;
	
	void PreparePhysicsObject(struct btRigidBodyInput& input, bool isDynamics, float mass,btCollisionShape* colShape, const btVector3& linearVelocity, const btVector3& angularVelocity);
	
	void prepareConstraints(ConstraintInput& input);

	void ConvertRigidBodyRef( struct btRigidBodyInput& , struct btRigidBodyOutput& output );

	bool convert ();

	btRigidBodyColladaInfo*			findRigidBodyColladaInfo(const btRigidBody* body);
	btRigidConstraintColladaInfo*	findRigidConstraintColladaInfo(btTypedConstraint* constraint);

	bool instantiateDom ();

	/* Searches based on matching name/id */
	class domNode* findNode (const char* nodeName);
	class domRigid_body* findRigid_body (const char* rigidBodyName);
	class domInstance_rigid_body* findRigid_body_instance (const char* nodeName);
	class domRigid_constraint* findRigid_constraint (const char* constraintName);
	class domGeometry* findGeometry (const char* shapeName);

	class domNode* findNode (btRigidBody* rb);
	class domRigid_body* findRigid_body (const btRigidBody* rb);
	class domInstance_rigid_body* findRigid_body_instance (btRigidBody* rb);
	class domRigid_constraint* findRigid_constraint (btTypedConstraint* constraint);
	//class domGeometry* findGeometry (btCollisionShape* shape);

	/* These are the locations that NEW elements will be added to */
	class domLibrary_geometries* getDefaultGeomLib ();
	class domLibrary_physics_materials* getDefaultMaterialsLib ();
	class domPhysics_model* getDefaultPhysicsModel ();
	class domInstance_physics_model* getDefaultInstancePhysicsModel ();

	/* Currently we assume that there is only a single physics and visual scene */
	class domPhysics_scene* getDefaultPhysicsScene ();
	class domVisual_scene* getDefaultVisualScene ();

	void buildShape (btCollisionShape* shape, void* collada_shape, const char* shapeName);

	void addConvexHull (btCollisionShape* shape, const char* nodeName);
	void addConvexMesh (btCollisionShape* shape, const char* nodeName);
	void addConcaveMesh(btCollisionShape* shape, const char* nodeName);
	class domNode* addNode       (btRigidBody* body, const char* nodeName, const char* shapeName);
	class domRigid_constraint*	addConstraint (btTypedConstraint* constraint, const char* constraintName);
	class domInstance_rigid_constraint* addConstraintInstance (btTypedConstraint* constraint, const char* constraintName);
	void addMaterial   (btRigidBody* body, const char* nodeName);
	class domRigid_body* addRigidBody  (btRigidBody* body, const char* nodeName, const char* shapeName);
	class domInstance_rigid_body*  addRigidBodyInstance (btRigidBody* body, const char* nodeName);

	void updateRigidBodyPosition (btRigidBody* body, class domNode* node);
	void updateRigidBodyVelocity (btRigidBody* body);
	void updateConstraint (btTypedConstraint* constraint, class domRigid_constraint* rigidConstraint);

	void syncOrAddGeometry (btCollisionShape* shape, const char* nodeName);
	void syncOrAddRigidBody (btRigidBody* body);
	void syncOrAddConstraint (btTypedConstraint* constraint);
public:
	ColladaConverter(btDynamicsWorld* dynaWorld);
	virtual ~ColladaConverter ();

	///load a COLLADA .dae file
	bool	load(const char* filename);
	
	// resets the collada converter state
	void    reset ();

	///save a snapshot in COLLADA physics .dae format.
	///if the filename is left empty, modify the filename used during loading
	bool	save(const char* filename = 0);

	void	setUse32bitIndices(bool use32bitIndices)
	{
		m_use32bitIndices = use32bitIndices;
	}
	bool	setUse32bitIndices() const
	{
		return m_use32bitIndices;
	}

	void	setUse4componentVertices(bool use4componentVertices)
	{
		m_use4componentVertices = use4componentVertices;
	}
	bool	getUse4componentVertices() const
	{
		return m_use4componentVertices;
	}
	
	void registerRigidBody(btRigidBody* body, const char* name);
	void deRegisterRigidBody(btRigidBody* body);
	const char* getName (btRigidBody* body);

	void registerConstraint(btTypedConstraint* constraint, const char* name);
	void deRegisterConstraint(btTypedConstraint* constraint);
	const char* getName (btTypedConstraint* constraint);

	///those virtuals are called by load and save.
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
		float mass, 
		const btTransform& startTransform,
		btCollisionShape* shape);
	virtual int getNumRigidBodies ();
	virtual btRigidBody* getRigidBody (int i);
	virtual int getNumConstraints ();
	virtual btTypedConstraint* getConstraint (int i);
	virtual	void	setGravity(const btVector3& gravity);
	virtual btVector3 getGravity ();
	virtual	void	setCameraInfo(const btVector3& up, int forwardAxis)
	{
	};

	virtual btCollisionShape* createPlaneShape(const btVector3& planeNormal,btScalar planeConstant);
	virtual btCollisionShape* createBoxShape(const btVector3& halfExtents);
	virtual btCollisionShape* createSphereShape(btScalar radius);
	virtual btCollisionShape* createCylinderShapeY(btScalar radius,btScalar height);
	virtual class btTriangleMesh*	createTriangleMeshContainer();
	virtual	btCollisionShape* createBvhTriangleMeshShape(btTriangleMesh* trimesh);
	virtual btCollisionShape* createConvexTriangleMeshShape(btTriangleMesh* trimesh);
	virtual class btConvexHullShape* createConvexHullShape();
	virtual class btCompoundShape* createCompoundShape();

	int	getNumCollisionShapes() const;
	btCollisionShape* getCollisionShape(int shapeIndex);
	void	deleteAllocatedCollisionShapes();
	void	deleteShape(btCollisionShape* shape);


	char* getLastFileName();

	char* fixFileName(const char* lpCmdLine);

};

#endif //COLLADA_CONVERTER_H


