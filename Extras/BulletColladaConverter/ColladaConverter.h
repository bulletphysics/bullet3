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

class btCollisionShape;
class btRigidBody;
class btTypedConstraint;

class ConstraintInput;

///ColladaConverter helps converting the physics assets from COLLADA DOM into physics objects
class ColladaConverter
{
protected:
	class DAE* m_collada;
	class domCOLLADA* m_dom;
	char* m_filename;

	float m_unitMeterScaling;
	
	void PreparePhysicsObject(struct btRigidBodyInput& input, bool isDynamics, float mass,btCollisionShape* colShape, btVector3 linearVelocity, btVector3 angularVelocity);
	
	void prepareConstraints(ConstraintInput& input);

	void ConvertRigidBodyRef( struct btRigidBodyInput& , struct btRigidBodyOutput& output );

	bool convert ();

	/* Searches based on matching name/id */
	class domNode* findNode (const char* nodeName);
	class domRigid_body* findRigid_body (const char* rigidBodyName);
	class domInstance_rigid_body* findRigid_body_instance (const char* nodeName);
	class domRigid_constraint* findRigid_constraint (const char* constraintName);
	class domGeometry* findGeometry (const char* shapeName);

	/* Use btTypedUserInfo->getPrivatePointer() */
	class domNode* findNode (btRigidBody* rb);
	class domRigid_body* findRigid_body (btRigidBody* rb);
	class domInstance_rigid_body* findRigid_body_instance (btRigidBody* rb);
	class domRigid_constraint* findRigid_constraint (btTypedConstraint* constraint);
	class domGeometry* findGeometry (btCollisionShape* shape);

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
	void addNode       (btRigidBody* body, const char* nodeName, const char* shapeName);
	void addConstraint (btTypedConstraint* constraint, const char* constraintName);
	void addConstraintInstance (btTypedConstraint* constraint, const char* constraintName);
	void addMaterial   (btRigidBody* body, const char* nodeName);
	void addRigidBody  (btRigidBody* body, const char* nodeName, const char* shapeName);
	void addRigidBodyInstance (btRigidBody* body, const char* nodeName);

	void updateRigidBodyPosition (btRigidBody* body, class domNode* node);
	void updateRigidBodyVelocity (btRigidBody* body);
	void updateConstraint (btTypedConstraint* constraint, class domRigid_constraint* rigidConstraint);

	void syncOrAddGeometry (btCollisionShape* shape, const char* nodeName);
	void syncOrAddRigidBody (btRigidBody* body);
	void syncOrAddConstraint (btTypedConstraint* constraint);
public:
	ColladaConverter();

	///load a COLLADA .dae file
	bool	load(const char* filename);
	
	///save a snapshot in COLLADA physics .dae format.
	///if the filename is left empty, modify the filename used during loading
	bool	save(const char* filename = 0);

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
			) = 0;
	virtual btRigidBody*  createRigidBody(bool isDynamic, 
		float mass, 
		const btTransform& startTransform,
		btCollisionShape* shape) = 0;
	virtual int getNumRigidBodies () = 0;
	virtual btRigidBody* getRigidBody (int i) = 0;
	virtual int getNumConstraints () = 0;
	virtual btTypedConstraint* getConstraint (int i) = 0;
	virtual	void	setGravity(const btVector3& gravity) = 0;
	virtual btVector3 getGravity () = 0;
	virtual	void	setCameraInfo(const btVector3& up, int forwardAxis) = 0;
};

#endif //COLLADA_CONVERTER_H


