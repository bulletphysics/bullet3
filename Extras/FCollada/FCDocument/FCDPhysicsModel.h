/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#ifndef _FCD_PHYSICSMODEL_H_
#define _FCD_PHYSICSMODEL_H_

#include "FCDocument/FCDEntity.h"
#include "FCDocument/FCDEntityInstance.h"
#include "FUtils/FUDaeEnum.h"

class FCDocument;
class FCDPhysicsRigidBody;
class FCDPhysicsRigidConstraint;

typedef vector<FCDEntityInstance*> FCDEntityInstanceList;
typedef vector<FCDPhysicsRigidBody*> FCDPhysicsRigidBodyList;
typedef vector<FCDPhysicsRigidConstraint*> FCDPhysicsRigidConstraintList;

class FCOLLADA_EXPORT FCDPhysicsModel : public FCDEntity
{
private:
	FCDEntityInstanceList instances;
	FCDPhysicsRigidBodyList rigidBodies;
	FCDPhysicsRigidConstraintList rigidConstraints;

public:
	FCDPhysicsModel(FCDocument* document);
	virtual ~FCDPhysicsModel();

	// Returns the entity type
	virtual Type GetType() const { return PHYSICS_MODEL; }

	// Direct Accessors
	FCDEntityInstanceList& GetInstances() { return instances; }
	const FCDEntityInstanceList& GetInstances() const { return instances; }

	FCDPhysicsRigidBody* FindRigidBodyFromSid(const string& sid);
	FCDPhysicsRigidConstraint* FindRigidConstraintFromSid(const string& sid);

	// Create a copy of this physicsmodel, with the vertices overwritten
	FCDPhysicsModel* Clone(/*FloatList& newPositions, uint32 newPositionsStride, FloatList& newNormals, uint32 newNormalsStride*/);

	// Read in the <physics_model> node of the COLLADA document
	virtual FUStatus LoadFromXML(xmlNode* node);

	// Write out the <physics_model> node to the COLLADA xml tree
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

#endif // _FCD_PHYSICSMODEL_H_
