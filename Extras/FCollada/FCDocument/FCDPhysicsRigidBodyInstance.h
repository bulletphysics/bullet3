/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#ifndef _FCD_PHYSICS_RIGID_BODY_ENTITY_H_
#define _FCD_PHYSICS_RIGID_BODY_ENTITY_H_

#include "FCDocument/FCDEntityInstance.h"

class FCDocument;
class FCDSceneNode;
class FCDPhysicsRigidBody;
class FCDPhysicsMaterial;
class FCDPhysicsModelInstance;
class FCDPhysicsParameterGeneric;

typedef std::vector<FCDPhysicsParameterGeneric*> FCDPhysicsParameterList;

class FCOLLADA_EXPORT FCDPhysicsRigidBodyInstance : public FCDEntityInstance
{

private:
	FCDPhysicsParameterList parameters;

	FCDPhysicsModel* parent;
	FCDPhysicsRigidBody* rigidBody;
	FCDPhysicsMaterial* physicsMaterial;
	bool ownsPhysicsMaterial;
	FCDSceneNode* targetNode;

public:
	FCDPhysicsRigidBodyInstance(FCDocument* document, FCDEntity* _parent);
	virtual ~FCDPhysicsRigidBodyInstance();

	void AddParameter(FCDPhysicsParameterGeneric* parameter);

	FCDPhysicsRigidBody* FlattenRigidBody();

	FCDSceneNode* GetTargetNode() const {return targetNode;}

	// FCDEntity override for RTTI-like
	virtual Type GetType() const { return PHYSICS_RIGID_BODY; }

	// Load the geometry instance from the COLLADA document
	virtual FUStatus LoadFromXML(xmlNode* instanceNode);

	// Write out the instantiation information to the xml node tree
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

#endif // _FCD_PHYSICS_RIGID_BODY_ENTITY_H_
