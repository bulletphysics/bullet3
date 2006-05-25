/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#ifndef _FCD_PHYSICS_RIGID_CONSTRAINT_INSTANCE_H_
#define _FCD_PHYSICS_RIGID_CONSTRAINT_INSTANCE_H_

#include "FCDocument/FCDEntityInstance.h"

class FCDocument;
class FCDSceneNode;
class FCDPhysicsModel;
class FCDPhysicsRigidConstraint;


class FCOLLADA_EXPORT FCDPhysicsRigidConstraintInstance : public FCDEntityInstance
{

private:

	FCDPhysicsModel* parent;
	FCDPhysicsRigidConstraint* rigidConstraint;

public:
	FCDPhysicsRigidConstraintInstance(FCDocument* document, FCDEntity* _parent);
	virtual ~FCDPhysicsRigidConstraintInstance();

	// FCDEntity override for RTTI-like
	virtual Type GetType() const { return PHYSICS_RIGID_CONSTRAINT; }

	// Load the geometry instance from the COLLADA document
	virtual FUStatus LoadFromXML(xmlNode* instanceNode);

	// Write out the instantiation information to the xml node tree
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

#endif // _FCD_PHYSICS_RIGID_CONSTRAINT_INSTANCE_H_
