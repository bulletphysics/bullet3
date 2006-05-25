/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#ifndef _FCD_PHYSICSRIGIDBODY_H_
#define _FCD_PHYSICSRIGIDBODY_H_

#include "FCDocument/FCDEntity.h"
#include "FCDocument/FCDEntityInstance.h"
#include "FCDocument/FCDPhysicsParameter.h"
#include "FCDocument/FCDTransform.h"
#include "FCDocument/FCDPhysicsShape.h"
#include "FUtils/FUDaeEnum.h"

class FCDocument;

typedef std::vector<FCDPhysicsParameterGeneric*> FCDPhysicsParameterList;
typedef std::vector<FCDPhysicsShape*> FCDPhysicsShapeList;

class FCOLLADA_EXPORT FCDPhysicsRigidBody : public FCDEntity
{
private:
	string sid;
	bool ownsPhysicsMaterial;
	FCDPhysicsMaterial* physicsMaterial;
	
	FCDPhysicsParameterList parameters;
	FCDPhysicsShapeList physicsShape;

public:
	FCDPhysicsRigidBody(FCDocument* document);
	virtual ~FCDPhysicsRigidBody();

	string GetSid() const { return sid; }
	// Returns the entity type
	virtual Type GetType() const { return PHYSICS_RIGID_BODY; }
	
	void AddParameter(FCDPhysicsParameterGeneric* parameter);
	void CopyParameter(FCDPhysicsParameterGeneric* parameter);

	void SetParameters(FCDPhysicsParameterList& params);
	FCDPhysicsParameterGeneric* FindParameterByReference(const string& reference);

	// Direct Accessors
	FCDPhysicsMaterial* GetPhysicsMaterial() const {return physicsMaterial;}
	void SetPhysicsMaterial(FCDPhysicsMaterial* _physicsMaterial);
	FCDPhysicsShapeList& GetPhysicsShapeList() {return physicsShape;}
	void SetPhysicsShapes(FCDPhysicsShapeList& _physicsShape);
	void SetMaterialOwnership(bool val) {ownsPhysicsMaterial = val;}

	void Flatten();

	// Create a copy of this physicsmodel, with the vertices overwritten
	FCDPhysicsRigidBody* Clone();

	// Read in the <physics_model> node of the COLLADA document
	virtual FUStatus LoadFromXML(xmlNode* node);

	// Write out the <physics_model> node to the COLLADA xml tree
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

#endif // _FCD_PHYSICSRIGIDBODY_H_
