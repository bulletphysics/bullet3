/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#ifndef _FCD_PHYSICS_RIGID_CONSTRAINT_H_
#define _FCD_PHYSICS_RIGID_CONSTRAINT_H_

#include "FCDocument/FCDEntity.h"
#include "FCDocument/FCDEntityInstance.h"
#include "FCDocument/FCDTransform.h"
#include "FUtils/FUDaeEnum.h"

class FCDocument;
class FCDPhysicsModel;
class FCDPhysicsRigidBody;
typedef vector<FCDTransform*> FCDTransformList;


class FCOLLADA_EXPORT FCDPhysicsRigidConstraint : public FCDEntity
{
private:
	string sid;
	bool enabled;
	bool interpenetrate;
	FCDPhysicsRigidBody* referenceRigidBody;
	FCDPhysicsRigidBody* targetRigidBody;
	FMVector3 limitsLinearMin;
	FMVector3 limitsLinearMax;
	FMVector3 limitsSCTMin;
	FMVector3 limitsSCTMax;

	FCDPhysicsModel* parent;

	float springLinearStiffness;
	float springLinearDamping;
	float springLinearTargetValue;

	float springAngularStiffness;
	float springAngularDamping;
	float springAngularTargetValue;

	FCDTransformList transformsRef;
	FCDTransformList transformsTar;

public:
	FCDPhysicsRigidConstraint(FCDocument* document, FCDPhysicsModel* _parent);
	virtual ~FCDPhysicsRigidConstraint();

	string GetSid() const { return sid; }
	// Returns the entity type
	virtual Type GetType() const { return PHYSICS_RIGID_CONSTRAINT; }

	const bool& GetEnabled() const { return enabled;}
	const bool& GetInterpenetrate() const { return interpenetrate;}

	FCDPhysicsRigidBody* GetReferenceRigidBody() const { return referenceRigidBody;}
	FCDPhysicsRigidBody* GetTargetRigidBody() const { return targetRigidBody;}

	const FCDTransformList& GetTransformsRef() const { return transformsRef; }
	const FCDTransformList& GetTransformsTar() const { return transformsTar; }

	FMVector3 GetLimitsLinearMin() const { return limitsLinearMin;}
	FMVector3 GetLimitsLinearMax() const { return limitsLinearMax;}
	FMVector3 GetLimitsSCTMin() const { return limitsSCTMin;}
	FMVector3 GetLimitsSCTMax() const { return limitsSCTMax;}

	float GetSpringLinearStiffness() const { return springLinearStiffness;}
	float GetSpringLinearDamping() const { return springLinearDamping;}
	float GetSpringLinearTargetValue() const { return springLinearTargetValue;}
	float GetSpringAngularStiffness() const { return springAngularStiffness;}
	float GetSpringAngularDamping() const { return springAngularDamping;}
	float GetSpringAngularTargetValue() const { return springAngularTargetValue;}

	// Create a copy of this physicsmodel, with the vertices overwritten
	FCDPhysicsRigidConstraint* Clone();

	// Read in the <physics_model> node of the COLLADA document
	virtual FUStatus LoadFromXML(xmlNode* node);

	// Write out the <physics_model> node to the COLLADA xml tree
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

#endif // _FCD_PHYSICS_RIGID_CONSTRAINT_H_
