/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#ifndef _FCD_PHYSICSMATERIAL_H_
#define _FCD_PHYSICSMATERIAL_H_

#include "FCDocument/FCDEntity.h"

class FCDocument;

class FCOLLADA_EXPORT FCDPhysicsMaterial : public FCDEntity
{
private:
	float staticFriction;
	float dynamicFriction;
	float restitution;

public:
	FCDPhysicsMaterial(FCDocument* document);
	virtual ~FCDPhysicsMaterial();

	// Accessors
	virtual Type GetType() const { return FCDEntity::PHYSICS_MATERIAL; }
	float GetStaticFriction() const { return staticFriction; }
	void  SetStaticFriction(float _staticFriction) { staticFriction = _staticFriction; }
	float GetDynamicFriction() const { return dynamicFriction; }
	void  SetDynamicFriction(float _dynamicFriction) { dynamicFriction = _dynamicFriction; }
	float GetRestitution() const { return restitution; }
	void  SetRestitution(float _restitution) { restitution = _restitution;}

	// Cloning
	FCDPhysicsMaterial* Clone();

	// Parse COLLADA document's <material> element
	virtual FUStatus LoadFromXML(xmlNode* physicsMaterialNode);

	// Write out the <material> element to the COLLADA xml tree
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

#endif // _FCD_MATERIAL_H_
