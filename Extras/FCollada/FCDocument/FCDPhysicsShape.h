/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#ifndef _FCD_PHYSICS_SHAPE_H_
#define _FCD_PHYSICS_SHAPE_H_

#include "FCDocument/FCDEntity.h"
#include "FCDocument/FCDEntityInstance.h"
#include "FUtils/FUDaeEnum.h"

class FCDocument;
class FCDPhysicsRigidBody;
class FCDPhysicsRigidConstraint;
class FCDGeometryInstance;
class FCDPhysicsAnalyticalGeometry;
class FCDTransform;

typedef vector<FCDTransform*> FCDTransformList;
typedef vector<FCDPhysicsRigidBody*> FCDPhysicsRigidBodyList;

class FCOLLADA_EXPORT FCDPhysicsShape : public FCDEntity
{
private:
	bool hollow;
	FCDPhysicsMaterial* physicsMaterial;
	bool ownsPhysicsMaterial;
	
	//one of these two will define the rigid body
	FCDGeometryInstance* geometry;
	FCDPhysicsAnalyticalGeometry* analGeom; //pun not intended

	bool ownsGeometryInstance;

	float mass;
	float density;
	FCDTransformList transforms;


public:
	FCDPhysicsShape(FCDocument* document);
	virtual ~FCDPhysicsShape();

	// Returns the entity type
	virtual Type GetType() const { return PHYSICS_SHAPE; }

	const float& GetMass() const {return mass;}
	void SetMass(float _mass) {mass = _mass;}
	const float& GetDensity() const {return density;}
	void SetDensity(float _density) {density = _density;}
	FCDPhysicsAnalyticalGeometry* GetAnalyticalGeometry() const {return analGeom;}
	void SetAnalyticalGeometry(FCDPhysicsAnalyticalGeometry* _analGeom) {analGeom = _analGeom;}
	FCDGeometryInstance* GetGeometryInstance() const {return geometry;}
	void SetGeometryInstance(FCDGeometryInstance* _geometry) {geometry = _geometry;}
	void SetGeometryInstanceOwnership(bool val) {ownsGeometryInstance = val;}
	FCDPhysicsMaterial* GetPhysicsMaterial() const {return physicsMaterial;}
	void SetPhysicsMaterial(FCDPhysicsMaterial* _physicsMaterial) {physicsMaterial = _physicsMaterial;}
	bool ownsItsPhysicsMaterial() const {return ownsPhysicsMaterial;}
	void SetOwnsPhysicsMaterial(bool _ownsPhysicsMaterial) {ownsPhysicsMaterial = _ownsPhysicsMaterial;}
	bool isHollow() const {return hollow;}
	void SetHollow(bool _hollow) {hollow = _hollow;}
	const FCDTransformList& GetTransforms() const {return transforms;}
	void AddTransform(FCDTransform* t) {transforms.push_back(t);}

	// Create a copy of this shape
	FCDPhysicsShape* Clone();

	// Read in the <physics_shape> node of the COLLADA document
	virtual FUStatus LoadFromXML(xmlNode* node);

	// Write out the <physics_shape> node to the COLLADA xml tree
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

#endif // _FCD_PHYSICS_SHAPE_H_
