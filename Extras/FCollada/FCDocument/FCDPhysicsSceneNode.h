/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/
/*
	Based on the FS Import classes:
	Copyright (C) 2005-2006 Feeling Software Inc
	Copyright (C) 2005-2006 Autodesk Media Entertainment
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#ifndef _FCD_PHYSICS_SCENE_NODE_
#define _FCD_PHYSICS_SCENE_NODE_

#include "FCDocument/FCDEntity.h"

class FCDocument;
class FCDEntityInstance;
class FCDExtra;

class FCDPhysicsModelInstance;

typedef vector<FCDPhysicsModelInstance*> FCDPhysicsModelInstanceList;

class FCOLLADA_EXPORT FCDPhysicsSceneNode : public FCDEntity
{
private:
	FCDExtra* extra;
	FMVector3 gravity;
	float timestep;
	FCDPhysicsModelInstanceList instances;

public:
	FCDPhysicsSceneNode(FCDocument* document);
	virtual ~FCDPhysicsSceneNode();

	// Returns the entity type
	virtual Type GetType() const { return PHYSICS_SCENE_NODE; }
	
	FCDPhysicsModelInstanceList& GetInstances() { return instances; }
	const FCDPhysicsModelInstanceList& GetInstances() const { return instances; }
	size_t GetNumInstances() const { return instances.size(); };

	// Visibility parameter
	const FMVector3& GetGravity() const { return gravity; }
	const float& GetTimestep() const { return timestep; }

	// Parse a <physics_scene> node from a COLLADA document
	virtual FUStatus LoadFromXML(xmlNode* sceneNode);

	// Write out a <physics_scene> element to a COLLADA xml document
	void WriteToNodeXML(xmlNode* node) const;
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;

	// Instantiating a scene node means ensuring that there is no cycles in the scene graph
	void Instantiate(FCDPhysicsSceneNode* sceneNode);
};

#endif // _FCD_SCENE_NODE_
