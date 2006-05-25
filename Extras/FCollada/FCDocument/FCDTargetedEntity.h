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

/**
	@file FCDTargetedEntity.h
	This file contains the FCDTargetedEntity class.
*/

#ifndef _FCD_TARGETED_ENTITY_H_
#define _FCD_TARGETED_ENTITY_H_

#include "FCDocument/FCDEntity.h"

class FCDSceneNode;

/**
	A targeted entity.

	COLLADA doesn't have targeted entity.
	Therefore: the behavior of a targeted entity is application-defined.
	
	This class is used to support 3dsMax's targeted cameras and lights
	and we also plan to support Maya's targeted cameras and lights.

	@see FCDCamera FCDLight
	@ingroup FCDEntity
*/
class FCOLLADA_EXPORT FCDTargetedEntity : public FCDEntity
{
private:
	// Target
	string targetId; // only valid during the import
	FCDSceneNode* targetNode;

public:
	/** Constructor: do not use directly.
		Instead, create objects of the up-classes.
		@param document The COLLADA document that owns the targeted entity.
		@param baseId The prefix COLLADA id to be used if no COLLADA id is provided. */
	FCDTargetedEntity(FCDocument* document, const char* baseId);

	/** Destructor: do not use directly.
		Instead, release objects through their libraries or their parent entities. */
	virtual ~FCDTargetedEntity();

	/** Retrieves whether a target is defined for this entity.
		@return Whether a target is defined for this entity. */
	inline bool HasTarget() const { return targetNode != NULL; }

	/** Retrieves the target visual scene node for this entity.
		@return The target visual scene node. */
	inline FCDSceneNode* GetTargetNode() { return targetNode; }
	inline const FCDSceneNode* GetTargetNode() const { return targetNode; } /**< See above. */

	/** Sets the target visual scene node for this entity.
		@param target The new target node. */
	void SetTargetNode(FCDSceneNode* target);

	/** [INTERNAL] Links the entity with its target.
		This function is used during the import of a COLLADA document.
		@todo Modify this function to support multiple visual scenes.
		@param sceneRoot The root visual scene.
		@return The status of the linkage. */
	FUStatus LinkTarget(FCDSceneNode* sceneRoot);

protected:
	/** [INTERNAL] Retrieves the COLLADA id of the target entity.
		The actual resolution of the COLLADA id into a visual scene node happens
		during the LinkTarget function. 
		@return The COLLADA id of the target entity. */
	inline const string& GetTargetId() { return targetId; }

	/** [INTERNAL] Sets the COLLADA id of the target entity.
		This function is used during the import of a COLLADA document.
		The actual resolution of the COLLADA id into a visual scene node happens
		during the LinkTarget function. 
		@param _targetId A COLLADA id. */
	inline void SetTargetId(const string& _targetId) { targetId = _targetId; }
	inline void SetTargetId(const char* _targetId) { targetId = _targetId; } /**< See above. */
};

#endif // _FCD_TARGETED_ENTITY_H_

