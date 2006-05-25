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

#ifndef _FCD_ENTITY_INSTANCE_H_
#define _FCD_ENTITY_INSTANCE_H_

class FCDocument;
class FCDEntity;

#include "FCDocument/FCDObject.h"

class FCOLLADA_EXPORT FCDEntityInstance : public FCDObject
{
public:
	enum Type { SIMPLE, EXTERNAL_REFERENCE, GEOMETRY, MATERIAL, PHYSICS_MODEL, PHYSICS_RIGID_BODY, PHYSICS_RIGID_CONSTRAINT };

protected:
	FCDEntity* entity;

public:
	FCDEntityInstance(FCDocument* document, FCDEntity* _entity = NULL) : FCDObject(document, "FCDEntityInstance") { entity = _entity; }
	virtual ~FCDEntityInstance() { entity = NULL; }

	// Accessors
	FCDEntity* GetEntity() { return entity; }
	const FCDEntity* GetEntity() const { return entity; }
	virtual Type GetType() const { return SIMPLE; }

	// Load the per-instance information from the xml node tree
	virtual FUStatus LoadFromXML(xmlNode* UNUSED(instanceNode)) { return FUStatus(true); }

	// Write out the instantiation information to the xml node tree
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

#endif // _FCD_ENTITY_INSTANCE_H_
