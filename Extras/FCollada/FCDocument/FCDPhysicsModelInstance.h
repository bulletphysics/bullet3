/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#ifndef _FCD_PHYSICS_MODEL_ENTITY_H_
#define _FCD_PHYSICS_MODEL_ENTITY_H_

#include "FCDocument/FCDEntityInstance.h"

class FCDocument;

typedef vector<FCDEntityInstance*> FCDEntityInstanceList;

class FCOLLADA_EXPORT FCDPhysicsModelInstance : public FCDEntityInstance
{
private:
	FCDEntityInstanceList instances;

public:
	FCDPhysicsModelInstance(FCDocument* document, FCDEntity* entity);
	virtual ~FCDPhysicsModelInstance();

	FCDEntityInstanceList& GetInstances() {return instances;}

	// FCDEntity override for RTTI-like
	virtual Type GetType() const { return PHYSICS_MODEL; }

	// Load the geometry instance from the COLLADA document
	virtual FUStatus LoadFromXML(xmlNode* instanceNode);

	// Write out the instantiation information to the xml node tree
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

#endif // _FCD_PHYSICS_MODEL_ENTITY_H_
