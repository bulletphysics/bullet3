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

#ifndef _FCD_GEOMETRY_ENTITY_H_
#define _FCD_GEOMETRY_ENTITY_H_

#include "FCDocument/FCDEntityInstance.h"

class FCDocument;
class FCDMaterialInstance;

typedef vector<FCDMaterialInstance*> FCDMaterialInstanceList;

class FCOLLADA_EXPORT FCDGeometryInstance : public FCDEntityInstance
{
private:
	FCDMaterialInstanceList materials;

public:
	FCDGeometryInstance(FCDocument* document, FCDEntity* entity);
	virtual ~FCDGeometryInstance();

	// FCDEntity override for RTTI-like
	virtual Type GetType() const { return GEOMETRY; }

	// Access Bound Materials
	FCDMaterialInstance* FindMaterialInstance(const fstring& semantic);
	const FCDMaterialInstance* FindMaterialInstance(const fstring& semantic) const;
	const FCDMaterialInstanceList& GetMaterialInstanceList() const { return materials; }

	// Load the geometry instance from the COLLADA document
	virtual FUStatus LoadFromXML(xmlNode* instanceNode);

	// Write out the instantiation information to the xml node tree
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

#endif // _FCD_GEOMETRY_ENTITY_H_
