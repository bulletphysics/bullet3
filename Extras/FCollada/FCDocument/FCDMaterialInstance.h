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

#ifndef _FCD_MATERIAL_BIND_H_
#define	_FCD_MATERIAL_BIND_H_

#include "FCDocument/FCDEntityInstance.h"

class FCDocument;
class FCDGeometryInstance;

class FCOLLADA_EXPORT FCDMaterialInstanceBind
{
public:
	string semantic;
	string target;
};

typedef vector<FCDMaterialInstanceBind> FCDMaterialInstanceBindList;

class FCOLLADA_EXPORT FCDMaterialInstance : public FCDEntityInstance
{
private:
	FCDGeometryInstance* parent;
	fstring semantic;
	FCDMaterial* material;
	FCDMaterialInstanceBindList bindings;

public:
	FCDMaterialInstance(FCDocument* document, FCDGeometryInstance* parent);
	virtual ~FCDMaterialInstance();

	// Accessors
	virtual Type GetType() const { return MATERIAL; }
	const fstring& GetSemantic() const { return semantic; }
	FCDMaterial* GetMaterial() { return material; }
	const FCDMaterial* GetMaterial() const { return material; }
	FCDMaterialInstanceBindList& GetBindings() { return bindings; }
	const FCDMaterialInstanceBindList& GetBindings() const { return bindings; }

	// Create a flattened version of the instantiated material: this is the
	// prefered way to generate viewer materials from a COLLADA document
	FCDMaterial* FlattenMaterial();

	// Read in the materal instantiation from the COLLADA document
	virtual FUStatus LoadFromXML(xmlNode* instanceNode);
	FUStatus LoadFromId(const string& materialId); // COLLADA 1.3 backward compatibility

	// Write out the instantiation information to the xml node tree
	xmlNode* WriteToXML(xmlNode* parentNode) const;
};

#endif // _FCD_MATERIAL_BIND_H_
