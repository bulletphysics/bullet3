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
	@file FCDMaterialLibrary.h
	This file contains the FCDMaterialLibrary.h. 
*/

#ifndef _FCD_MATERIAL_LIBRARY_H_
#define _FCD_MATERIAL_LIBRARY_H_

class FCDocument;
class FCDTexture;
class FCDEffect;
class FCDMaterial;

typedef vector<FCDTexture*> FCDTextureList; /**< A dynamically-sized list of textures. */
typedef vector<FCDEffect*> FCDEffectList; /**< A dynamically-sized list of effects. */
typedef vector<FCDMaterial*> FCDMaterialList; /**< A dynamically-sized list of materials. */

#include "FCDocument/FCDLibrary.h"

/**
	The shared COLLADA material and effect libraries.
	This class covers the material and effect libraries, as well as the
	texture library for COLLADA 1.3 backward compatibility.

	@todo When information push is fully implemented: split the effect library out of this one.

	@ingroup FCDocument
*/
class FCOLLADA_EXPORT FCDMaterialLibrary : public FCDLibrary<FCDEntity>
{
private:
	FCDTextureList textures;
	FCDEffectList effects;
	FCDMaterialList materials;

public:
	/** Constructor: do not use directly.
		The document object will create the one and only object of this class.
		@param document The COLLADA document that owns this library. */
	FCDMaterialLibrary(FCDocument* document);

	/** Destructor: do not use directly.
		The document object will release its libraries. */
	virtual ~FCDMaterialLibrary();

	/** Retrieves the list of effects contained by this library.
		@return The list of effects. */
	FCDEffectList& GetEffects() { return effects; }
	const FCDEffectList& GetEffects() const { return effects; } /**< See above. */

	/** Retrieves the number of effects contained by this library.
		@return The number of effects within the library. */
	size_t GetEffectCount() const { return effects.size(); }

	/** Retrieves an effect contained by this library.
		@param index The index of the effect.
		@return The given effect. This pointer will be NULL if no effect matches the index. */
	FCDEffect* GetEffect(size_t index) { FUAssert(index < GetEffectCount(), return NULL); return effects.at(index); }
	const FCDEffect* GetEffect(size_t index) const { FUAssert(index < GetEffectCount(), return NULL); return effects.at(index); } /**< See above. */

	/** [INTERNAL] Retrieves an effect contained by this library.
		@param daeId The COLLADA id of the effect.
		@return The matching effect. This pointer will be NULL if no effect matches the COLLADA id. */
	FCDEffect* FindEffect(const string& daeId);

	/** Creates a new effect.
		@return The newly created effect. */
	FCDEffect* AddEffect();

	/** Releases an effect.
		@todo Implement FCDMaterialLibrary::ReleaseEffect.
		@param effect The effect to release. */
	void ReleaseEffect(FCDEffect* effect);

	/** Retrieves the list of materials contained by this library.
		@return The list of materials. */
	FCDMaterialList& GetMaterials() { return materials; }
	const FCDMaterialList& GetMaterials() const { return materials; } /**< See above. */

	/** Retrieves the number of materials contained by this library.
		@return The number of materials within the library. */
	size_t GetMaterialCount() const { return materials.size(); }

	/** Retrieves a material contained by this library.
		@param index The index of the material.
		@return The given material. This pointer will be NULL if no material matches the index. */
	FCDMaterial* GetMaterial(size_t index) { FUAssert(index < GetMaterialCount(), return NULL); return materials.at(index); }
	const FCDMaterial* GetMaterial(size_t index) const { FUAssert(index < GetMaterialCount(), return NULL); return materials.at(index); } /**< See above. */

	/** [INTERNAL] Retrieves a material contained by this library.
		@param daeId The COLLADA id of the material.
		@return The matching material. This pointer will be NULL
			if no material matches the COLLADA id. */
	FCDMaterial* FindMaterial(const string& daeId);

	/** [INTERNAL] Retrieves a texture contained by this library.
		@param daeId The COLLADA id of the texture.
		@return The matching texture. This pointer will be NULL
			if no texture matches the COLLADA id. */
	FCDTexture* FindTexture(const string& daeId);
	
	/** Creates a new material.
		@return The newly created material. */
	FCDMaterial* AddMaterial();

	/** Releases a material.
		@todo Implement FCDMaterialLibrary::ReleaseMaterial.
		@param material The material to release. */
	void ReleaseMaterial(FCDMaterial* material);

	/** [INTERNAL] Reads in the contents of the library from the COLLADA XML document.
		This method will be called once for the effect library and once for the material library.
		It may also be called once, for COLLADA 1.3 backward compatibility, for the texture library.
		@param node The COLLADA XML tree node to parse into entities.
		@return The status of the import. If the status is not successful, it may be dangerous to
			extract information from the library. */
	virtual FUStatus LoadFromXML(xmlNode* node);

	/** [INTERNAL] Writes out the library entities to the COLLADA XML document.
		This method writes out the material library within the given node and
		writes out the effect library as a sibling node to the given node.
		@param libraryNode The COLLADA XML tree node in which to write the materials. */
	virtual void WriteToXML(xmlNode* libraryNode) const;
};

#endif // _FCD_MATERIAL_LIBRARY_H_

