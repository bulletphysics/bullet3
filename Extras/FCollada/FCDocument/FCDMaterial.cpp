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

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDAnimated.h"
#include "FCDocument/FCDEffect.h"
#include "FCDocument/FCDEffectParameter.h"
#include "FCDocument/FCDEffectParameterFactory.h"
#include "FCDocument/FCDEffectParameterList.h"
#include "FCDocument/FCDMaterial.h"
#include "FUtils/FUStringConversion.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
#include "FUtils/FUUniqueStringMap.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDMaterial::FCDMaterial(FCDocument* document) : FCDEntity(document, "VisualMaterial")
{
	effect = NULL;
	parameters = new FCDEffectParameterList(GetDocument(), true);
	ownsEffect = false;
}

FCDMaterial::~FCDMaterial()
{
	if (ownsEffect) SAFE_DELETE(effect);
	effect = NULL;
	SAFE_DELETE(parameters);
	techniqueHints.clear();
}

// Cloning
FCDMaterial* FCDMaterial::Clone()
{
	FCDMaterial* clone = new FCDMaterial(GetDocument());
	FCDEntity::Clone(clone);
	if (effect != NULL)
	{
		clone->ownsEffect = true;
		clone->effect = effect->Clone();
	}
	SAFE_DELETE(clone->parameters);
	clone->parameters = parameters->Clone();
	return clone;
}

#ifdef __VISUALC__
#include <crtdbg.h>
#endif // __VISUALC__

// Flatten the material: remove all the modifier parameters from the parameter list, permanently modifying their base parameter
void FCDMaterial::Flatten()
{
	for (FCDEffectParameterList::iterator itP = parameters->begin(); itP != parameters->end(); ++itP)
	{
		FCDEffectParameterList generators;
		if ((*itP)->IsModifier())
		{
			// Overwrite the generators
			FindParametersByReference((*itP)->GetReference(), generators);
			for (FCDEffectParameterList::iterator itQ = generators.begin(); itQ != generators.end(); ++itQ)
			{
				if ((*itP) != (*itQ))
				{
					(*itP)->Overwrite(*itQ);
				}
			}
		}
		else
		{
			// Add this parameter to hierarchy below
			if (effect != NULL) effect->AddParameter((*itP)->Clone());
		}
	}
	CLEAR_POINTER_VECTOR(*parameters);

	if (effect != NULL) effect->Flatten();
}

void FCDMaterial::AddParameter(FCDEffectParameter* parameter)
{
	parameters->push_back(parameter);
}

// Look for the effect parameter with the correct semantic, in order to bind/set its value
FCDEffectParameter* FCDMaterial::FindParameterBySemantic(const string& semantic)
{
	return (effect != NULL) ? effect->FindParameterBySemantic(semantic) : NULL;
}

void FCDMaterial::FindParametersBySemantic(const string& semantic, FCDEffectParameterList& _parameters)
{
	parameters->FindSemantic(semantic, _parameters);
	if (effect != NULL) effect->FindParametersBySemantic(semantic, _parameters);
}

void FCDMaterial::FindParametersByReference(const string& reference, FCDEffectParameterList& _parameters)
{
	parameters->FindReference(reference, _parameters);
	if (effect != NULL) effect->FindParametersByReference(reference, _parameters);
}

// Parse COLLADA document's <material> element
FUStatus FCDMaterial::LoadFromXML(xmlNode* materialNode)
{
	CLEAR_POINTER_VECTOR(*parameters);

	FUStatus status = FCDEntity::LoadFromXML(materialNode);
	if (!status) return status;
	if (!IsEquivalent(materialNode->name, DAE_MATERIAL_ELEMENT))
	{
		return status.Warning(FS("Unknown element in material library."), materialNode->line);
	}

	// Read in the effect pointer node
	xmlNode* effectNode = FindChildByType(materialNode, DAE_INSTANCE_EFFECT_ELEMENT);
	if (effectNode != NULL)
	{
		FUUri url = ReadNodeUrl(effectNode);
		if (!url.prefix.empty())
		{
			return status.Warning(FS("Externally referenced effects are not supported. Material: ") + TO_FSTRING(GetDaeId()), effectNode->line);
		}
		else if (url.suffix.empty())
		{
			return status.Warning(FS("Empty material's <instance_effect> definition. Should instantiate an effect from the effect's library. Material: ") + TO_FSTRING(GetDaeId()), effectNode->line);
		}
		effect = GetDocument()->FindEffect(url.suffix);

		// Read in the parameter modifications
		for (xmlNode* child = effectNode->children; child != NULL; child = child->next)
		{
			if (child->type != XML_ELEMENT_NODE) continue;
			
			if (IsEquivalent(child->name, DAE_FXCMN_SETPARAM_ELEMENT))
			{
				AddParameter(FCDEffectParameterFactory::LoadFromXML(GetDocument(), child, &status));
			}
			else if (IsEquivalent(child->name, DAE_FXCMN_HINT_ELEMENT))
			{
				FCDMaterialTechniqueHint& hint = *(techniqueHints.insert(techniqueHints.end(), FCDMaterialTechniqueHint()));
				hint.platform = TO_FSTRING(ReadNodeProperty(child, DAE_PLATFORM_ATTRIBUTE));
				hint.technique = ReadNodeProperty(child, DAE_REF_ATTRIBUTE);
			}
		}
	}
	else
	{
		// COLLADA 1.3 backward compatibility: look for the effect.
		if(effect != NULL)
		{
			//kind of a hack: swap the ids between the material and the effect and append
			//some weird extension to the effect id so that it doesn't conflict with anybody else.
			effect->RemoveDaeId();
			status = FCDEntity::LoadFromXML(materialNode);
			if (!status) return status;
			effect->SetDaeId(GetDaeId() + "_effect1.3");
		}
	}

	if (effect == NULL)
	{
		return status.Warning(FS("Unable to find effect for material: ") + TO_FSTRING(GetDaeId()), materialNode->line);
	}
	
	return status;
}

// Write out the <material> element to the COLLADA xml tree
xmlNode* FCDMaterial::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* materialNode = WriteToEntityXML(parentNode, DAE_MATERIAL_ELEMENT);

	// The <instance_effect> element is required in COLLADA 1.4
	xmlNode* instanceEffectNode = AddChild(materialNode, DAE_INSTANCE_EFFECT_ELEMENT);
	if (effect != NULL)
	{
		AddAttribute(instanceEffectNode, DAE_URL_ATTRIBUTE, string("#") + effect->GetDaeId());

		// Write out the technique hints
		for (FCDMaterialTechniqueHintList::const_iterator itH = techniqueHints.begin(); itH != techniqueHints.end(); ++itH)
		{
			xmlNode* hintNode = AddChild(instanceEffectNode, DAE_FXCMN_HINT_ELEMENT);
			AddAttribute(hintNode, DAE_PLATFORM_ATTRIBUTE, (*itH).platform);
			AddAttribute(hintNode, DAE_REF_ATTRIBUTE, (*itH).technique);
		}

		// Write out the parameters
		for (FCDEffectParameterList::const_iterator itP = parameters->begin(); itP != parameters->end(); ++itP)
		{
			(*itP)->WriteToXML(instanceEffectNode);
		}
	}
	else
	{
		AddAttribute(instanceEffectNode, DAE_URL_ATTRIBUTE, string("#"));
	}

	FCDEntity::WriteToExtraXML(materialNode);
	return materialNode;
}
