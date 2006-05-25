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
#include "FCDocument/FCDEffectStandard.h"
#include "FCDocument/FCDEffectProfileFX.h"
#include "FCDocument/FCDEffectParameter.h"
#include "FCDocument/FCDEffectParameterFactory.h"
#include "FCDocument/FCDEffectParameterList.h"
#include "FCDocument/FCDLibrary.h"
#include "FCDocument/FCDImage.h"
#include "FUtils/FUStringConversion.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDEffect::FCDEffect(FCDocument* document) : FCDEntity(document, "Effect")
{
	parameters = new FCDEffectParameterList(GetDocument(), true);
}

FCDEffect::~FCDEffect()
{
	SAFE_DELETE(parameters);
	CLEAR_POINTER_VECTOR(profiles);
}

void FCDEffect::AddParameter(FCDEffectParameter* parameter)
{
	parameters->push_back(parameter);
}

// Flatten this effect: trickling down all the parameters to the technique level
void FCDEffect::Flatten()
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
			// Add this parameter to hierarchies below
			for (FCDEffectProfileList::iterator itR = profiles.begin(); itR != profiles.end(); ++itR)
			{
				if ((*itR)->GetType() != FUDaeProfileType::COMMON)
				{
					((FCDEffectProfileFX*) (*itR))->AddParameter((*itP)->Clone());
				}
			}
		}
	}
	CLEAR_POINTER_VECTOR(*parameters);

	for (FCDEffectProfileList::iterator itR = profiles.begin(); itR != profiles.end(); ++itR)
	{
		(*itR)->Flatten();
	}
}

// Search for a profile of the given type
FCDEffectProfile* FCDEffect::FindProfile(FUDaeProfileType::Type type)
{
	for (FCDEffectProfileList::iterator itR = profiles.begin(); itR != profiles.end(); ++itR)
	{
		if ((*itR)->GetType() == type) return (*itR);
	}
	return NULL;
}

const FCDEffectProfile* FCDEffect::FindProfile(FUDaeProfileType::Type type) const
{
	for (FCDEffectProfileList::const_iterator itR = profiles.begin(); itR != profiles.end(); ++itR)
	{
		if ((*itR)->GetType() == type) return (*itR);
	}
	return NULL;
}

// Create a new effect profile.
FCDEffectProfile* FCDEffect::AddProfile(FUDaeProfileType::Type type)
{
	// Delete any old profile of the same type
	FCDEffectProfile* profile = FindProfile(type);
	if (profile != NULL)
	{
		ReleaseProfile(profile);
		profile = NULL;
	}

	// Create the correct profile for this type.
	if (type == FUDaeProfileType::COMMON) profile = new FCDEffectStandard(GetDocument(), this);
	else profile = new FCDEffectProfileFX(GetDocument(), this, type);

	profiles.push_back(profile);
	return profile;
}

// Releases the effect profile.
void FCDEffect::ReleaseProfile(FCDEffectProfile* profile)
{
	FCDEffectProfileList::iterator itR = std::find(profiles.begin(), profiles.end(), profile);
	if (itR != profiles.end())
	{
		delete *itR;
		profiles.erase(itR);
	}
}

// Look for the effect parameter with the correct semantic, in order to bind/set its value
FCDEffectParameter* FCDEffect::FindParameterBySemantic(const string& semantic)
{
	FCDEffectParameter* p = parameters->FindSemantic(semantic);
	for (FCDEffectProfileList::iterator itR = profiles.begin(); itR != profiles.end() && p == NULL; ++itR)
	{
		p = (*itR)->FindParameterBySemantic(semantic);
	}
	return p;
}

void FCDEffect::FindParametersBySemantic(const string& semantic, FCDEffectParameterList& _parameters)
{
	parameters->FindSemantic(semantic, _parameters);
	for (FCDEffectProfileList::iterator itR = profiles.begin(); itR != profiles.end(); ++itR)
	{
		(*itR)->FindParametersBySemantic(semantic, _parameters);
	}
}

void FCDEffect::FindParametersByReference(const string& reference, FCDEffectParameterList& _parameters)
{
	parameters->FindReference(reference, _parameters);
	for (FCDEffectProfileList::iterator itR = profiles.begin(); itR != profiles.end(); ++itR)
	{
		(*itR)->FindParametersBySemantic(reference, _parameters);
	}
}

// Parse COLLADA document's <effect> element
// Also parses the <material> element for COLLADA 1.3 backward compatibility
FUStatus FCDEffect::LoadFromXML(xmlNode* effectNode)
{
	FUStatus status = FCDEntity::LoadFromXML(effectNode);
	if (!status) return status;

	CLEAR_POINTER_VECTOR(*parameters);

	// COLLADA 1.3 backward compatibility: look for a <material><shader> node and parse this into a standard effect.
	if (IsEquivalent(effectNode->name, DAE_MATERIAL_ELEMENT))
	{
		xmlNode* shaderNode = FindChildByType(effectNode, DAE_SHADER_ELEMENT);
		if (shaderNode != NULL)
		{
			FCDEffectProfile* profile = AddProfile(FUDaeProfileType::COMMON);
			status.AppendStatus(profile->LoadFromXML(shaderNode));
		}
	}
	else
	{
		// Accept solely <effect> elements at this point.
		if (!IsEquivalent(effectNode->name, DAE_EFFECT_ELEMENT))
		{
			return status.Warning(FS("Unknown element in effect library."), effectNode->line);
		}

		for (xmlNode* child = effectNode->children; child != NULL; child = child->next)
		{
			if (child->type != XML_ELEMENT_NODE) continue;

			if (IsEquivalent(child->name, DAE_IMAGE_ELEMENT))
			{
				FCDImage* image = GetDocument()->GetImageLibrary()->AddEntity();
				status.AppendStatus(image->LoadFromXML(child));
				images.push_back(image);
			}
			else if (IsEquivalent(child->name, DAE_FXCMN_SETPARAM_ELEMENT) || IsEquivalent(child->name, DAE_FXCMN_NEWPARAM_ELEMENT))
			{
				AddParameter(FCDEffectParameterFactory::LoadFromXML(GetDocument(), child, &status));
			}
			else if (IsEquivalent(child->name, DAE_EXTRA_ELEMENT))
			{
				// Valid element.. but not processed.
			}
			else
			{
				// Check for a valid profile element.
				FUDaeProfileType::Type type = FUDaeProfileType::FromString((const char*) child->name);
				if (type != FUDaeProfileType::UNKNOWN)
				{
					FCDEffectProfile* profile = AddProfile(type);
					status.AppendStatus(profile->LoadFromXML(child));
				}
				else
				{
					status.Warning(FS("Unsupported profile or unknown element in effect: ") + TO_FSTRING(GetDaeId()), child->line);
				}
			}
		}
	}

	return status;
}

// Returns a copy of the effect, with all the animations/textures attached
FCDEffect* FCDEffect::Clone()
{
	FCDEffect* clone = new FCDEffect(GetDocument());
	FCDEntity::Clone(clone);

	for (FCDEffectProfileList::iterator itR = profiles.begin(); itR != profiles.end(); ++itR)
	{
		FCDEffectProfile* p = (*itR)->Clone(clone);
		clone->profiles.push_back(p);
	}
	
	for (FCDEffectImageList::iterator itI = images.begin(); itI != images.end(); ++itI)
	{
		FCDImage* p = *itI;
		clone->images.push_back(p);
	}

	SAFE_DELETE(clone->parameters);
	clone->parameters = parameters->Clone();
	return clone;
}

// Write out the <material> element to the COLLADA xml tree
xmlNode* FCDEffect::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* effectNode = WriteToEntityXML(parentNode, DAE_EFFECT_ELEMENT);

	// Write out the parameters
	for (FCDEffectParameterList::iterator itP = parameters->begin(); itP != parameters->end(); ++itP)
	{
		(*itP)->WriteToXML(effectNode);
	}

	// Write out the profiles
	for (FCDEffectProfileList::const_iterator itR = profiles.begin(); itR != profiles.end(); ++itR)
	{
		(*itR)->WriteToXML(effectNode);
	}

	FCDEffect::WriteToExtraXML(effectNode);
	return effectNode;
}
