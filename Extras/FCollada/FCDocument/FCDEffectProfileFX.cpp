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
#include "FCDocument/FCDEffect.h"
#include "FCDocument/FCDEffectCode.h"
#include "FCDocument/FCDEffectProfileFX.h"
#include "FCDocument/FCDEffectParameter.h"
#include "FCDocument/FCDEffectParameterList.h"
#include "FCDocument/FCDEffectParameterFactory.h"
#include "FCDocument/FCDEffectTechnique.h"
#include "FCDocument/FCDLibrary.h"
#include "FCDocument/FCDImage.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDEffectProfileFX::FCDEffectProfileFX(FCDocument* document, FCDEffect* _parent, FUDaeProfileType::Type _type) : FCDEffectProfile(document, _parent)
{
	type = _type;
	parameters = new FCDEffectParameterList(GetDocument(), true);
}

FCDEffectProfileFX::~FCDEffectProfileFX()
{
	CLEAR_POINTER_VECTOR(techniques);
	CLEAR_POINTER_VECTOR(codes);
	SAFE_DELETE(parameters);
}

FCDEffectTechnique* FCDEffectProfileFX::AddTechnique()
{
	FCDEffectTechnique* technique = new FCDEffectTechnique(GetDocument(), this);
	techniques.push_back(technique);
	return technique;
}

void FCDEffectProfileFX::ReleaseTechnique(FCDEffectTechnique* technique)
{
	FCDEffectTechniqueList::iterator it = std::find(techniques.begin(), techniques.end(), technique);
	if (it != techniques.end())
	{
		delete *it;
		techniques.erase(it);
	}
}

void FCDEffectProfileFX::AddParameter(FCDEffectParameter* parameter)
{
	parameters->push_back(parameter);
}

// Look for the parameter with the given reference.
const FCDEffectParameter* FCDEffectProfileFX::FindParameter(const char* ref) const
{
	const FCDEffectParameter* parameter = parameters->FindReference(ref);
	for (FCDEffectTechniqueList::const_iterator it = techniques.begin(); it != techniques.end() && parameter == NULL; ++it)
	{
		parameter = (*it)->FindParameter(ref);
	}
	return parameter;
}

// Look for the effect parameter with the correct semantic, in order to bind/set its value
FCDEffectParameter* FCDEffectProfileFX::FindParameterBySemantic(const string& semantic)
{
	FCDEffectParameter* parameter = parameters->FindSemantic(semantic);
	for (FCDEffectTechniqueList::iterator it = techniques.begin(); it != techniques.end() && parameter == NULL; ++it)
	{
		parameter = (*it)->FindParameterBySemantic(semantic);
	}
	return parameter;
}

void FCDEffectProfileFX::FindParametersBySemantic(const string& semantic, FCDEffectParameterList& _parameters)
{
	// Look in the local parameters
	parameters->FindSemantic(semantic, _parameters);

	// Look in the techniques
	for( FCDEffectTechniqueList::iterator it = techniques.begin(); it != techniques.end(); ++it)
	{
		(*it)->FindParametersBySemantic(semantic, _parameters);
	}
}

void FCDEffectProfileFX::FindParametersByReference(const string& reference, FCDEffectParameterList& _parameters)
{
	// Look in the local parameters
	parameters->FindReference(reference, _parameters);

	// Look in the techniques
	for( FCDEffectTechniqueList::iterator it = techniques.begin(); it != techniques.end(); ++it)
	{
		(*it)->FindParametersBySemantic(reference, _parameters);
	}
}

FCDEffectCode* FCDEffectProfileFX::FindCode(const string& sid)
{
	for (FCDEffectCodeList::iterator itC = codes.begin(); itC != codes.end(); ++itC)
	{
		if ((*itC)->GetSid() == sid) return (*itC);
	}
	return NULL;
}
const FCDEffectCode* FCDEffectProfileFX::FindCode(const string& sid) const
{
	for (FCDEffectCodeList::const_iterator itC = codes.begin(); itC != codes.end(); ++itC)
	{
		if ((*itC)->GetSid() == sid) return (*itC);
	}
	return NULL;
}

// Adds a new code inclusion to this effect profile.
FCDEffectCode* FCDEffectProfileFX::AddCode()
{
	FCDEffectCode* code = new FCDEffectCode(GetDocument());
	codes.push_back(code);
	return code;
}

// Releases a code inclusion contained within this effect profile.
void FCDEffectProfileFX::ReleaseCode(FCDEffectCode* code)
{
	FCDEffectCodeList::iterator itC = std::find(codes.begin(), codes.end(), code);
	if (itC != codes.end())
	{
		delete *itC;
		codes.erase(itC);
	}
}

// Clone the profile effect and its parameters
FCDEffectProfile* FCDEffectProfileFX::Clone(FCDEffect* newParent)
{
	// Hack, because I'm time-bound right now.
	FCDEffectProfileFX* clone = new FCDEffectProfileFX(GetDocument(), newParent, type);
	SAFE_DELETE(clone->parameters);
	clone->parameters = parameters->Clone();
	clone->includeFilename = includeFilename;

	// Clone the codes: needs to happen before the techniques are cloned.
	clone->codes.reserve(codes.size());
	for (FCDEffectCodeList::const_iterator itC = codes.begin(); itC != codes.end(); ++itC)
	{
		clone->codes.push_back((*itC)->Clone());
	}

	// Clone the techniques
	clone->techniques.reserve(techniques.size());
	for (FCDEffectTechniqueList::iterator itPs = techniques.begin(); itPs != techniques.end(); ++itPs)
	{
		clone->techniques.push_back((*itPs)->Clone(clone));
	}
	return clone;
}

// Flatten this effect profile: trickling down all the parameters to the techniques
void FCDEffectProfileFX::Flatten()
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
			// Add this parameter to the techniques
			for (FCDEffectTechniqueList::iterator itT = techniques.begin(); itT != techniques.end(); ++itT)
			{
                (*itT)->AddParameter((*itP)->Clone());
			}
		}
	}
	CLEAR_POINTER_VECTOR(*parameters);

	// Flatten the techniques
	for (FCDEffectTechniqueList::iterator itT = techniques.begin(); itT != techniques.end(); ++itT)
	{
        (*itT)->Flatten();
	}
}

// Read a <profile_X> node for a given COLLADA effect
// Note that this function should do most of the work, except for profile-specific states
FUStatus FCDEffectProfileFX::LoadFromXML(xmlNode* profileNode)
{
	FUStatus status;

	// Verify that we are given a valid XML input node.
	const char* profileName = FUDaeProfileType::ToString(type);
	if (!IsEquivalent(profileNode->name, profileName))
	{
		return status.Warning(FS("Invalid profile input node for effect") + TO_FSTRING(GetDaeId()), profileNode->line);
	}

	// Read in the target platform for this effect profile
	platform = TO_FSTRING(ReadNodeProperty(profileNode, DAE_PLATFORM_ATTRIBUTE));

	// Parse in the child elements: parameters and techniques
	SAFE_DELETE(parameters);
	parameters = new FCDEffectParameterList(GetDocument(), true);

	for (xmlNode* child = profileNode->children; child != NULL; child = child->next)
	{
		if (child->type != XML_ELEMENT_NODE) continue;

		if (IsEquivalent(child->name, DAE_TECHNIQUE_ELEMENT))
		{
			FCDEffectTechnique* technique = AddTechnique();
			status.AppendStatus(technique->LoadFromXML(child, profileNode));
		}
		else if (IsEquivalent(child->name, DAE_FXCMN_NEWPARAM_ELEMENT))
		{
			AddParameter(FCDEffectParameterFactory::LoadFromXML(GetDocument(), child, &status));
		}
		else if (IsEquivalent(child->name, DAE_FXCMN_CODE_ELEMENT) || IsEquivalent(child->name, DAE_FXCMN_INCLUDE_ELEMENT))
		{
			FCDEffectCode* code = AddCode();
			status.AppendStatus(code->LoadFromXML(child));
		}
		else if (IsEquivalent(child->name, DAE_IMAGE_ELEMENT))
		{
			// You can create images within the ColladaFX profile: tell the image library about it.
			FCDImage* image = GetDocument()->GetImageLibrary()->AddEntity();
			status.AppendStatus(image->LoadFromXML(child));
		}
	}
	
	return status;
}

xmlNode* FCDEffectProfileFX::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* profileNode = AddChild(parentNode, FUDaeProfileType::ToString(type));

	// Write out the profile properties/base elements
	if (!platform.empty()) AddAttribute(profileNode, DAE_PLATFORM_ATTRIBUTE, platform);
	if (!includeFilename.empty()) AddChild(profileNode, DAE_FXCMN_INCLUDE_ELEMENT, includeFilename);

	// Write out the code/includes
	for (FCDEffectCodeList::const_iterator itC = codes.begin(); itC != codes.end(); ++itC)
	{
		(*itC)->WriteToXML(profileNode);
	}

	// Write out the parameters
	for (FCDEffectParameterList::const_iterator itP = parameters->begin(); itP != parameters->end(); ++itP)
	{
		(*itP)->WriteToXML(profileNode);
	}

	// Write out the techniques
	for (FCDEffectTechniqueList::const_iterator itT = techniques.begin(); itT != techniques.end(); ++itT)
	{
		(*itT)->WriteToXML(profileNode);
	}

	return profileNode;
}
