/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDEffectCode.h"
#include "FCDocument/FCDEffectPass.h"
#include "FCDocument/FCDEffectProfileFX.h"
#include "FCDocument/FCDEffectParameter.h"
#include "FCDocument/FCDEffectParameterFactory.h"
#include "FCDocument/FCDEffectParameterList.h"
#include "FCDocument/FCDEffectTechnique.h"
#include "FCDocument/FCDLibrary.h"
#include "FCDocument/FCDImage.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDEffectTechnique::FCDEffectTechnique(FCDocument* document, FCDEffectProfileFX *_parent) : FCDObject(document, "FCDEffectTechnique")
{
	parent = _parent;
	parameters = new FCDEffectParameterList(GetDocument(), true);;
}

FCDEffectTechnique::~FCDEffectTechnique()
{
	CLEAR_POINTER_VECTOR(codes);
	CLEAR_POINTER_VECTOR(passes);
	SAFE_DELETE(parameters);
	parent = NULL;
}

// Adds a new pass to this effect technique.
FCDEffectPass* FCDEffectTechnique::AddPass()
{
	FCDEffectPass* pass = new FCDEffectPass(GetDocument(), this);
	passes.push_back(pass);
	return pass;
}

// Releases a pass contaied within this effect technique.
void FCDEffectTechnique::ReleasePass(FCDEffectPass* pass)
{
	FCDEffectPassList::iterator it = std::find(passes.begin(), passes.end(), pass);
	if (it != passes.end())
	{
		delete *it;
		passes.erase(it);
	}
}

// Adds a new code inclusion to this effect profile.
FCDEffectCode* FCDEffectTechnique::AddCode()
{
	FCDEffectCode* code = new FCDEffectCode(GetDocument());
	codes.push_back(code);
	return code;
}

// Releases a code inclusion contained within this effect profile.
void FCDEffectTechnique::ReleaseCode(FCDEffectCode* code)
{
	FCDEffectCodeList::iterator itC = std::find(codes.begin(), codes.end(), code);
	if (itC != codes.end())
	{
		delete *itC;
		codes.erase(itC);
	}
}

FCDEffectTechnique* FCDEffectTechnique::Clone(FCDEffectProfileFX* newParent)
{
	FCDEffectTechnique* clone = new FCDEffectTechnique(GetDocument(), newParent);
	clone->name = name;
	SAFE_DELETE(clone->parameters);
	clone->parameters = parameters->Clone();

	// Clone the codes: need to happen before the passes are cloned
	clone->codes.reserve(codes.size());
	for (FCDEffectCodeList::const_iterator itC = codes.begin(); itC != codes.end(); ++itC)
	{
		clone->codes.push_back((*itC)->Clone());
	}

	// Clone the passes
	for (FCDEffectPassList::iterator itP = passes.begin(); itP != passes.end(); ++itP)
	{
		clone->passes.push_back((*itP)->Clone(clone));
	}
	return clone;
}

const string& FCDEffectTechnique::GetDaeId() const
{
	return parent->GetDaeId();
}

void FCDEffectTechnique::AddParameter(FCDEffectParameter* parameter)
{
	parameters->push_back(parameter);
}

// Flatten this effect technique: merge the parameter modifiers and generators
void FCDEffectTechnique::Flatten()
{
	for (FCDEffectParameterList::iterator itP = parameters->begin(); itP != parameters->end();)
	{
		FCDEffectParameterList generators(GetDocument());
		if ((*itP)->IsModifier())
		{
			// Overwrite the generators
			FindParametersByReference((*itP)->GetReference(), generators);
			for (FCDEffectParameterList::iterator itQ = generators.begin(); itQ != generators.end(); ++itQ)
			{
				if ((*itQ)->IsGenerator())
				{
					(*itP)->Overwrite(*itQ);
				}
			}
			SAFE_DELETE(*itP);
			parameters->erase(itP);
		}
		else
		{
			++itP;
		}
	}
}

FUStatus FCDEffectTechnique::LoadFromXML(xmlNode* techniqueNode, xmlNode* profileNode)
{
	FUStatus status;
	if (!IsEquivalent(techniqueNode->name, DAE_TECHNIQUE_ELEMENT))
	{
		return status.Warning(FS("Technique contains unknown element."), techniqueNode->line);
	}
	
	string techniqueName = ReadNodeProperty(techniqueNode, DAE_SID_ATTRIBUTE);
	name = TO_FSTRING(techniqueName);
	
	// Look for the pass and parameter elements
	SAFE_DELETE(parameters);
	parameters = new FCDEffectParameterList(GetDocument(), true);
	for (xmlNode* child = techniqueNode->children; child != NULL; child = child->next)
	{
		if (child->type != XML_ELEMENT_NODE) continue;

		if (IsEquivalent(child->name, DAE_PASS_ELEMENT))
		{
			FCDEffectPass* pass = AddPass();
			status.AppendStatus(pass->LoadFromXML(child, techniqueNode, profileNode));
		}
		else if (IsEquivalent(child->name, DAE_FXCMN_NEWPARAM_ELEMENT) || IsEquivalent(child->name, DAE_FXCMN_SETPARAM_ELEMENT))
		{
			AddParameter(FCDEffectParameterFactory::LoadFromXML(GetDocument(), child, &status));
		}
		else if (IsEquivalent(child->name, DAE_FXCMN_CODE_ELEMENT) || IsEquivalent(child->name, DAE_FXCMN_INCLUDE_ELEMENT))
		{
			FCDEffectCode* code = new FCDEffectCode(GetDocument());
			codes.push_back(code);
			status.AppendStatus(code->LoadFromXML(child));
		}
		else if (IsEquivalent(child->name, DAE_IMAGE_ELEMENT))
		{
			FCDImage* image = GetDocument()->GetImageLibrary()->AddEntity();
			status.AppendStatus(image->LoadFromXML(child));
		}
	}
	
	return status;
}

// Write out the effect techniques to the COLLADA xml node tree
xmlNode* FCDEffectTechnique::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* techniqueNode = AddChild(parentNode, DAE_TECHNIQUE_ELEMENT);
	const_cast<FCDEffectTechnique*>(this)->name = TO_FSTRING(AddNodeSid(techniqueNode, !name.empty() ? TO_STRING(name).c_str() : "common"));

	// Write out the code/includes
	for (FCDEffectCodeList::const_iterator itC = codes.begin(); itC != codes.end(); ++itC)
	{
		(*itC)->WriteToXML(techniqueNode);
	}

	// Write out the effect parameters at this level
	for (FCDEffectParameterList::const_iterator itP = parameters->begin(); itP != parameters->end(); ++itP)
	{
		(*itP)->WriteToXML(techniqueNode);
	}

	// Write out the passes.
	// In COLLADA 1.4: there should always be at least one pass.
	if (!passes.empty())
	{
		for (FCDEffectPassList::const_iterator itP = passes.begin(); itP != passes.end(); ++itP)
		{
			(*itP)->WriteToXML(techniqueNode);
		}
	}
	else
	{
		UNUSED(xmlNode* dummyPassNode =) AddChild(techniqueNode, DAE_PASS_ELEMENT);
	}

	return techniqueNode;
}

// Look for the parameter with the given reference.
const FCDEffectParameter* FCDEffectTechnique::FindParameter(const char* ref) const
{
	return parameters->FindReference(ref);
}

// Look for the effect parameter with the correct semantic, in order to bind/set its value
FCDEffectParameter* FCDEffectTechnique::FindParameterBySemantic(const string& semantic)
{
	return parameters->FindSemantic(semantic);
}

void FCDEffectTechnique::FindParametersBySemantic(const string& semantic, FCDEffectParameterList& _parameters)
{
	for (FCDEffectParameterList::iterator it = parameters->begin(); it != parameters->end(); ++it)
	{
		if ((*it)->GetSemantic() == semantic) _parameters.push_back(*it);
	}
}

void FCDEffectTechnique::FindParametersByReference(const string& reference, FCDEffectParameterList& _parameters)
{
	for (FCDEffectParameterList::iterator it = parameters->begin(); it != parameters->end(); ++it)
	{
		if ((*it)->GetReference() == reference) _parameters.push_back(*it);
	}
}

FCDEffectCode* FCDEffectTechnique::FindCode(const string& sid)
{
	for (FCDEffectCodeList::iterator itC = codes.begin(); itC != codes.end(); ++itC)
	{
		if ((*itC)->GetSid() == sid) return (*itC);
	}
	return NULL;
}
const FCDEffectCode* FCDEffectTechnique::FindCode(const string& sid) const
{
	for (FCDEffectCodeList::const_iterator itC = codes.begin(); itC != codes.end(); ++itC)
	{
		if ((*itC)->GetSid() == sid) return (*itC);
	}
	return NULL;
}
