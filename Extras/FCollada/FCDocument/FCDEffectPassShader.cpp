/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDEffectCode.h"
#include "FCDocument/FCDEffectPass.h"
#include "FCDocument/FCDEffectPassShader.h"
#include "FCDocument/FCDEffectProfileFX.h"
#include "FCDocument/FCDEffectTechnique.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDEffectPassShader::FCDEffectPassShader(FCDocument* document, FCDEffectPass* _parent) : FCDObject(document, "FCDEffectPassShader")
{
	parent = _parent;
	isFragment = false;
	code = NULL;
}

FCDEffectPassShader::~FCDEffectPassShader()
{
	parent = NULL;
	code = NULL;
}


// Adds a new binding to this shader.
FCDEffectPassBind* FCDEffectPassShader::AddBinding()
{
	bindings.push_back(FCDEffectPassBind());
	return &bindings.back();
}

// Releases a binding contained within this shader.
void FCDEffectPassShader::ReleaseBinding(FCDEffectPassBind* binding)
{
	for (FCDEffectPassBindList::iterator it = bindings.begin(); it != bindings.end(); ++it)
	{
		if (&(*it) == binding)
		{
			bindings.erase(it);
			break;
		}
	}
}

// Cloning
FCDEffectPassShader* FCDEffectPassShader::Clone(FCDEffectPass* newParent) const
{
	FCDEffectPassShader* clone = new FCDEffectPassShader(GetDocument(), newParent);
	clone->isFragment = isFragment;
	clone->bindings = bindings;
	clone->compilerTarget = compilerTarget;
	clone->compilerOptions = compilerOptions;
	clone->name = name;

	// Look for the new code within the parent.
	if (code != NULL)
	{
		clone->code = newParent->GetParent()->FindCode(code->GetSid());
		if (clone->code == NULL) clone->code = newParent->GetParent()->GetParent()->FindCode(code->GetSid());
	}
	return clone;
}

// Read in the ColladaFX pass shader from the xml node tree
FUStatus FCDEffectPassShader::LoadFromXML(xmlNode* shaderNode)
{
	FUStatus status;
	if (!IsEquivalent(shaderNode->name, DAE_SHADER_ELEMENT))
	{
		return status.Warning(FS("Pass shader contains unknown element."), shaderNode->line);
	}

	// Read in the shader's name and stage
	xmlNode* nameNode = FindChildByType(shaderNode, DAE_FXCMN_NAME_ELEMENT);
	name = ReadNodeContentDirect(nameNode);
	string codeSource = ReadNodeProperty(nameNode, DAE_SOURCE_ATTRIBUTE);
	if (name.empty())
	{
		return status.Warning(FS("Unnamed effect pass shader found."), shaderNode->line);
	}
	string stage = ReadNodeStage(shaderNode);
	isFragment = stage == DAE_FXCMN_FRAGMENT_SHADER;
	if (!isFragment && stage != DAE_FXCMN_VERTEX_SHADER)
	{
		return status.Warning(FS("Unknown stage for effect pass shader: ") + TO_FSTRING(name), shaderNode->line);
	}

	// Look-up the code filename for this shader, if available
	code = parent->GetParent()->FindCode(codeSource);
	if (code == NULL) code = parent->GetParent()->GetParent()->FindCode(codeSource);

	// Read in the compiler-related elements
	xmlNode* compilerTargetNode = FindChildByType(shaderNode, DAE_FXCMN_COMPILERTARGET_ELEMENT);
	compilerTarget = TO_FSTRING(ReadNodeContentDirect(compilerTargetNode));
	xmlNode* compilerOptionsNode = FindChildByType(shaderNode, DAE_FXCMN_COMPILEROPTIONS_ELEMENT);
	compilerOptions = TO_FSTRING(ReadNodeContentDirect(compilerOptionsNode));

	// Read in the bind parameters
	xmlNodeList bindNodes;
	FindChildrenByType(shaderNode, DAE_FXCMN_BIND_ELEMENT, bindNodes);
	for (xmlNodeList::iterator itB = bindNodes.begin(); itB != bindNodes.end(); ++itB)
	{
		xmlNode* paramNode = FindChildByType(*itB, DAE_PARAMETER_ELEMENT);

		FCDEffectPassBind& bind = *(bindings.insert(bindings.end(), FCDEffectPassBind()));
		bind.symbol = ReadNodeProperty((*itB), DAE_SYMBOL_ATTRIBUTE);
		bind.reference = ReadNodeProperty(paramNode, DAE_REF_ATTRIBUTE);
	}

	return status;
}

// Write out the pass shader to the COLLADA xml node tree
xmlNode* FCDEffectPassShader::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* shaderNode = AddChild(parentNode, DAE_SHADER_ELEMENT);

	// Write out the compiler information and the shader's name/stage
	if (!compilerTarget.empty()) AddChild(shaderNode, DAE_FXCMN_COMPILERTARGET_ELEMENT, compilerTarget);
	if (!compilerOptions.empty()) AddChild(shaderNode, DAE_FXCMN_COMPILEROPTIONS_ELEMENT, compilerOptions);
	AddAttribute(shaderNode, DAE_STAGE_ATTRIBUTE, isFragment ? DAE_FXCMN_FRAGMENT_SHADER : DAE_FXCMN_VERTEX_SHADER);
	if (!name.empty())
	{
		xmlNode* nameNode = AddChild(shaderNode, DAE_FXCMN_NAME_ELEMENT, name);
		if (code != NULL) AddAttribute(nameNode, DAE_SOURCE_ATTRIBUTE, code->GetSid());
	}

	// Write out the bindings
	for (FCDEffectPassBindList::const_iterator itB = bindings.begin(); itB != bindings.end(); ++itB)
	{
		const FCDEffectPassBind& b = (*itB);
		if (!b.reference.empty() && !b.symbol.empty())
		{
			xmlNode* bindNode = AddChild(shaderNode, DAE_BIND_ELEMENT);
			AddAttribute(bindNode, DAE_SYMBOL_ATTRIBUTE, b.symbol);
			xmlNode* paramNode = AddChild(bindNode, DAE_PARAMETER_ELEMENT);
			AddAttribute(paramNode, DAE_REF_ATTRIBUTE, b.reference);
		}
	}
	return shaderNode;
}
