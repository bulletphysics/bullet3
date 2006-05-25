/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDEffectTechnique.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDEffectPass.h"
#include "FCDocument/FCDEffectPassShader.h"
#include "FCDocument/FCDEffectParameter.h"
#include "FCDocument/FCDEffectParameterList.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDEffectPass::FCDEffectPass(FCDocument* document, FCDEffectTechnique *_parent) : FCDObject(document, "FCDEffectPass")
{
	parent = _parent;
}

FCDEffectPass::~FCDEffectPass()
{
	CLEAR_POINTER_VECTOR(shaders);
	meshdata.clear();
	parent = NULL;
}

// Adds a new shader to the pass.
FCDEffectPassShader* FCDEffectPass::AddShader()
{
	FCDEffectPassShader* shader = new FCDEffectPassShader(GetDocument(), this);
	shaders.push_back(shader);
	return shader;
}

// Releases a shader contained within the pass.
void FCDEffectPass::ReleaseShader(FCDEffectPassShader* shader)
{
	FCDEffectPassShaderList::iterator it = std::find(shaders.begin(), shaders.end(), shader);
	if (it != shaders.end())
	{
		delete *it;
		shaders.push_back(shader);
	}
}

// Adds a new vertex shader to the pass.
// If a vertex shader already exists within the pass, it will be released.
FCDEffectPassShader* FCDEffectPass::AddVertexShader()
{
	FCDEffectPassShader* vertexShader;
	for (vertexShader = GetVertexShader(); vertexShader != NULL; vertexShader = GetVertexShader())
	{
		ReleaseShader(vertexShader);
	}

	vertexShader = AddShader();
	vertexShader->AffectsVertices();
	return vertexShader;
}

// Adds a new fragment shader to the pass.
// If a fragment shader already exists within the pass, it will be released.
FCDEffectPassShader* FCDEffectPass::AddFragmentShader()
{
	FCDEffectPassShader* fragmentShader;
	for (fragmentShader = GetFragmentShader(); fragmentShader != NULL; fragmentShader = GetFragmentShader())
	{
		ReleaseShader(fragmentShader);
	}

	fragmentShader = AddShader();
	fragmentShader->AffectsFragments();
	return fragmentShader;
}

FCDEffectPass* FCDEffectPass::Clone(FCDEffectTechnique* newParent) const
{
	FCDEffectPass* clone = new FCDEffectPass(GetDocument(), newParent);
	clone->name = name;
	clone->meshdata = meshdata;

	// Clone the shaderss
	clone->shaders.reserve(shaders.size());
	for (FCDEffectPassShaderList::const_iterator itS = shaders.begin(); itS != shaders.end(); ++itS)
	{
		clone->shaders.push_back((*itS)->Clone(clone));
	}

	return clone;
}

const string& FCDEffectPass::GetDaeId() const
{
	return parent->GetDaeId();
}

// Retrieve the type-specific shaders
FCDEffectPassShader* FCDEffectPass::GetVertexShader()
{
	for (FCDEffectPassShaderList::iterator itS = shaders.begin(); itS != shaders.end(); ++itS)
	{
		if ((*itS)->IsVertexShader()) return (*itS);
	}
	return NULL;
}
const FCDEffectPassShader* FCDEffectPass::GetVertexShader() const
{
	for (FCDEffectPassShaderList::const_iterator itS = shaders.begin(); itS != shaders.end(); ++itS)
	{
		if ((*itS)->IsVertexShader()) return (*itS);
	}
	return NULL;
}

FCDEffectPassShader* FCDEffectPass::GetFragmentShader()
{
	for (FCDEffectPassShaderList::iterator itS = shaders.begin(); itS != shaders.end(); ++itS)
	{
		if ((*itS)->IsFragmentShader()) return (*itS);
	}
	return NULL;
}
const FCDEffectPassShader* FCDEffectPass::GetFragmentShader() const
{
	for (FCDEffectPassShaderList::const_iterator itS = shaders.begin(); itS != shaders.end(); ++itS)
	{
		if ((*itS)->IsFragmentShader()) return (*itS);
	}
	return NULL;
}

FUStatus FCDEffectPass::LoadFromXML(xmlNode* passNode, xmlNode* techniqueNode, xmlNode* profileNode)
{
	FUStatus status;
	if (!IsEquivalent(passNode->name, DAE_PASS_ELEMENT))
	{
		return status.Warning(FS("Pass contains unknown element."), passNode->line);
	}
	name = TO_FSTRING(ReadNodeProperty(passNode, DAE_SID_ATTRIBUTE));

	// parse mesh data
	const char* smdnames[10] = { "COLOR0", "COLOR1", "TEXCOORD0", "TEXCOORD1", "TEXCOORD2", "TEXCOORD3", "TEXCOORD4", "TEXCOORD5", "TEXCOORD6", "TEXCOORD7" };
	for(uint32 i = 0; i < 10; ++i)
	{
		string smdname = smdnames[i];

		string smdata = "";
		xmlNode *mdataNode = FindChildByProperty(profileNode, "sid", smdname.c_str() );
		if(mdataNode == NULL)
			mdataNode = FindChildByProperty(techniqueNode, "sid", smdname.c_str() );

		if( mdataNode != NULL ) 
		{
			xmlNode *stringNode = FindChildByType(mdataNode, "string" );
			smdata = ReadNodeContentDirect(stringNode);
		}

		meshdata.push_back(smdata);
	}

	// Look for the <shader> elements
	xmlNodeList shaderNodes;
	FindChildrenByType(passNode, DAE_SHADER_ELEMENT, shaderNodes);
	for (xmlNodeList::iterator itS = shaderNodes.begin(); itS != shaderNodes.end(); ++itS)
	{
		FCDEffectPassShader* shader = new FCDEffectPassShader(GetDocument(), this);
		shaders.push_back(shader);
		status.AppendStatus(shader->LoadFromXML(*itS));
	}

	return status;
}

// Write out the pass to the COLLADA xml node tree
xmlNode* FCDEffectPass::WriteToXML(xmlNode* parentNode) const
{
	// Write out the <pass> element, with the shader's name
	xmlNode* passNode = AddChild(parentNode, DAE_PASS_ELEMENT);
	if (!name.empty())
	{
		const_cast<FCDEffectPass*>(this)->name = TO_FSTRING(AddNodeSid(passNode, TO_STRING(name).c_str()));
	}

	// Write out the shaders
	for (FCDEffectPassShaderList::const_iterator itS = shaders.begin(); itS != shaders.end(); ++itS)
	{
		(*itS)->WriteToXML(passNode);
	}

	return passNode;
}
