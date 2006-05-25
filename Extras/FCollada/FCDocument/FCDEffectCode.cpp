/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDEffectCode.h"
#include "FUtils/FUFileManager.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDEffectCode::FCDEffectCode(FCDocument* document) : FCDObject(document, "FCDEffectCode")
{
	type = INCLUDE;
}

FCDEffectCode::~FCDEffectCode()
{
}

// Clone
FCDEffectCode* FCDEffectCode::Clone() const
{
	FCDEffectCode* clone = new FCDEffectCode(GetDocument());
	clone->type = type;
	clone->sid = sid;
	clone->filename = filename;
	clone->code = code;
	return clone;
}

// Read in the code/include from the xml node tree
FUStatus FCDEffectCode::LoadFromXML(xmlNode* codeNode)
{
	FUStatus status;
	if (IsEquivalent(codeNode->name, DAE_FXCMN_INCLUDE_ELEMENT)) type = INCLUDE;
	else if (IsEquivalent(codeNode->name, DAE_FXCMN_CODE_ELEMENT)) type = CODE;
	else
	{
		return status.Fail(FS("Unknown effect code type."), codeNode->line);
	}

	// Read in the code identifier and the actual code or filename
	sid = ReadNodeProperty(codeNode, DAE_SID_ATTRIBUTE);
	if (type == INCLUDE && sid.empty())
	{
		status.Warning(FS("<code>/<include> nodes must have an 'sid' attribute to identify them."), codeNode->line);
	}
	if (type == INCLUDE) 
	{
		filename = ReadNodeUrl(codeNode).prefix;
		filename = GetDocument()->GetFileManager()->GetFilePath(filename);
	}
	else
	{
		code = TO_FSTRING(ReadNodeContentDirect(codeNode)); 
	}

	return status;
}

// Write out the code/include to the COLLADA xml node tree
xmlNode* FCDEffectCode::WriteToXML(xmlNode* parentNode) const
{
	// In COLLADA 1.4, the 'sid' and 'url' attributes are required.
	// In the case of the sub-id, save it for later use.
	xmlNode* codeNode;
	switch (type)
	{
	case CODE:
		codeNode = AddChild(parentNode, DAE_FXCMN_CODE_ELEMENT, code);
		const_cast<FCDEffectCode*>(this)->sid = AddNodeSid(codeNode, !sid.empty() ? sid.c_str() : "code");
		break;

	case INCLUDE:
		codeNode = AddChild(parentNode, DAE_FXCMN_INCLUDE_ELEMENT);
		const_cast<FCDEffectCode*>(this)->sid = AddNodeSid(codeNode, !sid.empty() ? sid.c_str() : "include");
		AddAttribute(codeNode, DAE_URL_ATTRIBUTE, filename);
		break;

	default:
		codeNode = NULL;
	}
	return codeNode;
}
