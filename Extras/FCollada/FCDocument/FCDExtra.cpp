/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDAnimated.h"
#include "FCDocument/FCDExtra.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDExtra::FCDExtra(FCDocument* document) : FCDObject(document, "FCDExtra")
{
}

FCDExtra::~FCDExtra()
{
	CLEAR_POINTER_VECTOR(techniques);
}

// Adds a technique of the given profile (or return the existing technique with this profile).
FCDETechnique* FCDExtra::AddTechnique(const char* profile)
{
	FCDETechnique* technique = FindTechnique(profile);
	if (technique == NULL)
	{
		technique = new FCDETechnique(GetDocument(), profile);
		techniques.push_back(technique);
	}
	return technique;
}

// Releases a technique contained within the extra tree.
void FCDExtra::ReleaseTechnique(FCDETechnique* technique)
{
	FCDETechniqueList::iterator it = std::find(techniques.begin(), techniques.end(), technique);
	if (it != techniques.end())
	{
		delete (*it);
		techniques.erase(it);
	}
}

// Search for a profile-specific technique
FCDETechnique* FCDExtra::FindTechnique(const char* profile)
{
	for (FCDETechniqueList::iterator itT = techniques.begin(); itT != techniques.end(); ++itT)
	{
		if (IsEquivalent((*itT)->GetProfile(), profile)) return *itT;
	}
	return NULL;
}
const FCDETechnique* FCDExtra::FindTechnique(const char* profile) const
{
	for (FCDETechniqueList::const_iterator itT = techniques.begin(); itT != techniques.end(); ++itT)
	{
		if (IsEquivalent((*itT)->GetProfile(), profile)) return *itT;
	}
	return NULL;
}

// Search for a root node with a specific element name
FCDENode* FCDExtra::FindRootNode(const char* name)
{
	FCDENode* rootNode = NULL;
	for (FCDETechniqueList::iterator itT = techniques.begin(); itT != techniques.end(); ++itT)
	{
		rootNode = (*itT)->FindChildNode(name);
		if (rootNode != NULL) break;
	}
	return rootNode;
}
const FCDENode* FCDExtra::FindRootNode(const char* name) const
{
	FCDENode* rootNode = NULL;
	for (FCDETechniqueList::const_iterator itT = techniques.begin(); itT != techniques.end(); ++itT)
	{
		rootNode = (*itT)->FindChildNode(name);
		if (rootNode != NULL) break;
	}
	return rootNode;
}

// Read in/Write to a COLLADA xml document
FUStatus FCDExtra::LoadFromXML(xmlNode* extraNode)
{
	FUStatus status;

	// Read in the techniques
	xmlNodeList techniqueNodes;
	FindChildrenByType(extraNode, DAE_TECHNIQUE_ELEMENT, techniqueNodes);
	for (xmlNodeList::iterator itN = techniqueNodes.begin(); itN != techniqueNodes.end(); ++itN)
	{
		xmlNode* techniqueNode = (*itN);
		FCDETechnique* technique = AddTechnique(ReadNodeProperty(techniqueNode, DAE_PROFILE_ATTRIBUTE));
		status.AppendStatus(technique->LoadFromXML(techniqueNode));
	}

	return status;
}

xmlNode* FCDExtra::WriteToXML(xmlNode* parentNode) const
{
	if (techniques.empty()) return NULL;

	// Add the <extra> element and its techniques
	xmlNode* extraNode = AddChildOnce(parentNode, DAE_EXTRA_ELEMENT);
	for (FCDETechniqueList::const_iterator itT = techniques.begin(); itT != techniques.end(); ++itT)
	{
		(*itT)->WriteToXML(extraNode);
	}
	return extraNode;
}

FCDENode::FCDENode(FCDocument* document, FCDENode* _parent) : FCDObject(document, "FCDENode")
{
	parent = _parent;
	animated = NULL;
}

FCDENode::~FCDENode()
{
	GetDocument()->UnregisterAnimatedValue(animated);
	SAFE_DELETE(animated);
	parent = NULL;

	CLEAR_POINTER_VECTOR(children);
	CLEAR_POINTER_VECTOR(attributes);
}

void FCDENode::Release()
{
	if (parent != NULL)
	{
		parent->ReleaseChildNode(this);
	}

	// Otherwise, we have a technique so don't release
}

void FCDENode::SetContent(const fchar* _content)
{
	// As COLLADA doesn't allow for mixed content, release all the children.
	while (!children.empty())
	{
		children.back()->Release();
	}

	content = _content;
}

// Search for a children with a specific name
FCDENode* FCDENode::FindChildNode(const char* name)
{
	for (FCDENodeList::iterator itN = children.begin(); itN != children.end(); ++itN)
	{
		if (IsEquivalent((*itN)->GetName(), name)) return (*itN);
	}
	return NULL;
}

const FCDENode* FCDENode::FindChildNode(const char* name) const
{
	for (FCDENodeList::const_iterator itN = children.begin(); itN != children.end(); ++itN)
	{
		if (IsEquivalent((*itN)->GetName(), name)) return (*itN);
	}
	return NULL;
}

// Adds a new child node
FCDENode* FCDENode::AddChildNode()
{
	FCDENode* node = new FCDENode(GetDocument(), this);
	children.push_back(node);
	return node;
}

// Releases a child node
void FCDENode::ReleaseChildNode(FCDENode* childNode)
{
	FCDENodeList::iterator itN = std::find(children.begin(), children.end(), childNode);
	if (itN != children.end())
	{
		delete (*itN);
		children.erase(itN);
	}
}


FCDENode* FCDENode::FindParameter(const char* name)
{
	for (FCDENodeList::iterator itN = children.begin(); itN != children.end(); ++itN)
	{
		FCDENode* node = (*itN);
		if (IsEquivalent(node->GetName(), name)) return node;
		else if (IsEquivalent(node->GetName(), DAE_PARAMETER_ELEMENT))
		{
			FCDEAttribute* nameAttribute = node->FindAttribute(DAE_NAME_ATTRIBUTE);
			if (nameAttribute != NULL && nameAttribute->value == TO_FSTRING(name)) return node;
		}
	}
	return NULL;
}

void FCDENode::FindParameters(FCDENodeList& nodes, StringList& names)
{
	for (FCDENodeList::iterator itN = children.begin(); itN != children.end(); ++itN)
	{
		FCDENode* node = (*itN);
		if (node->GetChildNodeCount() > 1) continue;

		if (IsEquivalent(node->GetName(), DAE_PARAMETER_ELEMENT))
		{
			FCDEAttribute* nameAttribute = node->FindAttribute(DAE_NAME_ATTRIBUTE);
			if (nameAttribute != NULL)
			{
				nodes.push_back(node);
				names.push_back(FUStringConversion::ToString(nameAttribute->value));
			}
		}
		else 
		{
			nodes.push_back(node);
			names.push_back(node->GetName());
		}
	}
}

// Adds a new attribute to this extra tree node.
FCDEAttribute* FCDENode::AddAttribute(const char* _name, const fchar* _value)
{
	FCDEAttribute* attribute = FindAttribute(_name);
	if (attribute == NULL)
	{
		attribute = new FCDEAttribute();
		attribute->name = _name;
		attributes.push_back(attribute);
	}

	attribute->value = _value;
	return attribute;
}

// Releases an attribute
void FCDENode::ReleaseAttribute(FCDEAttribute* attribute)
{
	FCDEAttributeList::iterator it = std::find(attributes.begin(), attributes.end(), attribute);
	if (it != attributes.end())
	{
		delete *it;
		attributes.erase(it);
	}
}

// Search for an attribute with a specific name
FCDEAttribute* FCDENode::FindAttribute(const char* name)
{
	for (FCDEAttributeList::iterator itA = attributes.begin(); itA != attributes.end(); ++itA)
	{
		if ((*itA)->name == name) return (*itA);
	}
	return NULL;
}
const FCDEAttribute* FCDENode::FindAttribute(const char* name) const
{
	for (FCDEAttributeList::const_iterator itA = attributes.begin(); itA != attributes.end(); ++itA)
	{
		if ((*itA)->name == name) return (*itA);
	}
	return NULL;
}

// Read in this extra node from a COLLADA xml document
FUStatus FCDENode::LoadFromXML(xmlNode* customNode)
{
	FUStatus status;

	// Read in the node's name and children
	name = (const char*) customNode->name;
	ReadChildrenFromXML(customNode);
	
	// If there are no child nodes, we have a tree leaf: parse in the content and its animation
	content = (children.empty()) ? TO_FSTRING(ReadNodeContentDirect(customNode)) : FS("");
	animated = FCDAnimatedCustom::Create(GetDocument(), customNode);

	// Read in the node's attributes
	for (xmlAttr* a = customNode->properties; a != NULL; a = a->next)
	{
		AddAttribute((const char*) a->name, (a->children != NULL) ? TO_FSTRING((const char*) (a->children->content)) : FS(""));
	}

	return status;
}

// Write out this extra to a COLLADA xml document
xmlNode* FCDENode::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* customNode = AddChild(parentNode, name.c_str(), content);
	
	// Write out the attributes
	for (FCDEAttributeList::const_iterator itA = attributes.begin(); itA != attributes.end(); ++itA)
	{
		const FCDEAttribute* attribute = (*itA);
		FUXmlWriter::AddAttribute(customNode, attribute->name.c_str(), attribute->value);
	}

	// Write out the children
	WriteChildrenToXML(customNode);
	return customNode;
}

// Read in the child nodes from the xml tree node
FUStatus FCDENode::ReadChildrenFromXML(xmlNode* customNode)
{
	FUStatus status;

	// Read in the node's children
	for (xmlNode* k = customNode->children; k != NULL; k = k->next)
	{
		if (k->type != XML_ELEMENT_NODE) continue;

		FCDENode* node = AddChildNode();
		status.AppendStatus(node->LoadFromXML(k));
	}

	return status;
}

// Write out the child nodes to the xml tree node
void FCDENode::WriteChildrenToXML(xmlNode* customNode) const
{
	for (FCDENodeList::const_iterator itN = children.begin(); itN != children.end(); ++itN)
	{
		(*itN)->WriteToXML(customNode);
	}
}

FCDETechnique::FCDETechnique(FCDocument* document, const char* _profile) : FCDENode(document, NULL)
{
	profile = _profile;
}

FCDETechnique::~FCDETechnique() {}

// Read in/Write to a COLLADA xml document
FUStatus FCDETechnique::LoadFromXML(xmlNode* techniqueNode)
{
	FUStatus status;

	// Read in only the child elements: none of the attributes
	status.AppendStatus(ReadChildrenFromXML(techniqueNode));
	return status;
}

xmlNode* FCDETechnique::WriteToXML(xmlNode* parentNode) const
{
	// Create the technique for this profile and write out the children
	xmlNode* customNode = AddTechniqueChild(parentNode, profile.c_str());
	WriteChildrenToXML(customNode);
	return customNode;
}

