/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FUtils/FUXmlWriter.h"
#include "FUtils/FUXmlParser.h"
#include "FUtils/FUStringConversion.h"

#define xcT(text) (const xmlChar*) (text)

namespace FUXmlWriter
{
	// Create a new xml tree node
	xmlNode* CreateNode(const char* name)
	{
		return xmlNewNode(NULL, xcT(name));
	}

	// Create a new xml tree child node, parented to the given xml tree node
	void AddChild(xmlNode* parent, xmlNode* child)
	{
		xmlAddChild(parent, child);
	}

	xmlNode* AddChild(xmlNode* parent, const char* name)
	{
		return (parent != NULL) ? xmlNewChild(parent, NULL, xcT(name), NULL) : NULL;
	}

	xmlNode* AddChild(xmlNode* parent, const char* name, const char* content)
	{
		if (content != NULL && *content == 0) content = NULL;
		return (parent != NULL) ? xmlNewChild(parent, NULL, xcT(name), xcT(content)) : NULL;
	}

#ifdef UNICODE
	xmlNode* AddChild(xmlNode* parent, const char* name, const fstring& content)
	{
		string s = FUStringConversion::ToString(content);
		return AddChild(parent, name, !s.empty() ? s.c_str() : NULL);
	}
#endif

	xmlNode* AddChildOnce(xmlNode* parent, const char* name, const char* content)
	{
		xmlNode* node = NULL;
		if (parent != NULL)
		{
			node = FUXmlParser::FindChildByType(parent, name);
			if (node == NULL) node = AddChild(parent, name, (content == NULL || *content == 0) ? NULL : content);
		}
		return node;
	}

	// Create/Append a new xml tree node, as a sibling to a given xml tree node
	void AddSibling(xmlNode* sibling, xmlNode* dangling)
	{
		xmlAddSibling(sibling, dangling);
	}

	xmlNode* AddSibling(xmlNode* node, const char* name)
	{
		xmlNode* n = CreateNode(name);
		AddSibling(node, n);
		return n;
	}

	void AddContent(xmlNode* node, const char* content)
	{
		if (node != NULL)
		{
			xmlNodeAddContent(node, xcT(content));
		}
	}

#ifdef UNICODE
	void AddContent(xmlNode* node, const fstring& content)
	{
		string s = FUStringConversion::ToString(content);
		AddContent(node, s.c_str());
	}
#endif

	void AddAttribute(xmlNode* node, const char* attributeName, const char* value)
	{
		if (node != NULL)
		{
			xmlNewProp(node, xcT(attributeName), xcT(value));
		}
	}

#ifdef UNICODE
	void AddAttribute(xmlNode* node, const char* attributeName, const fstring& attributeValue)
	{
		string s = FUStringConversion::ToString(attributeValue);
		AddAttribute(node, attributeName, s.c_str());
	}
#endif

	// Insert a child, respecting lexical ordering
	void AddChildSorted(xmlNode* parent, xmlNode* child)
	{
		// Do an insertion sort in alphabetical order of the element names. 
		// Walk backward from the last child, to make sure that
		// the chronological ordering of elements of the same type is respected.
		//
		for (xmlNode* p = xmlGetLastChild(parent); p != NULL; p = p->prev)
		{
			if (p->type != XML_ELEMENT_NODE) continue;
			if (strcmp((const char*) p->name, (const char*) child->name) <= 0)
			{
				xmlAddNextSibling(p, child);
				return;
			}
		}

		// Add to the top of the list.
		if (parent->children && parent->children->type == XML_ELEMENT_NODE)
		{
			xmlAddPrevSibling(parent->children, child);
		}
		else
		{
			AddChild(parent, child);
		}
	}

	xmlNode* AddChildSorted(xmlNode* parent, const char* name, const char* content)
	{
		xmlNode* node = CreateNode(name);
		if (content != NULL && *content != 0) xmlNodeAddContent(node, xcT(content));
		AddChildSorted(parent, node);
		return node;
	}
};
