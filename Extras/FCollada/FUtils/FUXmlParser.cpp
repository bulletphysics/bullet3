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
#include "FUtils/FUXmlParser.h"

#define xmlT(a) (const xmlChar*) (a)

namespace FUXmlParser
{
	// Returns the first child node of a given type
	xmlNode* FindChildByType(xmlNode* parent, const char* type)
	{
		if (parent != NULL)
		{
			for (xmlNode* child = parent->children; child != NULL; child = child->next)
			{
				if (child->type == XML_ELEMENT_NODE)
				{
					if (IsEquivalent(child->name, type)) return child;
				}
			}
		}
		return NULL;
	}

	// return the first child node of a given name
	xmlNode* FindChildByName(xmlNode* parent, const char* name)
	{
		if (parent != NULL)
		{
			for (xmlNode* child = parent->children; child != NULL; child = child->next)
			{
				if (child->type == XML_ELEMENT_NODE)
				{
					string str_name = ReadNodeProperty(child, "name");
					if (str_name == name) return child;
				}
			}
		}
		return NULL;
	}
	
	
	// return the first child node of a given property value
	xmlNode* FindChildByProperty(xmlNode* parent, const char* prop, const char* val )
	{
		if (parent != NULL)
		{
			for (xmlNode* child = parent->children; child != NULL; child = child->next)
			{
				string str_pop = ReadNodeProperty(child, prop);
				if (str_pop == val) return child;
				
			}
		}
		return NULL;
	}

	// return the first node in list of a given property
	xmlNode* FindNodeInListByProperty(xmlNodeList list, const char* property, const char* prop)
	{
		for (xmlNodeList::iterator it = list.begin(); it != list.end(); ++it)
		{
			xmlNode* element = *it;
			string str_prop = ReadNodeProperty(element, property);
			if (str_prop == prop) return element;
		
		}
		return NULL;
	}

	// Retrieves all the child nodes of a given type
	void FindChildrenByType(xmlNode* parent, const char* type, xmlNodeList& nodes)
	{
		if (parent != NULL)
		{
			for (xmlNode* child = parent->children; child != NULL; child = child->next)
			{
				if (child->type == XML_ELEMENT_NODE)
				{
					if (IsEquivalent(child->name, type)) nodes.push_back(child);
				} 
			}
		}
	}

	// Returns whether the given node has the given property
	bool HasNodeProperty(xmlNode* node, const char* property)
	{
		xmlAttr* attribute = xmlHasProp(node, xmlT(property));
		return attribute != NULL;
	}

	// Returns the string value of a node's property
	string ReadNodeProperty(xmlNode* node, const char* property)
	{
		string ret;
		if (node != NULL && property != NULL)
		{
			xmlChar* data = xmlGetProp(node, xmlT(property));
			if (data != NULL) ret = (const char*) data;
			xmlFree(data);
		}

		// Process the string for special characters
		XmlToString(ret);
		return ret;
	}

	// Returns the CRC value of a node's property
	FUCrc32::crc32 ReadNodePropertyCRC(xmlNode* node, const char* property)
	{
		FUCrc32::crc32 ret = 0;
		if (node != NULL && property != NULL)
		{
			xmlChar* data = xmlGetProp(node, xmlT(property));
			if (data != NULL) ret = FUCrc32::CRC32((const char*) data);
			xmlFree(data);
		}
		return ret;
	}

	// Returns the text content directly attached to a node
	const char* ReadNodeContentDirect(xmlNode* node)
	{
		if (node == NULL || node->children == NULL
			|| node->children->type != XML_TEXT_NODE || node->children->content == NULL) return "";
		return (const char*) node->children->content;
	}

	// Returns the inner text content of a node
	string ReadNodeContentFull(xmlNode* node)
	{
		string ret;
		if (node != NULL)
		{
			xmlChar* content = xmlNodeGetContent(node);
			ret = (const char*) content;
			xmlFree(content);
		}
		return ret;
	}

	// Convert a XML string to a text string: handles the '%' character
	void XmlToString(string& s)
	{
		if (s.find('%') == string::npos) return;

		// Replace all the '%XX' characters by the equivalent hex value
		const char* original = s.c_str();
		uint32 length = (uint32) s.length();
		char* copy = new char[length],* p = copy;
		for (uint32 i = 0; i < length; ++i, ++p)
		{
			char c = original[i];
			if (c != '%' || i >= length - 2) *p = c;
			else
			{
				char* dummy;
				p[0] = original[i + 1]; p[1] = original[i + 2]; p[2] = 0;
				uint32 v = strtoul(p, &dummy, 16);
				*p = (char) v;
				i += 2;
			}
		}
		*p = 0;
		s = copy;
	}
};
