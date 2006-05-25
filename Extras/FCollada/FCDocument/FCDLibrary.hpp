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

#include "FUtils/FUDaeParser.h"

template <class T>
FCDLibrary<T>::FCDLibrary(FCDocument* document) : FCDObject(document, "FCDLibrary")
{
}

template <class T>
FCDLibrary<T>::~FCDLibrary()
{
	CLEAR_POINTER_VECTOR(entities);
}


// Create a new entity within this library
template <class T>
T* FCDLibrary<T>::AddEntity()
{
	T* entity = new T(GetDocument());
	entities.push_back(entity);
	return entity;
}

// Deletes a entity of this library
template <class T>
void FCDLibrary<T>::ReleaseEntity(T* entity)
{
	// Not yet implemented, as this will most likely result in dangling pointers!
	// Needs more structure...
}


// Read in a list of entities for a library of a COLLADA document
template <class T>
FUStatus FCDLibrary<T>::LoadFromXML(xmlNode* node)
{
	FUStatus status;
	for (xmlNode* entityNode = node->children; entityNode != NULL; entityNode = entityNode->next)
	{
		if (entityNode->type == XML_ELEMENT_NODE)
		{
			T* entity = AddEntity();
			status.AppendStatus(entity->LoadFromXML(entityNode));
		}
	}
	return status;
}

// Write out the library to the COLLADA xml document
template <class T>
void FCDLibrary<T>::WriteToXML(xmlNode* node) const
{
	for (typename FCDEntityList::const_iterator itEntity = entities.begin(); itEntity != entities.end(); ++itEntity)
	{
		const T* entity = (const T*) (*itEntity);
		entity->WriteToXML(node);
	}
}

// Search for the entity in this library with a given COLLADA id.
template <class T>
T* FCDLibrary<T>::FindDaeId(const string& _daeId)
{
	const char* daeId = FUDaeParser::SkipPound(_daeId);
	for (typename FCDEntityList::iterator itEntity = entities.begin(); itEntity != entities.end(); ++itEntity)
	{
		if ((*itEntity)->GetDaeId() == daeId) return (*itEntity);
	}
	return NULL;
}

template <class T>
StringList FCDLibrary<T>::GetPostProcessCmds() const
{
	StringList res;
	for (typename FCDEntityList::const_iterator itEntity = entities.begin(); itEntity != entities.end(); ++itEntity)
	{
		res = (*itEntity)->GetPostProcessCmds();
	}
	return res;
}
