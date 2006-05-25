/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeWriter;

template <class T>
FCDPhysicsParameter<T>::FCDPhysicsParameter(FCDocument* document, const string& ref) : FCDPhysicsParameterGeneric(document, ref)
{
	value = NULL;
}

template <class T>
FCDPhysicsParameter<T>::~FCDPhysicsParameter()
{
	SAFE_DELETE(value);
}

// Clone
template <class T>
FCDPhysicsParameterGeneric* FCDPhysicsParameter<T>::Clone()
{
	FCDPhysicsParameterGeneric *clone = new FCDPhysicsParameter<T>(GetDocument(), reference);
	((FCDPhysicsParameter<T>*)clone)->value = new T(*value);
	return clone;
}

template <class T>
void FCDPhysicsParameter<T>::SetValue(T val)
{
	if(value)
	{
		SAFE_DELETE(value);
	}
	value = new T();
	*value = val;
}


template <class T>
void FCDPhysicsParameter<T>::SetValue(T* val)
{
	if(value)
	{
		SAFE_DELETE(value);
	}
	value = val;
}

// Flattening: overwrite the target parameter with this parameter
template <class T>
void FCDPhysicsParameter<T>::Overwrite(FCDPhysicsParameterGeneric* target)
{
	((FCDPhysicsParameter<T>*) target)->SetValue(value);
}

/*
// Parse in this Collada parameter from the xml node tree
template <class T>
FUStatus FCDPhysicsParameter<T>::LoadFromXML(xmlNode* parameterNode)
{
	FUStatus status;
	return status;
}
*/
// Write out this ColladaFX parameter to the xml node tree
template <class T>
xmlNode* FCDPhysicsParameter<T>::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* parameterNode = FUXmlWriter::AddChild(parentNode, reference.c_str());
	//TODO: complete
	return parameterNode;
}
