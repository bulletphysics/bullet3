/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDEffectParameter.h"
#include "FCDocument/FCDEffectParameterFactory.h"
#include "FCDocument/FCDEffectParameterList.h"

FCDEffectParameterList::FCDEffectParameterList(FCDocument* document, bool _ownParameters)
 :	FCDObject(document, "FCDEffectParameterList")
{
	ownParameters = _ownParameters;
}

FCDEffectParameterList::~FCDEffectParameterList()
{
	if (ownParameters)
	{
		size_t l = size();
		for (size_t i = 0; i < l; ++i)
		{
			FCDEffectParameter* p = at(i);
			SAFE_DELETE(p);
		}
	}
	clear();
	ownParameters = false;
}

FCDEffectParameter* FCDEffectParameterList::AddParameter(uint32 type)
{
	FCDEffectParameter* parameter = NULL;
	if (ownParameters)
	{
		parameter = FCDEffectParameterFactory::Create(GetDocument(), type);
		push_back(parameter);
	}
	return parameter;
}

void FCDEffectParameterList::ReleaseParameter(FCDEffectParameter* parameter)
{
	iterator it = std::find(begin(), end(), parameter);
	if (it != end())
	{
		if (ownParameters) delete *it;
		erase(it);
	}
}

FCDEffectParameter* FCDEffectParameterList::FindReference(const char* reference)
{
	for (iterator it = begin(); it != end(); ++it)
	{
		if ((*it)->GetReference() == reference) return (*it);
	}
	return NULL;
}
const FCDEffectParameter* FCDEffectParameterList::FindReference(const char* reference) const
{
	for (const_iterator it = begin(); it != end(); ++it)
	{
		if ((*it)->GetReference() == reference) return (*it);
	}
	return NULL;
}

FCDEffectParameter* FCDEffectParameterList::FindSemantic(const char* semantic)
{
	for (iterator it = begin(); it != end(); ++it)
	{
		if ((*it)->GetSemantic() == semantic) return (*it);
	}
	return NULL;
}
const FCDEffectParameter* FCDEffectParameterList::FindSemantic(const char* semantic) const
{
	for (const_iterator it = begin(); it != end(); ++it)
	{
		if ((*it)->GetSemantic() == semantic) return (*it);
	}
	return NULL;
}

void FCDEffectParameterList::FindReference(const char* reference, FCDEffectParameterList& list)
{
	for (iterator it = begin(); it != end(); ++it)
	{
		if ((*it)->GetReference() == reference) list.push_back(*it);
	}
}

void FCDEffectParameterList::FindSemantic(const char* semantic, FCDEffectParameterList& list)
{
	for (iterator it = begin(); it != end(); ++it)
	{
		if ((*it)->GetSemantic() == semantic) list.push_back(*it);
	}
}

// Copy this list
FCDEffectParameterList* FCDEffectParameterList::Clone() const
{
	FCDEffectParameterList* clone = new FCDEffectParameterList(GetDocument(), ownParameters);
	if (!empty())
	{
		clone->reserve(size());

		if (ownParameters)
		{
			for (const_iterator it = begin(); it != end(); ++it)
			{
				clone->push_back((*it)->Clone());
			}
		}
		else
		{
			(*clone) = (*this);
		}
	}
	return clone;
}
