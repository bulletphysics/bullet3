/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDObject.h"
#include "FUtils/FUUniqueStringMap.h"
#include "FUtils/FUDaeWriter.h"

// 
// FCDObject
//

FCDObject::FCDObject(FUObjectContainer* container)
:	FUObject_Construct(container, "FCDObject")
{
}

FCDObject::FCDObject(FCDocument* document, const char* UNUSED_NDEBUG(className))
:	FUObject_Construct(document, className)
{
}

//
// FCDObjectWithId
//

FCDObjectWithId::FCDObjectWithId(FCDocument* document, const char* baseId)
:	FCDObject(document, baseId)
{
	daeId = baseId;
	hasUniqueId = false;
}

FCDObjectWithId::~FCDObjectWithId()
{
	RemoveDaeId();
}

void FCDObjectWithId::Clone(FCDObjectWithId* clone) const
{
	clone->daeId = daeId;
	clone->hasUniqueId = hasUniqueId;
}

const string& FCDObjectWithId::GetDaeId() const
{
	if (!hasUniqueId)
	{
		// Generate a new id
		FCDObjectWithId* e = const_cast<FCDObjectWithId*>(this);
		FUSUniqueStringMap* names = e->GetDocument()->GetUniqueNameMap();
		FUAssert(!e->daeId.empty(), e->daeId = "unknown_object");
		names->AddUniqueString(e->daeId);
		e->hasUniqueId = true;
	}
	return daeId;
}

void FCDObjectWithId::SetDaeId(const string& id)
{
	RemoveDaeId();

	// Use this id to enforce a unique id.
	FUSUniqueStringMap* names = GetDocument()->GetUniqueNameMap();
	daeId = FUDaeWriter::CleanId(id);
	names->AddUniqueString(daeId);
	hasUniqueId = true;
}

void FCDObjectWithId::SetDaeId(string& id)
{
	RemoveDaeId();

	// Use this id to enforce a unique id.
	FUSUniqueStringMap* names = GetDocument()->GetUniqueNameMap();
	daeId = FUDaeWriter::CleanId(id);
	names->AddUniqueString(daeId);
	id = daeId;
	hasUniqueId = true;
}

void FCDObjectWithId::RemoveDaeId()
{
	if (hasUniqueId)
	{
		FUSUniqueStringMap* names = GetDocument()->GetUniqueNameMap();
		names->Erase(daeId);
		hasUniqueId = false;
	}
}
