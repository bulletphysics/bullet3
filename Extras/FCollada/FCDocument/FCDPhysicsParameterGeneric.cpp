/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDPhysicsParameterGeneric.h"
#include "FCDocument/FCDocument.h"

FCDPhysicsParameterGeneric::FCDPhysicsParameterGeneric(FCDocument* document, const string& ref) : FCDObject(document, "FCDPhysicsParameterGeneric")
{
	isGenerator = true;
	reference = ref;
}
FCDPhysicsParameterGeneric::~FCDPhysicsParameterGeneric()
{
}

