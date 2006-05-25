/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDEffect.h"
#include "FCDocument/FCDEffectProfile.h"

FCDEffectProfile::FCDEffectProfile(FCDocument* document, FCDEffect* _parent) : FCDObject(document, "FCDEffectProfile")
{
	parent = _parent;
}

FCDEffectProfile::~FCDEffectProfile()
{
	parent = NULL;
}

const string& FCDEffectProfile::GetDaeId() const
{
	return parent->GetDaeId();
}
