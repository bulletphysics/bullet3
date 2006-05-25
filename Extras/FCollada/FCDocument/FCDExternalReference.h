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

#ifndef _FCD_XREF_ENTITY_H_
#define _FCD_XREF_ENTITY_H_

#include "FCDocument/FCDEntityInstance.h"
#include "FUtils/FUStringConversion.h"
#include "FUtils/FUUri.h"

class FCOLLADA_EXPORT FCDExternalReference : public FCDEntityInstance
{
private:
	FUUri uri;

public:
	FCDExternalReference(FCDocument* document, const FUUri& _uri) : FCDEntityInstance(document) { uri = _uri; }
	virtual ~FCDExternalReference() {}

	// FCDEntity override for RTTI-like
	virtual Type GetType() const { return EXTERNAL_REFERENCE; }

	// Accessors
	const FUUri& GetUri() const { return uri; }
	const fstring& GetFilename() const { return uri.prefix; }
	fstring GetObjectName() const { return TO_FSTRING(uri.suffix); }
};

#endif // _FCD_XREF_ENTITY_H_
