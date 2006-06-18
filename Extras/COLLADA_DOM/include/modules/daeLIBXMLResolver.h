/*
 * Copyright 2006 Sony Computer Entertainment Inc.
 *
 * Licensed under the SCEA Shared Source License, Version 1.0 (the "License"); you may not use this 
 * file except in compliance with the License. You may obtain a copy of the License at:
 * http://research.scea.com/scea_shared_source_license.html
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License 
 * is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or 
 * implied. See the License for the specific language governing permissions and limitations under the 
 * License. 
 */

#ifndef __DAE_LIBXMLRESOLVER__
#define __DAE_LIBXMLRESOVLER__

#include "dae/daeURI.h"
class daeIOPlugin;
class daeDatabase;

/**
 * The @c daeLIBXMLResolver class derives from @c daeURIResolver and implements
 * the default XML backend resolver.
 */
class daeLIBXMLResolver : public daeURIResolver
{
public:
	/**
	 * Constructor.
	 * @param database The @c daeDatabase used.
	 * @param plugin The @c daeIOPlugin used.
	 */
	daeLIBXMLResolver(daeDatabase* database, daeIOPlugin* plugin);
	/**
	 * Destructor.
	 */
	~daeLIBXMLResolver();

protected:
	daeDatabase* _database;
	daeIOPlugin* _plugin;
public:
public: // Abstract Interface
	virtual daeBool resolveElement(daeURI& uri, daeString typeNameHint = NULL);
	virtual daeBool resolveURI(daeURI& uri);
	virtual daeString getName();
	virtual daeBool isProtocolSupported(daeString protocol);
	virtual daeBool isExtensionSupported(daeString extension);
};

#endif //__DAE_XMLRESOLVER__

