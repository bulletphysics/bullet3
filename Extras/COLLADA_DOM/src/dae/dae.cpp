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

#include <dae.h>
#include <dae/daeDom.h>
#include <dae/daeIDRef.h>
#include <dae/daeMetaElement.h>
#include <modules/daeSTLDatabase.h>
#include <dae/daeErrorHandler.h>

#ifndef NO_DEFAULT_PLUGIN
#ifndef DEFAULT_BXCEPLUGIN
#include <modules/daeLIBXMLPlugin.h>
#include <modules/daeLIBXMLResolver.h>
#else
//This plugin is not provided with the public release. If you don't know about it then you don't need
//to worry about it.
#include <modules/daebXCePlugin.h>
#include <modules/daebXCeResolver.h>
#endif
#endif

// Don't include domConstants.h because it varies depending on the dom version,
// just extern the one thing we need (COLLADA_VERSION) which all versions of
// domConstants.h/.cpp are required to define.

extern daeString COLLADA_VERSION;		 

daeInt DAEInstanceCount = 0;

void
DAE::cleanup()
{
	if (DAEInstanceCount == 0) {
		daeMetaElement::releaseMetas();
		daeAtomicType::uninitializeKnownTypes();
	}
}
	
// constructor
DAE::DAE() : database(NULL),
	     plugin(NULL),
	     resolver(NULL),
	     idResolver(NULL),
	     defaultDatabase(false),
	     defaultPlugin(false),
	     registerFunc(NULL)
{
	topMeta = initializeDomMeta();
	DAEInstanceCount++;
}

DAE::~DAE()
{
	if (defaultDatabase)
		delete database;
	if (defaultPlugin) {
		delete plugin;
		delete resolver;
		delete idResolver;
	}
	topMeta = NULL;
	--DAEInstanceCount;
}

// Database setup	
daeDatabase* DAE::getDatabase()
{
	return database;	
}

daeInt DAE::setDatabase(daeDatabase* _database)
{
	if (defaultDatabase)
		delete database;
	if (_database)
		database = _database;
	else
	{
		//create default database
		database = new daeSTLDatabase;
		defaultDatabase = true;
	}
	// !!!GAC Not sure what good the error return is, current implementations never fail, what would we do if they did?
	int res = database->setMeta(topMeta);
	(void)res;
	return DAE_OK;
}

// IO Plugin setup
daeIOPlugin* DAE::getIOPlugin()
{
	return plugin;
}

daeInt DAE::setIOPlugin(daeIOPlugin* _plugin)
{
	if (defaultPlugin)
		delete plugin;
	if (_plugin)
		plugin = _plugin;
	else
	{
		//create default plugin
#ifndef NO_DEFAULT_PLUGIN
#ifndef DEFAULT_BXCEPLUGIN
		plugin = new daeLIBXMLPlugin;
		defaultPlugin = true;
		resolver = new daeLIBXMLResolver(database,plugin);
#else
		plugin = new daebXCePlugin();
		defaultPlugin = true;
		resolver = new daebXCeResolver(database, plugin);
#endif
#else
		daeErrorHandler::get()->handleWarning( "No IOPlugin Set! NO_DEFAULT_PLUGIN is defined." );
		plugin = NULL;
		return DAE_ERR_BACKEND_IO;
#endif

		// Setup the IDRef resolver
		idResolver = new daeDefaultIDRefResolver(database);
	}
	int res = plugin->setMeta(topMeta);
	if (res != DAE_OK)
	{
		if (defaultPlugin)
		{
			defaultPlugin = false;
			delete plugin;
		}
		plugin = NULL;
		return res;
	}
	return DAE_OK;
}

// Integration Library Setup
daeIntegrationLibraryFunc DAE::getIntegrationLibrary()
{
	return registerFunc;
}

daeInt DAE::setIntegrationLibrary(daeIntegrationLibraryFunc _registerFunc)
{
	registerFunc = _registerFunc;
	return DAE_OK;
}

// batch file operations
daeInt DAE::load(daeString name, daeString docBuffer)
{
	if (!database)
		setDatabase(NULL);

	if (!plugin)
		setIOPlugin(NULL);
	
	if (registerFunc) 
		registerFunc();

	if ( !plugin || !database ) {
		printf( "no plugin or database\n" );
		return DAE_ERR_BACKEND_IO;
	}

	plugin->setDatabase(database);

	if (!name || name[0] == '\0')
		return DAE_ERR_INVALID_CALL;

	daeURI tempURI(name);
	
	return plugin->read(tempURI, docBuffer);
}
daeInt DAE::save(daeString documentName, daeBool replace)
{
	if (!database)
		setDatabase(NULL);

	if (!plugin)
		setIOPlugin(NULL);

	if (registerFunc) 
		registerFunc();
	
	if ( !plugin || !database ) {
		return DAE_ERR_BACKEND_IO;
	}

	plugin->setDatabase(database);
	
	// Find the document we want by name
	daeDocument* document = database->getDocument(documentName);
	if(document == NULL)
		return DAE_ERR_COLLECTION_DOES_NOT_EXIST;

	// Save it out to the URI it was loaded from
	return plugin->write(document->getDocumentURI(), document, replace);

}
daeInt DAE::save(daeUInt documentIndex, daeBool replace)
{
	if (!database)
		setDatabase(NULL);

	if (!plugin)
		setIOPlugin(NULL);

	if (registerFunc) 
		registerFunc();
	
	if ( !plugin || !database ) {
		return DAE_ERR_BACKEND_IO;
	}

	plugin->setDatabase(database);
	
	if(documentIndex >= database->getDocumentCount())
		return DAE_ERR_COLLECTION_DOES_NOT_EXIST;

	daeDocument *document = database->getDocument(documentIndex);
	
	// Save it out to the URI it was loaded from
	return plugin->write(document->getDocumentURI(), document, replace);
}
daeInt DAE::saveAs(daeString name, daeString documentName, daeBool replace)
{
	if (!database)
		setDatabase(NULL);

	if (!plugin)
		setIOPlugin(NULL);

	if (registerFunc) 
		registerFunc();
	
	if ( !plugin || !database ) {
		return DAE_ERR_BACKEND_IO;
	}

	plugin->setDatabase(database);
	
	// Find the document we want by name
	daeDocument* document = database->getDocument(documentName);
	if(document == NULL)
		return DAE_ERR_COLLECTION_DOES_NOT_EXIST;

	// Make a URI from "name" and save to that

	daeURI tempURI(name);
	return plugin->write(&tempURI, document, replace);
	
}
daeInt DAE::saveAs(daeString name, daeUInt documentIndex, daeBool replace)
{
	if (!database)
		setDatabase(NULL);

	if (!plugin)
		setIOPlugin(NULL);

	if (registerFunc) 
		registerFunc();
	
	if ( !plugin || !database ) {
		return DAE_ERR_BACKEND_IO;
	}

	plugin->setDatabase(database);
	
	if(documentIndex >= database->getDocumentCount())
		return DAE_ERR_COLLECTION_DOES_NOT_EXIST;

	daeDocument *document = database->getDocument(documentIndex);
	
	daeURI tempURI(name);
	return plugin->write(&tempURI, document, replace);
}
daeInt DAE::unload(daeString name)
{
	daeDocument *col = database->getDocument( name );
	if ( col == NULL ) return DAE_ERR_COLLECTION_DOES_NOT_EXIST;
	return database->removeDocument( col );
}

daeInt DAE::clear()
{
	if (database)
		database->clear();
	return DAE_OK;
}

// Load/Save Progress	
void DAE::getProgress(daeInt* bytesParsed,daeInt* lineNumber,daeInt* totalBytes,daeBool reset)
{
	if (!database || !plugin)
	{
		if (bytesParsed)
			*bytesParsed=0;
		if (bytesParsed)
			*lineNumber=0;
		if (totalBytes)
			*totalBytes=0;
	}
	else
		plugin->getProgress(bytesParsed,lineNumber,totalBytes,reset);
}

// Simple Query
domCOLLADA* DAE::getDom(daeString name)
{
	if (!database)
		return NULL;

	// Find the document by name
	daeDocument *document = database->getDocument(name);
	if(document)
	{
		// Return the root domCOLLADA element
		return (domCOLLADA*)(daeElement*)document->getDomRoot();
	}
	else
	{
		return(NULL);
	}
}

daeInt DAE::setDom(daeString name, domCOLLADA* dom)
{
	if (!database)
		setDatabase(NULL);
	
	// Find the document by name
	
	daeDocument *document = database->getDocument(name);

	if(document)
	{
		//replace a DOM on an existing document by the one provided.
		// Note that the casts are needed because we don't want to include the full definition of domCOLLADA
		document->setDomRoot((daeElement*)dom);
		return DAE_OK;
	}
	else
	{
		// Document doesn't exist, make a new one
		return database->insertDocument(name,(daeElement*)dom);
	}
}
daeString DAE::getDomVersion()
{
	return(COLLADA_VERSION);
}
