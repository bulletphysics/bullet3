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

#ifndef __DAE__
#define __DAE__

#include <dae/daeTypes.h>
#include <dae/daeError.h>
#include <dae/daeInterface.h>
#include <dae/daeDatabase.h>
#include <dae/daeIOPlugin.h>
#include <dae/daeIntegrationObject.h>

class daeIDRefResolver;
class domCOLLADA;

/**
 * The @c DAE class implements a standard interface to the
 * COLLADA runtime database.
 *
 * @c DAE serves as a wrapper for the entire pipeline ensuring
 * a consistent interface, regardless of extensions to or replacements
 * for the various API components. It provides methods to load, store,
 * translate and query COLLADA elements. A @c DAE object automatically creates
 * and initializes default versions of the COLLADA backend, the COLLADA
 * runtime database, and registered integration libraries. 
 */
class DAE : public daeInterface
{
public:	
	/** 
	*  Constructor.
	*/	
	DAE();
	/** 
	* Destructor.
	*/	
	virtual ~DAE();

	/**
	 * Before exiting an application, call @c cleanup()
	 * to release all static meta information associated with the
	 * COLLADA api.
	 * The @c daeMetaElement::releaseMetas() and
	 * @c daeAtomicType::uninitializeKnownTypes() functions are called
	 * if there are no remaining instances of a @c DAE.
	 * @note This function is useless when called by the application in a non-static
	 * context. It should be called after you delete your DAE object.
	 */
	static void cleanup();
	
	// Abstract Interface Class for the daeDatabase front end
public:
	// Database setup	
	virtual daeDatabase* getDatabase();
	virtual daeInt setDatabase(daeDatabase* database);

	// IO Plugin setup
	virtual daeIOPlugin* getIOPlugin();
	virtual daeInt setIOPlugin(daeIOPlugin* plugin);

	// Integration Library Setup
	virtual daeIntegrationLibraryFunc getIntegrationLibrary();
	virtual daeInt setIntegrationLibrary(daeIntegrationLibraryFunc regFunc);

	// batch file operations
	virtual daeInt load(daeString name, daeString docBuffer = NULL);
	virtual daeInt save(daeString documentName, daeBool replace=true);
	virtual daeInt save(daeUInt documentIndex, daeBool replace=true);
	virtual daeInt saveAs(daeString name, daeString documentName, daeBool replace=true);
	virtual daeInt saveAs(daeString name, daeUInt documentIndex=0, daeBool replace=true);

	virtual daeInt unload(daeString name);
	virtual daeInt clear();

	// Load/Save Progress	
	virtual void getProgress(daeInt* bytesParsed,
		daeInt* lineNumber,
		daeInt* totalBytes,
		daeBool reset = false );

	// Simple Query
	virtual domCOLLADA* getDom(daeString name);
	virtual daeString getDomVersion();
	virtual daeInt setDom(daeString name, domCOLLADA* dom);

private:
	daeDatabase *database;
	daeIOPlugin *plugin;
	daeURIResolver* resolver;
	daeIDRefResolver* idResolver;
	bool defaultDatabase;
	bool defaultPlugin;
	daeIntegrationLibraryFunc registerFunc; 
	daeMetaElement *topMeta;
};

#endif // __DAE_INTERFACE__
