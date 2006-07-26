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

#include <modules/daeLIBXMLResolver.h>
#include <dae/daeDatabase.h>
#include <dae/daeURI.h>
#include <modules/daeLIBXMLPlugin.h>
#include <dae/daeErrorHandler.h>

daeLIBXMLResolver::daeLIBXMLResolver(daeDatabase* database,daeIOPlugin* plugin)
{
	_database = database;
	_plugin = plugin;
}

daeLIBXMLResolver::~daeLIBXMLResolver()
{
}

daeBool
daeLIBXMLResolver::resolveURI(daeURI& uri)
{
	(void)uri;
	return false;
}

daeString
daeLIBXMLResolver::getName()
{
	return "XMLResolver";
}

daeBool
daeLIBXMLResolver::isExtensionSupported(daeString extension)
{
	if ((extension!=NULL) &&
		(strlen(extension) > 0) &&
		((strncmp(extension,"xml",3)==0) ||
		 (strncmp(extension,"XML",3)==0) ||
		 (strncmp(extension,"DAE",3)==0) ||
		 (strncmp(extension,"dae",3)==0)))
		return true;
	return false;
}
		
daeBool
daeLIBXMLResolver::isProtocolSupported(daeString protocol)
{
	if ((protocol!=NULL) &&
		(strlen(protocol) > 0) &&
		((strncmp(protocol,"file",4) == 0) ||
		(strncmp(protocol,"http",4) == 0)))
		return true;
	return false;
}

daeBool
daeLIBXMLResolver::resolveElement(daeURI& uri, daeString typeNameHint)
{
	// Make sure the URI is validated
	if (uri.getState() == daeURI::uri_loaded)
	{
		uri.validate();
	}

	daeElement* resolved = NULL;
	int status;

	// Does the URI have a document reference?
	if ( (uri.getFile() != NULL) &&	(strlen(uri.getFile())>0)) 
	{
		// The URI contains a document reference, see if it is loaded and try to load it if it's not
		if (!_database->isDocumentLoaded(uri.getURI())) {
			if ( _loadExternalDocuments ) {
				_plugin->read(uri,NULL);
			}
			else {
				uri.setState( daeURI::uri_failed_external_document );
				return false;
			}
		}
		// Try to find the id by searching this document only
		status = _database->getElement(&resolved,0,uri.getID(),typeNameHint,uri.getURI());
	}
	else
	{
		// The URI was just a fragment, so try to find it in the document that contains it.
		// !!!GAC not sure if all these pointers will be set when we get here, so assert if any of them aren't
		daeElement *tempElement = uri.getContainer();
		//assert(tempElement);
		daeDocument *tempDocument;
		if ( tempElement == NULL || (tempDocument = tempElement->getDocument()) == NULL ) {
			uri.setState(daeURI::uri_failed_missing_container);
			char msg[256];
			sprintf(msg,
					"daeLIBXMLResolver::resolveElement() - failed to resolve %s\n",
					uri.getURI());
			daeErrorHandler::get()->handleError( msg );
			return false;
		}
		//assert(tempDocument);
		daeURI *tempURI = tempDocument->getDocumentURI();
		//assert(tempURI);
		status = _database->getElement(&resolved,0,uri.getID(),typeNameHint,tempURI->getURI());
	}

	uri.setElement(resolved);

	// Error if we didn't successfully resolve the uri

	if (status ||(resolved==NULL)) 
	{
		uri.setState(daeURI::uri_failed_id_not_found);
		char msg[256];
		sprintf(msg,
				"daeLIBXMLResolver::resolveElement() - failed to resolve %s\n",
				uri.getURI());
		daeErrorHandler::get()->handleError( msg );
		return false;
	}

	uri.setState(daeURI::uri_success);
	return true;
}

   


