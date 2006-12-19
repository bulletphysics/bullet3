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

#ifndef __DAE_LIBXMLPLUGIN__
#define __DAE_LIBXMLPLUGIN__

#include <vector>
#include <dae/daeElement.h>
#include <dae/daeMetaAttribute.h>
#include <dae/daeIOPlugin.h>

class daeElement;
class daeIntegrationObject;
class daeMetaElement;
class daeDocument;

struct _xmlTextReader;
struct _xmlTextWriter;

/**
 * The @c daeLIBXMLPlugin class derives from @c daeIOPlugin and implements an XML
 * input/output backend using libxml2 as a parser. When using this plugin, daeInterface::load() expects
 * an rfc 2396 compliant URI,  any URI supported by libxml2 should be properly 
 * handled including ones with network schemes and authority.  If the URI contains a fragment it will be ignored
 * and the entire referenced document will be loaded.  daeInterface::saveAs will only
 * handle a filename path at present (ie: no scheme or authority).
 */
class daeLIBXMLPlugin : public daeIOPlugin
{
public:
	// Constructor / destructor
	/**
	 * Constructor.
	 */
	DLLSPEC daeLIBXMLPlugin();
	/**
	 * Destructor.
	 */
	virtual DLLSPEC ~daeLIBXMLPlugin();
	virtual DLLSPEC daeInt setMeta(daeMetaElement *topMeta);

	// Database setup	
	virtual DLLSPEC void setDatabase(daeDatabase* database);

	// Operations
	virtual DLLSPEC daeInt read(daeURI& uri, daeString docBuffer);
	virtual DLLSPEC daeInt write(daeURI *name, daeDocument *document, daeBool replace);

	// Parsing support 
	DLLSPEC daeElementRef startParse(daeMetaElement* thisMetaElement, _xmlTextReader *reader);
	DLLSPEC daeElementRef nextElement(daeMetaElement* thisMetaElement, _xmlTextReader *reader);

	// Stats	
	virtual DLLSPEC void getProgress(daeInt* bytesParsed,
		daeInt* lineNumber,
		daeInt* totalBytes,
		daeBool reset = false );
	
private:
//	xmlTextReaderPtr reader;
	_xmlTextWriter *writer;

	typedef struct
	{
		daeElement* element;
		daeIntegrationObject* intObject;
	} INTEGRATION_ITEM;
	
	daeMetaElement* topMeta;
//	std::vector<INTEGRATION_ITEM> intItems;
	daeDatabase* database;
	
	void postProcessDom(daeDocument *document, daeElement* element, std::vector<INTEGRATION_ITEM> &intItems);

	void writeElement( daeElement* element ); 
	void writeAttribute( daeMetaAttribute* attr, daeElement* element, daeInt attrNum = -1 );

	void readAttributes( daeElement *element, _xmlTextReader *reader );
	void readValue( daeElement *element, _xmlTextReader *reader );
};

#endif //__DAE_XMLPLUGIN__
