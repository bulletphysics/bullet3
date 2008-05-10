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

// This is a rework of the XML plugin that contains a complete interface to libxml2 "readXML"
// This is intended to be a seperate plugin but I'm starting out by rewriting it in daeLIBXMLPlugin
// because I'm not sure if all the plugin handling stuff has been tested.  Once I get a working
// plugin I'll look into renaming it so the old daeLIBXMLPlugin can coexist with it.
//
#include <dae/daeMetaElement.h>
#include <modules/daeLIBXMLPlugin.h>
#include <dae.h>
#include <dom.h>

#include <libxml/xmlreader.h>
#include <libxml/xmlwriter.h>
#include <libxml/xmlmemory.h>
#include <dae/daeErrorHandler.h>
#include <dae/daeMetaElementAttribute.h>

daeLIBXMLPlugin::daeLIBXMLPlugin():topMeta(NULL),database(NULL)
{
	 xmlInitParser();
}

daeLIBXMLPlugin::~daeLIBXMLPlugin()
{
	 xmlCleanupParser();
}

daeInt daeLIBXMLPlugin::setMeta(daeMetaElement *_topMeta)
{
	topMeta = _topMeta;
	return DAE_OK;
}

void daeLIBXMLPlugin::setDatabase(daeDatabase* _database)
{
	database = _database;
}

void daeLIBXMLPlugin::getProgress(daeInt* bytesParsed,
						 daeInt* lineNumber,
						 daeInt* totalBytes,
						 daeBool reset)
{
	// Need to interface this to libxml
	if (reset)
	{
		//daeChunkBuffer::resetProgress();
	}
#if LIBXML_VERSION >= 20620
	if (bytesParsed)
		*bytesParsed= 0; //xmlTextReaderByteConsumed(reader); // Not sure if this is the right data
	if (lineNumber)
		*lineNumber = 0; //xmlTextReaderGetParserLineNumber(reader);
#else
	if (bytesParsed)
		*bytesParsed= 0;
	if (lineNumber)
		*lineNumber = 0;
#endif
	if (totalBytes)
		*totalBytes = 0; // Not available
}
// This function needs to be re-entrant, it can be called recursively from inside of resolveAll
// to load files that the first file depends on.
daeInt daeLIBXMLPlugin::read(daeURI& uri, daeString docBuffer)
{
    // Make sure topMeta has been set before proceeding
	
	if (topMeta == NULL) 
	{
		return DAE_ERR_BACKEND_IO;
	}

	// Generate a version of the URI with the fragment removed

	daeURI fileURI(uri.getURI(),true);

	//check if document already exists
	if ( database->isDocumentLoaded( fileURI.getURI() ) )
	{
		return DAE_ERR_COLLECTION_ALREADY_EXISTS;
	}

	// Create the right type of xmlTextReader on the stack so this function can be re-entrant

	xmlTextReaderPtr reader;

	if(docBuffer)
	{
		// Load from memory (experimental)
#if 0 //debug stuff
		printf("Reading %s from memory buffer\n", fileURI.getURI());
#endif
		reader = xmlReaderForDoc((xmlChar*)docBuffer, fileURI.getURI(), NULL,0);
	}
	else
	{
		// Load from URI
#if 0 //debug stuff
		printf("Opening %s\n", fileURI.getURI());
#endif
		reader = xmlReaderForFile(fileURI.getURI(), NULL,0);
	}

	if(!reader)
	{
		char msg[512];
		sprintf( msg, "Failed to open %s\n", fileURI.getURI() );
		daeErrorHandler::get()->handleError( msg );
		return DAE_ERR_BACKEND_IO;
	}

	// Start parsing the file

	daeElementRef domObject = startParse(topMeta, reader);

	// Parsing done, free the xmlReader and error check to make sure we got a valid DOM back
	
	xmlFreeTextReader(reader);

	if (!domObject)
	{
		char msg[512];
		sprintf(msg,"daeLIBXMLPlugin::read(%s) failed - XML Parse Failed\n",fileURI.getFile());
		daeErrorHandler::get()->handleError( msg );
		return DAE_ERR_BACKEND_IO;
	}

	// Insert the document into the database, the Database will keep a ref on the main dom, so it won't gets deleted
	// until we clear the database

	daeDocument *document = NULL;

	int res = database->insertDocument(fileURI.getURI(),domObject,&document);
	if (res!= DAE_OK)
		return res;

	// Make a vector to store a list of the integration items that need to be processed later
	// postProcessDom will fill this in for us (this should probably not be done in the IOPlugin)
	
	std::vector<INTEGRATION_ITEM> intItems;
	
	//insert the elements into the database, for this DB the elements are the Collada object which have
	//an ID. 
	//this function will fill the _integrationItems array as well
	postProcessDom(document, domObject, intItems);
	database->validate();
	daeElement::resolveAll();

	//create the integration objects
	int size = (int)intItems.size();
	int i;
	for (i=0;i<size;i++)
		intItems[i].intObject->createFromChecked(intItems[i].element);
	
	for (i=0;i<size;i++)
		intItems[i].intObject->fromCOLLADAChecked();

	for (i=0;i<size;i++)
		intItems[i].intObject->fromCOLLADAPostProcessChecked();

	//clear the temporary integration items array
	intItems.clear();

	return DAE_OK;
}
daeElementRef
daeLIBXMLPlugin::startParse(daeMetaElement* thisMetaElement, xmlTextReaderPtr reader)
{
	// The original parsing system would drop everything up to the first element open, usually <COLLADA>
	// This behavior will have to be replicated here till we have someplace to put the headers/comments

	int ret = xmlTextReaderRead(reader);
	if(ret != 1)
	{
		// empty or hit end of file
		return NULL;
	}
	  //printf("xmlTextReaderConstBaseUri is %s\n",xmlTextReaderConstBaseUri(reader));
	  //printf("xmlTextReaderConstNamespaceUri is %s\n",xmlTextReaderConstNamespaceUri(reader));
	  //printf("xmlTextReaderConstPrefix is %s\n",xmlTextReaderConstPrefix(reader));
	  //printf("xmlTextReaderName is %s\n",xmlTextReaderName(reader));

	// Process the current element
	// Skip over things we don't understand
	while(xmlTextReaderNodeType(reader) != XML_READER_TYPE_ELEMENT)
	{
		ret = xmlTextReaderRead(reader);
		if(ret != 1)
			return(NULL);
	}

	// Create the element that we found
	daeElementRef element = thisMetaElement->create((const daeString)xmlTextReaderConstName(reader));
	if(!element)
	{
		char err[512];
		memset( err, 0, 512 );
		const xmlChar * mine =xmlTextReaderConstName(reader);
#if LIBXML_VERSION >= 20620
		sprintf(err,"The DOM was unable to create an element type %s at line %d\nProbably a schema violation.\n", mine,xmlTextReaderGetParserLineNumber(reader));
#else
		sprintf(err,"The DOM was unable to create an element type %s\nProbably a schema violation.\n", mine);
#endif
		daeErrorHandler::get()->handleWarning( err );
		xmlTextReaderNext(reader);
		return NULL;
	}
	int currentDepth = xmlTextReaderDepth(reader);

	//try and read attributes
	readAttributes( element, reader );

#if 1
	//Check COLLADA Version
	if ( strcmp( element->getTypeName(), "COLLADA" ) != 0 ) {
		//invalid root
		daeErrorHandler::get()->handleError("Loading document with invalid root element!");
		return NULL;
	}
	daeURI *xmlns = (daeURI*)(element->getMeta()->getMetaAttribute( "xmlns" )->getWritableMemory( element ));
	if ( strcmp( xmlns->getURI(), COLLADA_NAMESPACE ) != 0 ) {
		//invalid COLLADA version
		daeErrorHandler::get()->handleError("Trying to load an invalid COLLADA version for this DOM build!");
		return NULL;
	}
#endif

	
	ret = xmlTextReaderRead(reader);
	// If we're out of data, return the element
	if(ret != 1)
		return(element);

	// Read all the tags that are part of this tag
	bool trew = true;
	while(trew)
	{
		int thisType = xmlTextReaderNodeType(reader);
		if(thisType == XML_READER_TYPE_ELEMENT)
		{
			// Is the new element at the same depth as this one?
			if(currentDepth == xmlTextReaderDepth(reader))
			{
				// Same depth means the current element ended in a /> so this is a sibling
				// so we return and let our parent process it.
				return(element);
			}
			else
			{
				// The element is a child of this one, so we recurse
				if(!element->placeElement(nextElement(element->getMeta(), reader)))
				{
					char err[512];
					memset( err, 0, 512 );
#if LIBXML_VERSION >= 20620
				sprintf(err,"placeElement failed at line %d\n", xmlTextReaderGetParserLineNumber(reader));
#else
				sprintf(err,"placeElement failed\n");
#endif
				daeErrorHandler::get()->handleWarning(err);
				ret = xmlTextReaderRead(reader);
				if ( ret != 1 ) {
					return element;
				}
				}
			}
		}
		else if(thisType == XML_READER_TYPE_TEXT)
		{
			readValue( element, reader );
		}
		else if(thisType == XML_READER_TYPE_END_ELEMENT)
		{
			// Done with this element so read again and return
			ret = xmlTextReaderRead(reader);
			return(element);
		}
		else
		{	// Skip element types we don't care about
			ret = xmlTextReaderRead(reader);
			// If we're out of data, return the element
			if(ret != 1)
				return(element);
		}
	}
	// Return NULL on an error
	return NULL;
}

void daeLIBXMLPlugin::readAttributes( daeElement *element, xmlTextReaderPtr reader ) {
	// See if the element has attributes
	if(xmlTextReaderHasAttributes(reader))
	{
		// Read in and set all the attributes
		while(xmlTextReaderMoveToNextAttribute(reader))
		{
			daeMetaAttribute *ma = element->getMeta()->getMetaAttribute((const daeString)xmlTextReaderConstName(reader));
			if( ( ma != NULL && ma->getType() != NULL && ma->getType()->getUsesStrings() ) || 
				strcmp(element->getMeta()->getName(), "any") == 0 )
			{
				// String is used as one piece
				if(!element->setAttribute( (const daeString)xmlTextReaderConstName(reader), 
					(const daeString)xmlTextReaderConstValue(reader) ) )
				{
					const xmlChar * attName	 = xmlTextReaderConstName(reader);
					const xmlChar * attValue = xmlTextReaderConstValue(reader);
					char err[512];
					memset( err, 0, 512 );
#if LIBXML_VERSION >= 20620
					sprintf(err,"The DOM was unable to create an attribute %s = %s at line %d\nProbably a schema violation.\n", attName, attValue ,xmlTextReaderGetParserLineNumber(reader));
#else
					sprintf(err,"The DOM was unable to create an attribute %s = %s \nProbably a schema violation.\n", attName, attValue);
#endif
					daeErrorHandler::get()->handleWarning( err );
				}
			}
			else
			{
				// String needs to be broken up into whitespace seperated items.  The "set" methods for numbers are smart enough to
				// grab just the first number in a string, but the ones for string lists require a null terminator between each
				// string.  If this could be fixed we could avoid a copy and memory allocation by using xmlTextReaderConstValue(reader)
				if ( ma == NULL ) {
					const xmlChar * attName	 = xmlTextReaderConstName(reader);
					const xmlChar * attValue = xmlTextReaderConstValue(reader);
					char err[512];
					memset( err, 0, 512 );
#if LIBXML_VERSION >= 20620
					sprintf(err,"The DOM was unable to create an attribute %s = %s at line %d\nProbably a schema violation.\n", attName, attValue ,xmlTextReaderGetParserLineNumber(reader));
#else				
					sprintf(err,"The DOM was unable to create an attribute %s = %s \nProbably a schema violation.\n", attName, attValue);
#endif
					daeErrorHandler::get()->handleWarning( err );
					continue;
				}
				xmlChar* value = xmlTextReaderValue(reader);
				daeChar* current = (daeChar *)value;
				while(*current != 0)
				{
					// !!!GAC NEEDS TO BE CHANGED to use XML standard whitespace parsing
					// Skip leading whitespace
					while(*current == ' ' || *current == '\r' || *current == '\n' || *current == '\t') current++;
					if(*current != 0)
					{
						daeChar* start=current;
						// Find end of string and insert a zero terminator
						while(*current != ' ' && *current != '\r' && *current != '\n' && *current != '\t' && *current != 0) current++;
						if(*current != 0)
						{
							*current = 0;
							current++;
						}
						if(!element->setAttribute( (const daeString)xmlTextReaderConstName(reader), start) )
						{
							const xmlChar * attName	 = xmlTextReaderConstName(reader);
							const xmlChar * attValue = xmlTextReaderConstValue(reader);
							char err[512];
							memset( err, 0, 512 );
#if LIBXML_VERSION >= 20620
							sprintf(err,"The DOM was unable to create an attribute %s = %s at line %d\nProbably a schema violation.\n", attName, attValue ,xmlTextReaderGetParserLineNumber(reader));
#else
							sprintf(err,"The DOM was unable to create an attribute %s = %s \nProbably a schema violation.\n", attName, attValue);
#endif
							daeErrorHandler::get()->handleWarning( err );
						}
					}
				}
				xmlFree(value);
			}
		}
	}
}

void daeLIBXMLPlugin::readValue( daeElement *element, xmlTextReaderPtr reader ) {
	if ( element->getMeta()->getValueAttribute() == NULL ) {
		char err[512];
		memset( err, 0, 512 );
#if LIBXML_VERSION >= 20620
		sprintf(err,"The DOM was unable to set a value for element of type %s at line %d\nProbably a schema violation.\n", element->getTypeName() ,xmlTextReaderGetParserLineNumber(reader));
#else
		sprintf(err,"The DOM was unable to set a value for element of type %s at line %d\nProbably a schema violation.\n", element->getTypeName() );
#endif
		daeErrorHandler::get()->handleWarning( err );
	}
	else if(element->getMeta()->getUsesStringContents())
	{
		// String is used as one piece
		element->getMeta()->getValueAttribute()->set(element,(const daeString)xmlTextReaderConstValue(reader));
	}
	else
	{
		// String needs to be broken up into whitespace seperated items.  The "set" methods for numbers are smart enough to
		// grab just the first number in a string, but the ones for string lists require a null terminator between each
		// string.  If this could be fixed we could avoid a copy and memory allocation by using xmlTextReaderConstValue(reader)
		xmlChar* value = xmlTextReaderValue(reader);
		daeChar* current = (daeChar *)value;
		while(*current != 0)
		{
			// !!!GAC NEEDS TO BE CHANGED to use XML standard whitespace parsing
			// Skip leading whitespace
			while(*current == ' ' || *current == '\r' || *current == '\n' || *current == '\t') current++;
			if(*current != 0)
			{
				daeChar* start=current;
				// Find end of string and insert a zero terminator
				while(*current != ' ' && *current != '\r' && *current != '\n' && *current != '\t' && *current != 0) current++;
				if(*current != 0)
				{
					*current = 0;
					current++;
				}
				element->getMeta()->getValueAttribute()->set(element,start);
				// eat the characters we just read (would be nice if set returned characters used.
			}
		}
		xmlFree(value);
	}
	int ret = xmlTextReaderRead(reader);
	assert(ret==1);
}



daeElementRef
daeLIBXMLPlugin::nextElement(daeMetaElement* thisMetaElement, xmlTextReaderPtr reader)
{
	int ret;
	// Process the current element
	// Skip over things we don't understand
	while(xmlTextReaderNodeType(reader) != XML_READER_TYPE_ELEMENT)
	{
		ret = xmlTextReaderRead(reader);
		if(ret != 1)
			return(NULL);
	}

	// Create the element that we found
	daeElementRef element = thisMetaElement->create((const daeString)xmlTextReaderConstName(reader));
	if(!element)
	{
		const xmlChar * mine =xmlTextReaderConstName(reader);
		char err[512];
		memset( err, 0, 512 );
#if LIBXML_VERSION >= 20620
		sprintf(err,"The DOM was unable to create an element type %s at line %d\nProbably a schema violation.\n", mine,xmlTextReaderGetParserLineNumber(reader));
#else
		sprintf(err,"The DOM was unable to create an element type %s\nProbably a schema violation.\n", mine);
#endif
		daeErrorHandler::get()->handleWarning( err );
		xmlTextReaderNext(reader);
		return NULL;
	}
	int currentDepth = xmlTextReaderDepth(reader);

	//try and read attributes
	readAttributes( element, reader );
	
	ret = xmlTextReaderRead(reader);
	// If we're out of data, return the element
	if(ret != 1)
		return(element);

	// Read all the tags that are part of this tag
	bool trew = true;
	while(trew)
	{
		int thisType = xmlTextReaderNodeType(reader);
		if(thisType == XML_READER_TYPE_ELEMENT)
		{
			// Is the new element at the same depth as this one?
			if(currentDepth == xmlTextReaderDepth(reader))
			{
				// Same depth means the current element ended in a /> so this is a sibling
				// so we return and let our parent process it.
				return(element);
			}
			else
			{
				// The element is a child of this one, so we recurse
				daeElementRef newEl = nextElement(element->getMeta(), reader);
				if( newEl != NULL && !element->placeElement(newEl) )
				{
					char err[512];
					memset( err, 0, 512 );
					sprintf(err,"placeElement failed placing element %s in element %s\n", newEl->getTypeName(), element->getTypeName() );
					daeErrorHandler::get()->handleWarning( err );
					ret = xmlTextReaderRead(reader);
					if ( ret != 1 ) {
						return element;
					}
				}
			}
		}
		else if(thisType == XML_READER_TYPE_TEXT)
		{
			readValue( element, reader );
		}
		else if(thisType == XML_READER_TYPE_END_ELEMENT)
		{
			// Done with this element so read again and return
			ret = xmlTextReaderRead(reader);
			return(element);
		}
		else
		{	// Skip element types we don't care about
			ret = xmlTextReaderRead(reader);
			// If we're out of data, return the element
			if(ret != 1)
				return(element);
		}
	}
	//program will never get here but this line is needed to supress a warning
	return NULL;
}
// postProcessDom traverses all elements below the passed in one and creates a list of all the integration objects.
// this should probably NOT be done in the IO plugin.
void daeLIBXMLPlugin::postProcessDom(daeDocument *document, daeElement* element, std::vector<INTEGRATION_ITEM> &intItems)
{
	// Null element?  Return

	if (!element)
		return;

	//element->setDocument(document);
	// If this element has an integration object, add it to a list so we can process them all in a bunch later

	if (element->getIntObject(daeElement::int_uninitialized))
	{
		INTEGRATION_ITEM item;
		item.element = element;
		item.intObject = element->getIntObject(daeElement::int_uninitialized);
		intItems.push_back(item);
	}

	// Recursively call postProcessDom on all of this element's children
	daeElementRefArray children;
	element->getChildren( children );
	for ( size_t x = 0; x < children.getCount(); x++ ) {
		postProcessDom(document, children.get(x), intItems);
	}
	
	/*if (element->getMeta()->getContents() != NULL) {
		daeMetaElementArrayAttribute *contents = element->getMeta()->getContents();
		for ( int i = 0; i < contents->getCount( element ); i++ ) {
			//array.append( *(daeElementRef*)contents->get( this, i ) );
			daeElementRef elem = *(daeElementRef*)contents->get(element,i);
			postProcessDom(document,elem, intItems);
		}
	}
	else
	{
		daeMetaElementAttributeArray& children = element->getMeta()->getMetaElements();
		int cnt = (int)children.getCount();
		for(int i=0;i<cnt;i++)
		{
			daeMetaElementAttribute* mea = children[i];
			int elemCnt = mea->getCount(element);
			int j;
			for(j=0;j<elemCnt;j++)
			{
				daeElementRef elem = *(daeElementRef*)mea->get(element,j);
				postProcessDom(document,elem, intItems);
			}
		}
	}*/
}

daeInt daeLIBXMLPlugin::write(daeURI *name, daeDocument *document, daeBool replace)
{
	// Make sure database and document are both set
	if (!database)
		return DAE_ERR_INVALID_CALL;
	if(!document)
		return DAE_ERR_COLLECTION_DOES_NOT_EXIST;

	// Extract just the file path from the URI
	daeFixedName finalname;
	if (!name->getPath(finalname,sizeof(finalname)))
	{
		daeErrorHandler::get()->handleError( "can't get path in write\n" );
		return DAE_ERR_BACKEND_IO;
	}

	// If replace=false, don't replace existing files
	if(!replace)
	{
		// Using "stat" would be better, but it's not available on all platforms
		FILE *tempfd = fopen(finalname,"r");
		if(tempfd != NULL)
		{
			// File exists, return error
			fclose(tempfd);
			return DAE_ERR_BACKEND_FILE_EXISTS;
		}
	}

	// Open the file we will write to
	writer = xmlNewTextWriterFilename(name->getURI(), 0);
	if ( !writer ) {
		char msg[512];
		sprintf(msg,"daeLIBXMLPlugin::write(%s) failed\n",name->getURI());
		daeErrorHandler::get()->handleError( msg );
		return DAE_ERR_BACKEND_IO;
	}
	xmlChar indentString[10] = "    ";
	xmlTextWriterSetIndentString( writer, indentString );
	xmlTextWriterSetIndent( writer, 1 );
	xmlTextWriterStartDocument( writer, NULL, NULL, NULL );
	
	writeElement( document->getDomRoot() );
	
	xmlTextWriterEndDocument( writer );
	xmlTextWriterFlush( writer );
	xmlFreeTextWriter( writer );
	return DAE_OK;
}

void daeLIBXMLPlugin::writeElement( daeElement* element )
{
	daeIntegrationObject* _intObject = element->getIntObject();
	daeMetaElement* _meta = element->getMeta();
	if(_intObject)
	{
		// added in response to bug 478
		_intObject->toCOLLADAChecked();
		_intObject->toCOLLADAPostProcessChecked();
	}
	if (!_meta->getIsTransparent() ) {
		if ( element->getElementName() ) {
			xmlTextWriterStartElement(writer, (xmlChar*)element->getElementName());
		}
		else {
            xmlTextWriterStartElement(writer, (xmlChar*)(daeString)_meta->getName());
		}
		daeMetaAttributeRefArray& attrs = _meta->getMetaAttributes();
		
		int acnt = (int)attrs.getCount();
		
		for(int i=0;i<acnt;i++) {
			writeAttribute( attrs[i], element, i );
		}
	}
	daeMetaAttribute* valueAttr = _meta->getValueAttribute();
	if (valueAttr!=NULL)
		writeAttribute(valueAttr, element);
	
	daeElementRefArray children;
	element->getChildren( children );
	for ( size_t x = 0; x < children.getCount(); x++ ) {
		writeElement( children.get(x) );
	}

	/*if (_meta->getContents() != NULL) {
		daeElementRefArray* era = (daeElementRefArray*)_meta->getContents()->getWritableMemory(element);
		int elemCnt = (int)era->getCount();
		for(int i = 0; i < elemCnt; i++) {
			daeElementRef elem = (daeElementRef)era->get(i);
			if (elem != NULL) {
				writeElement( elem );
			}
		}
	}
	else
	{
		daeMetaElementAttributeArray& children = _meta->getMetaElements();
		int cnt = (int)children.getCount();
		for(int i=0;i<cnt;i++) {
			daeMetaElement *type = children[i]->getElementType();
			if ( !type->getIsAbstract() ) {
				for (int c = 0; c < children[i]->getCount(element); c++ ) {
					writeElement( *(daeElementRef*)children[i]->get(element,c) );
				}
			}
		}
	}*/
	if (!_meta->getIsTransparent() ) {
		xmlTextWriterEndElement(writer);
	}
}

#define TYPE_BUFFER_SIZE 1024*1024

/*void daeLIBXMLPlugin::writeAttribute( daeMetaAttribute* attr, daeElement* element, daeInt attrNum )
{
	static daeChar atomicTypeBuf[TYPE_BUFFER_SIZE];
	
	if (element == NULL)
		return;
	if ( attr->getCount(element) == 0 ) {
		//we don't have a value if its required print it empty else just skip
		if ( attr->getIsRequired() ) {
			xmlTextWriterStartAttribute( writer, (xmlChar*)(daeString)attr->getName() );
			xmlTextWriterEndAttribute( writer );
		}
		return;
	}
	else if ( attr->getCount(element) == 1 ) { 
		//single value or an array of a single value
		char* elemMem = attr->get(element, 0);

		// !!!GAC recoded the suppression logic so you could enable suppressions individually
		if(!attr->getIsRequired())
		{
			// This attribute was not required and might get suppressed
			int typeSize = attr->getType()->getSize();
			if(attr->getDefault() != NULL)
			{
				#if 1
				// The attribute has a default, convert the default to binary and suppress 
				// output of the attribute if the value matches the default.
				// DISABLE THIS CODE IF YOU WANT DEFAULT VALUES TO ALWAYS EXPORT
				if(typeSize >= TYPE_BUFFER_SIZE)
				{
					char msg[512];
					sprintf(msg,
							"daeMetaAttribute::print() - buffer too small for default value of %s in %s\n",
							(daeString)attr->getName(),(daeString)attr->getContainer()->getName());
					daeErrorHandler::get()->handleError( msg );
					return;
				}
				attr->getType()->stringToMemory((daeChar*)attr->getDefault(),atomicTypeBuf);
				if(memcmp(atomicTypeBuf,elemMem,typeSize) == 0)
					return;
				#endif
			}
			else
			{
				#if 0
				// The attribute does not have a default, suppress it if its value is all zeros (binary)
				// DISABLE THIS CODE IF YOU WANT OPTIONAL ATTRIBUTES THAT HAVE A VALUE OF ZERO TO EXPORT
				// Disabling this code may cause some unused attributes to be exported if _isValid is not
				// enabled and properly used.
				int i;
				for(i=0; i<typeSize;i++)
				{
					if(elemMem[i] != 0)
						break;
				}
				if(i == typeSize && attr->getContainer()->getValueAttribute() != attr && 
					attr->getType()->getTypeEnum() != daeAtomicType::BoolType &&
					attr->getType()->getTypeEnum() != daeAtomicType::EnumType )
					return;
				#endif
				if ( attrNum != -1 && !element->isAttributeSet( attr->getName() ) ) {
					return;
				}
			}
		}

		// Convert the attribute to a string

		if (attr->getType()->memoryToString(elemMem, atomicTypeBuf,	TYPE_BUFFER_SIZE)== false) {
			char msg[512];
			sprintf(msg,
					"daeMetaAttribute::print() - buffer too small for %s in %s\n",
					(daeString)attr->getName(),(daeString)attr->getContainer()->getName());
			daeErrorHandler::get()->handleError( msg );
		}
					
		// Suppress attributes that convert to an empty string.

		if (strlen(atomicTypeBuf) == 0)
			return;

		// Is this a value attribute or something else?

		if (attr->getContainer()->getValueAttribute() == attr)
		{
			// Always export value attributes
			xmlTextWriterWriteString( writer, (xmlChar*)atomicTypeBuf);
		}
		else
		{
			// Suppress attributes not marked as containing valid values
			// DO NOT TURN THIS ON TILL ALL USER CODE HAS BEEN CHANGED TO SET _isValid OR
			// ATTRIBUTES THAT WERE CREATED/SET BY USER CODE MAY NOT EXPORT.
			#if 0
			// NOTE: even if a value is marked REQUIRED by the schema, if _isValid isn't true it means
			// the value in this parameter was never set and may be garbage, so we suppress it even though it is required.
			// To change this add && !_isRequired to the expression below.
			if(!attr->getIsValid() )
				return;
			#endif
			// Export the attribute name and value
			xmlTextWriterWriteAttribute( writer, (xmlChar*)(daeString)attr->getName(), (xmlChar*)atomicTypeBuf);
		}
	}
	else {
		if (attr->getContainer()->getValueAttribute() != attr)
		{
			xmlTextWriterStartAttribute( writer, (xmlChar*)(daeString)attr->getName() );
		}
		for( int i = 0; i < attr->getCount(element); i++ ) {
			char* elemMem = attr->get(element, i);
			if (attr->getType()->memoryToString(elemMem, atomicTypeBuf,	TYPE_BUFFER_SIZE)== false) 
			{
				char msg[512];
				sprintf(msg,
						"daeMetaArrayAttribute::print() - buffer too small for %s in %s\n",
						(daeString)attr->getName(),(daeString)attr->getContainer()->getName());
				daeErrorHandler::get()->handleError( msg );
			}
			xmlTextWriterWriteFormatString( writer, "%s ", (xmlChar*)atomicTypeBuf );
		}
		if (attr->getContainer()->getValueAttribute() != attr)
		{
			xmlTextWriterEndAttribute( writer );
		}
	}
}*/

void daeLIBXMLPlugin::writeAttribute( daeMetaAttribute* attr, daeElement* element, daeInt attrNum )
{
	static daeChar atomicTypeBuf[TYPE_BUFFER_SIZE+1];
	daeChar *buf = atomicTypeBuf;
	daeUInt bufSz = TYPE_BUFFER_SIZE;

	size_t valCount = attr->getCount(element);
	//default values will not check correctly if they are arrays. Luckily there is only one array attribute in COLLADA.
	//don't write if !required and is set && is default

	int typeSize = attr->getType()->getSize();
	if ( typeSize >= TYPE_BUFFER_SIZE )
	{
		char msg[512];
		sprintf(msg,
			"daeMetaAttribute::print() - buffer too small for %s in %s\n",
			(daeString)attr->getName(),(daeString)attr->getContainer()->getName());
		daeErrorHandler::get()->handleError( msg );
		return;
	}

	if ( !attr->isArrayAttribute() && ( attr->getType()->getTypeEnum() == daeAtomicType::StringRefType || 
		 attr->getType()->getTypeEnum() == daeAtomicType::TokenType ) &&
		 *(char**)attr->getWritableMemory( element ) != NULL )
	{
		daeUInt sz = (daeUInt)strlen( *(char**)attr->getWritableMemory( element ) ) +1;
		if ( bufSz > TYPE_BUFFER_SIZE ) {
			buf = new char[ bufSz ];
			bufSz = sz;
		}
	}

	//always print value
	if ( attrNum != -1 )
	{
		//not value attribute
		if ( !attr->getIsRequired() )
		{
			//not required
			if ( !element->isAttributeSet( attr->getName() ) )
			{
				//early out if !value && !required && !set
				return;
			}
			
			//is set
			//check for default suppression
			if ( attr->getDefault() != NULL )
			{
				//has a default value
				attr->getType()->stringToMemory( (daeChar*)attr->getDefault(), buf );
				char* elemMem = attr->get( element, 0 );
				if( memcmp( buf, elemMem, typeSize ) == 0 )
				{
					//is the default value so exit early
					return;
				}
			}

			//not default so print
			xmlTextWriterStartAttribute( writer, (xmlChar*)(daeString)attr->getName() );
		}
		else {
			//print it because its required
			xmlTextWriterStartAttribute( writer, (xmlChar*)(daeString)attr->getName() );
		}
	}
	if (valCount>0) 
	{
		//do val 0 first then space and the rest of the vals.
		char* elemMem = attr->get( element, 0 );
		attr->getType()->memoryToString( elemMem, buf, bufSz );
		if ( buf[0] != 0 ) //null string check
		{
			xmlTextWriterWriteString( writer, (xmlChar*)buf );
		}

		*buf = ' ';
		for( size_t i = 1; i < valCount; i++ ) 
		{
			elemMem = attr->get( element, (int)i );
			attr->getType()->memoryToString( elemMem, buf+1, bufSz );
			xmlTextWriterWriteString( writer, (xmlChar*)buf );
		}
	}
	if ( attrNum != -1 )
	{
		xmlTextWriterEndAttribute( writer );
	}
	if ( buf != atomicTypeBuf )
	{
		delete[] buf;
	}
}

