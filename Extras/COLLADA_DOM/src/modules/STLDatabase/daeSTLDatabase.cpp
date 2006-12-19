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

#include <modules/daeSTLDatabase.h>
#include <dae/daeMetaElement.h>

daeSTLDatabase::daeSTLDatabase()
{}

daeSTLDatabase::~daeSTLDatabase()
{
	clear();
}

daeInt daeSTLDatabase::setMeta(daeMetaElement *_topMeta)
{
	topMeta = _topMeta;
	return DAE_OK;
}

daeBool
daeSTLDatabase::isDocumentLoaded(daeString name)
{
	daeDocument* document = getDocument(name);
	if(document)
		return(true);
	else
		return(false);
}

// Element Types of all Elements
daeUInt daeSTLDatabase::getTypeCount()
{
	validate();
	return (daeUInt)elements.size();
}


daeString daeSTLDatabase::getTypeName(daeUInt index)
{
	validate();
	daeUInt count = 0;
	
	std::map<std::string, std::vector< daeElement* > >::iterator iter = elements.begin();
	std::map<std::string, std::vector< daeElement* > >::iterator end = elements.end();
	while ( iter != end )
	{
		if ( count == index )
		{
			return (*iter).first.c_str();
		}
		++count;
		++iter;
	}

	return NULL;
}

// Documents
daeInt daeSTLDatabase::insertDocument(const char *name, daeElement* dom, daeDocument** document)
{
	return createDocument( name, dom, document );
}
daeInt daeSTLDatabase::createDocument(const char *name, daeElement* dom, daeDocument** document)
{
	// If a document already exists with the same name, error
	if(isDocumentLoaded(name))
	{
		if (document)
			*document = NULL;
		return DAE_ERR_COLLECTION_ALREADY_EXISTS;
	}
	
	// Make a new document
	daeDocument *newDocument = new daeDocument;
	// Create a Reference on the domCOLLADA passed into us
	newDocument->setDomRoot(dom);
	// Set the domCollada's document to this one
	//dom->setDocument(newDocument);
	// Set and resolve the document URI
	newDocument->getDocumentURI()->setURI(name);
	newDocument->getDocumentURI()->validate();
	//insertElement( newDocument, dom );
	newDocument->setModified(true);
	// Push the connection into the database
	documents.push_back(newDocument);
	
	if (document)
		*document = newDocument;

	//check if an already loaded document has external references to this one and resolve them.
	for ( unsigned int i = 0; i < documents.size(); i++ ) {
		if ( documents[i] != newDocument ) {
			documents[i]->resolveExternals( name );
		}
	}
	
	return DAE_OK;
}
// !!!GAC revised version of insertDocument, creates a domCollada and fills it in for you.
daeInt daeSTLDatabase::insertDocument(const char *name, daeDocument** document)
{
	return createDocument( name, document );
}
daeInt daeSTLDatabase::createDocument(const char *name, daeDocument** document)
{

	// If a document already exists with the same name, error
	if(isDocumentLoaded(name))
	{
		if (document)
			*document = NULL;
		return DAE_ERR_COLLECTION_ALREADY_EXISTS;
	}
	// Make the new document
	daeDocument *newDocument = new daeDocument;
	// Make a domCOLLADA to be the root of this new document (this makes a reference so the domCOLLADA won't delete itself
	daeElementRef myCOLLADA = topMeta->create();
	// Set the domCollada's document to this one
	myCOLLADA->setDocument(newDocument);
	newDocument->setDomRoot(myCOLLADA);
	// Set and resolve the document URI
	newDocument->getDocumentURI()->setURI(name);
	newDocument->getDocumentURI()->validate();

	newDocument->setModified(true);
	// Add this document to the list.
	documents.push_back(newDocument);
	// If the user gave us a place to put the document, send it back to them.
	if (document)
		*document = newDocument;
	
	return DAE_OK;
}

daeInt daeSTLDatabase::insertDocument( daeDocument *c ) {
	documents.push_back(c);
	insertElement( c, c->getDomRoot() );
	return DAE_OK;
}

daeInt daeSTLDatabase::removeDocument(daeDocument *document)
{	
	std::vector< daeDocument* >::iterator iter = documents.begin();
	while ( iter != documents.end() ) {
		if ( (*iter) == document ) {
			//delete all of its children
			removeElement( *iter, (*iter)->getDomRoot() );
            delete *iter; // sthomas (see bug 1466019)
			iter = documents.erase(iter);
		}
		else {
            iter++;
		}
	}
	return DAE_OK;
}

daeUInt daeSTLDatabase::getDocumentCount()
{
	return (daeUInt)documents.size();
}

daeDocument* daeSTLDatabase::getDocument(daeUInt index)
{
	if (index<documents.size())
		return (documents[index]);
	else
		return NULL;
}

daeDocument* daeSTLDatabase::getDocument(daeString name)
{
	// Normalize the input string to an absolute URI with no fragment

	daeURI tempURI(name, true);
	daeString targetURI = tempURI.getURI();

	// Try to find a document that matches

	daeDocument *document;
	int documentCount	= getDocumentCount();
	for (int i=0;i<documentCount;i++)
	{
		document = getDocument(i);
		if(strcmp(document->getDocumentURI()->getURI(), targetURI)==0)
			return(document);
	}
	return(NULL);
}

daeString daeSTLDatabase::getDocumentName(daeUInt index)
{
	if (index<documents.size())
		return getDocument(index)->getDocumentURI()->getURI();
	else
		return NULL;
}

// Elements
daeInt daeSTLDatabase::insertElement(daeDocument* document,daeElement* element)
{
	insertChildren( document, element );
	
	std::map<std::string, std::vector< daeElement* > >::iterator iter = elements.find( std::string( element->getTypeName() ) );
	if ( iter != elements.end() )
	{
		(*iter).second.push_back( element );
	}
	else
	{
		std::vector< daeElement* > vec;
		vec.push_back( element );
		elements.insert( std::make_pair( std::string( element->getTypeName() ), vec ) );
	}

	//insert into IDMap if element has an ID. IDMap is used to speed up URI resolution
	if ( element->getID() != NULL ) {
		elementsIDMap.insert( std::make_pair( std::string( element->getID() ), element ) );
	}

	return DAE_OK;
}

daeInt daeSTLDatabase::insertChildren( daeDocument *c, daeElement *element )
{
	daeElementRefArray era;
	element->getChildren( era );
	for ( unsigned int i = 0; i < era.getCount(); i++ ) {
		insertElement( c, era[i] );
	}
	return DAE_OK;
}

daeInt daeSTLDatabase::removeElement(daeDocument* document,daeElement* element)
{
	if ( !element ) {
		return DAE_ERR_INVALID_CALL;
	}
	removeChildren( document, element );
	
	std::map<std::string, std::vector< daeElement* > >::iterator iter = elements.find( std::string( element->getTypeName() ) );
	if ( iter != elements.end() )
	{
		std::vector< daeElement* > &vec = (*iter).second;
		std::vector< daeElement* >::iterator i = vec.begin();
		std::vector< daeElement* >::iterator end = vec.end();
		while( i != end )
		{
			if ( (*i) == element )
			{
				vec.erase( i );
				break;
			}
			++i;
		}
	}

	if ( element->getID() != NULL ) {
		std::pair< std::multimap<std::string, daeElement* >::iterator, std::multimap<std::string, daeElement* >::iterator> range;
		range = elementsIDMap.equal_range( std::string( element->getID() ) );
        std::multimap<std::string, daeElement* >::iterator iter = range.first;
		while( iter != range.second ) {
			if ( (*iter).second == element ) {
				elementsIDMap.erase( iter );
				break;
			}
			++iter;
		}
	}
	return DAE_OK;
}

daeInt daeSTLDatabase::removeChildren( daeDocument *c, daeElement *element )
{
	daeElementRefArray era;
	element->getChildren( era );
	for ( unsigned int i = 0; i < era.getCount(); i++ ) {
		removeElement( c, era[i] );	
	}
	return DAE_OK;
}


daeInt daeSTLDatabase::clear()
{
	elements.clear();
	elementsIDMap.clear();
	int i;
	for (i=0;i<(int)documents.size();i++)
		delete documents[i];
	documents.clear(); //this will free the daeElement
	return DAE_OK;
}

daeUInt daeSTLDatabase::getElementCount(daeString name,daeString type,daeString file)
{	
	validate(); 
	
	// If none of the search keys was specified, return the total element count in the database
	if ( !name && !type && !file ) 
	{
		daeUInt count = 0;
		std::map< std::string, std::vector< daeElement*> >::iterator iter = elements.begin();
		std::map< std::string, std::vector< daeElement*> >::iterator end = elements.end();
		while( iter != end )
		{
			count += (daeUInt)(*iter).second.size();
			++iter;
		}
		return count;
	}

	if ( name ) 
	{ 
		// name specified
		int count = 0;
		if ( file ) 
		{ 
			// If a document URI was a search key (in file) resolve it to a text URI with no fragment
			daeURI tempURI(file,true);
			daeDocument *col = getDocument( tempURI.getURI() );
			if ( col == NULL ) {
				return 0;
			}
			// a document was specified
			std::pair< std::multimap< std::string, daeElement* >::iterator, std::multimap< std::string, daeElement* >::iterator > range;
			range = elementsIDMap.equal_range( std::string( name ) );
			std::multimap< std::string, daeElement* >::iterator i = range.first;
			while ( i != range.second )
			{
				if ( col == (*i).second->getDocument() )
				{
					count++;
				}
				++i;
			}
			return count;
		}
		else 
		{ 
			//no file specified - just name
			return (daeUInt)elementsIDMap.count( std::string( name ) );
		}
	}

	if ( type ) 
	{ 
		// type specified
		std::map< std::string, std::vector< daeElement*> >::iterator iter = elements.find( std::string( type ) );
		if ( iter == elements.end() )
		{
			return 0;
		}

		int count = 0;
		if ( file ) 
		{ 
			// If a document URI was a search key (in file) resolve it to a text URI with no fragment
			daeURI tempURI(file,true);
			daeDocument *col = getDocument( tempURI.getURI() );
			if ( col == NULL ) {
				return 0;
			}
			// a document was specified
			std::vector< daeElement* > &vec = (*iter).second;
			std::vector< daeElement* >::iterator i = vec.begin();
			std::vector< daeElement* >::iterator end = vec.end();
			while( i != end )
			{
				if ( col == (*i)->getDocument() )
				{
					++count;
				}
				++i;
			}
			return count;
		}
		else 
		{ 
			//no file specified - just type
			return (daeUInt)(*iter).second.size();
		}
	}

	//if you get here only a file was specified
	daeURI tempURI(file,true);
	daeDocument *col = getDocument( tempURI.getURI() );
	if ( col == NULL ) {
		return 0;
	}
	//a document was specified
	int count = 0;
	std::map< std::string, std::vector< daeElement*> >::iterator iter = elements.begin();
	std::map< std::string, std::vector< daeElement*> >::iterator end = elements.end();
	while( iter != end )
	{
		std::vector< daeElement* > &vec = (*iter).second;
		std::vector< daeElement* >::iterator i = vec.begin();
		std::vector< daeElement* >::iterator end2 = vec.end();
		while( i != end2 )
		{
			if( col == (*i)->getDocument() )
			{
				++count;
			}
			++i;
		}
		++iter;
	}
	return count;

}

daeInt daeSTLDatabase::getElement(daeElement** pElement,daeInt index,daeString name,daeString type,daeString file)
{	
	// this sorts the vector if necessary
	validate(); 
	
	// If the index is out of range, there can be no match
	if ( index < 0 ) 
	{
		return DAE_ERR_QUERY_NO_MATCH;
	}

	// If no name, type or file was specified we return the element at "index" - SLOW
	if ( !name && !type && !file ) 
	{
		daeUInt count = 0;
		std::map< std::string, std::vector< daeElement*> >::iterator iter = elements.begin();
		std::map< std::string, std::vector< daeElement*> >::iterator end = elements.end();
		while( iter != end )
		{
			count += (daeUInt)(*iter).second.size();
			if ( (daeInt)count > index )
			{
				*pElement = (*iter).second[index - (count - (*iter).second.size())] ;
				return DAE_OK;
			}
			++iter;
		}
		return DAE_ERR_QUERY_NO_MATCH;
	}
	
	if ( name ) 
	{ 
		//name specified
		int count = 0;
		if ( file ) 
		{ 
			// If a document URI was a search key (in file) resolve it to a text URI with no fragment
			daeURI tempURI(file,true);
			daeDocument *col = getDocument( tempURI.getURI() );
			if ( col == NULL ) {
				*pElement = NULL;
				return DAE_ERR_QUERY_NO_MATCH;
			}
			//a document was specified
			std::pair< std::multimap< std::string, daeElement* >::iterator, std::multimap< std::string, daeElement* >::iterator> range;
			range = elementsIDMap.equal_range( std::string( name ) );
			std::multimap< std::string, daeElement* >::iterator i = range.first;
			while ( i != range.second )
			{
				if ( col == (*i).second->getDocument() )
				{
					if ( count == index )
					{
						*pElement = (*i).second;
						return DAE_OK;
					}
					count++;
				}
				++i;
			}
			*pElement = NULL;
			return DAE_ERR_QUERY_NO_MATCH;
		}
		else 
		{ 
			//no document specified
			std::multimap< std::string, daeElement* >::iterator i = elementsIDMap.find( std::string( name ) );
			if ( index > (daeInt)elementsIDMap.count( std::string( name ) ) || i == elementsIDMap.end() )
			{
				*pElement = NULL;
				return DAE_ERR_QUERY_NO_MATCH;
			}
			for ( int x = 0; x < index; x++ )
			{
				++i;
			}
			*pElement = i->second;
			return DAE_OK;
		}
	}
	
	if ( type ) 
	{ 
		std::map< std::string, std::vector< daeElement*> >::iterator iter = elements.find( std::string( type ) );
		if ( iter == elements.end() )
		{
			*pElement = NULL;
			return DAE_ERR_QUERY_NO_MATCH;
		}
		//type specified
		int count = 0;
		if ( file ) 
		{ 
			// If a document URI was a search key (in file) resolve it to a text URI with no fragment
			daeURI tempURI(file,true);
			daeDocument *col = getDocument( tempURI.getURI() );
			if ( col == NULL ) {
				return DAE_ERR_QUERY_NO_MATCH;
			}
			//a document was specified
			// a document was specified
			std::vector< daeElement* > &vec = (*iter).second;
			std::vector< daeElement* >::iterator i = vec.begin();
			std::vector< daeElement* >::iterator end = vec.end();
			while( i != end )
			{
				if ( col == (*i)->getDocument() )
				{
					if ( count == index )
					{
						*pElement = (*i);
						return DAE_OK;
					}
					++count;
				}
				++i;
			}
			return DAE_ERR_QUERY_NO_MATCH;
		}
		else 
		{ 
			//no document specified
			if ( index >= (daeInt)(*iter).second.size() )
			{
				*pElement = NULL;
				return DAE_ERR_QUERY_NO_MATCH;
			}
			*pElement = (*iter).second[index];
			return DAE_OK;
		}
	}

	//if you get here only the file was specified - SLOW
	daeURI tempURI(file,true);
	daeDocument *col = getDocument( tempURI.getURI() );
	if ( col == NULL ) {
		return DAE_ERR_QUERY_NO_MATCH;
	}
	//a document was specified
	int count = 0;
	std::map< std::string, std::vector< daeElement*> >::iterator iter = elements.begin();
	std::map< std::string, std::vector< daeElement*> >::iterator end = elements.end();
	while( iter != end )
	{
		std::vector< daeElement* > &vec = (*iter).second;
		std::vector< daeElement* >::iterator i = vec.begin();
		std::vector< daeElement* >::iterator end2 = vec.end();
		while( i != end2 )
		{
			if( col == (*i)->getDocument() ) 
			{
				if( count == index ) 
				{
					*pElement = (*i);
					return DAE_OK;
				}
				++count;
			}
			++i;
		}
		++iter;
	}
	return DAE_ERR_QUERY_NO_MATCH;

}

// Generic Query
daeInt daeSTLDatabase::queryElement(daeElement** pElement, daeString genericQuery)
{
	(void)pElement; 
	(void)genericQuery; 
	return DAE_ERR_NOT_IMPLEMENTED;
}

void daeSTLDatabase::validate()
{
	for( unsigned int i = 0; i < documents.size(); i++ ) {
		if (documents[i]->getModified() ) {
			daeDocument *tmp = documents[i];
			const daeElementRefArray &rea = tmp->getRemovedArray();
			for ( unsigned int x = 0; x < rea.getCount(); x++ ) {
				removeElement( tmp, rea[x] );
			}

			const daeElementRefArray &iea = tmp->getInsertedArray();
			for ( unsigned int x = 0; x < iea.getCount(); x++ ) {
				insertElement( tmp, iea[x] );
			}
			
			tmp->setModified(false);
		}
	}
}

