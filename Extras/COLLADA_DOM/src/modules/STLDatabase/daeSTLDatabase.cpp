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
	std::vector<DAE_STL_DATABASE_CELL>::iterator it = elements.begin();
	daeUInt count = 0;
	while (it != elements.end())
	{
		DAE_STL_DATABASE_CELL &tmp = (*it);
		std::vector<DAE_STL_DATABASE_CELL>::iterator it_up = std::upper_bound(it,elements.end(),tmp,daeSTLDatabaseTypeLess());
		it = it_up;
		count++;
	}
	return count;
}


daeString daeSTLDatabase::getTypeName(daeUInt index)
{
	validate();
	std::vector<DAE_STL_DATABASE_CELL>::iterator it = elements.begin();
	unsigned int count = 0;
	while (it != elements.end() && count<index)
	{
		DAE_STL_DATABASE_CELL &tmp = (*it);
		std::vector<DAE_STL_DATABASE_CELL>::iterator it_up = std::upper_bound(it,elements.end(),tmp,daeSTLDatabaseTypeLess());
		it = it_up;
		count++;
	}
	if (it != elements.end())
		return (*it).type;
	else
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
	dom->setDocument(newDocument);
	// Set and resolve the document URI
	newDocument->getDocumentURI()->setURI(name);
	newDocument->getDocumentURI()->validate();
	insertElement( newDocument, dom );
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
	// Copy the elements that are not in this collection into 
	// a new list.
	std::vector<DAE_STL_DATABASE_CELL> newElements;
	newElements.reserve(elements.size());

	std::vector<DAE_STL_DATABASE_CELL>::iterator iter =	elements.begin();
	while ( iter != elements.end() ) {
		if ( (*iter).document != document ) {
			newElements.push_back(*iter);
		}
		iter++;
	}
	// Replace our existing element list with the new one (that is missing 
	// the elements from this collection).
	elements = newElements;

	//remove the document from its vector
	std::vector<daeDocument*>::iterator iter2 = documents.begin();
	while ( iter2 != documents.end() ) {
		if ( (*iter2) == document ) {
			iter2 = documents.erase(iter2);
		}
		else {
            iter2++;
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

	if ((element->getMeta() != NULL) &&
		(!element->getMeta()->getIsTrackableForQueries()))
		return DAE_OK;
	
	DAE_STL_DATABASE_CELL tmp;
	tmp.document = document;
	tmp.name = element->getID();
	if (tmp.name == NULL)
		tmp.name = ""; //static string in the executable
	tmp.type = element->getTypeName();
	tmp.element = element;

	std::vector<DAE_STL_DATABASE_CELL>::iterator it = std::upper_bound(elements.begin(), elements.end(), tmp, daeSTLDatabaseLess());
	elements.insert(it, tmp);
	
	//insert into IDMap if element has an ID. IDMap is used to speed up URI resolution
	if ( element->getID() != NULL ) {
		elementsIDMap.insert( std::make_pair( std::string( element->getID() ), tmp ) );
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
	std::vector<DAE_STL_DATABASE_CELL>::iterator iter = elements.begin();
	while ( iter != elements.end() ) {
		if ( (*iter).element == element ) {
			elements.erase(iter);
			break;
		}
		iter++;
	} 

	if ( element->getID() != NULL ) {
		std::pair< std::multimap<std::string, DAE_STL_DATABASE_CELL>::iterator, std::multimap<std::string, DAE_STL_DATABASE_CELL>::iterator> range;
		range = elementsIDMap.equal_range( std::string( element->getID() ) );
        std::multimap<std::string, DAE_STL_DATABASE_CELL>::iterator iter = range.first;
		while( iter != range.second ) {
			if ( (*iter).second.element == element ) {
				elementsIDMap.erase( iter );
				break;
			}
			iter++;
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
	// This sorts the vector if necessary
	validate(); 
	
	// If none of the search keys was specified, return the total element count in the database

	if ( !name && !type && !file ) 
	{
		return (daeUInt)elements.size();
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
			std::pair< std::multimap< std::string, DAE_STL_DATABASE_CELL >::iterator, std::multimap< std::string, DAE_STL_DATABASE_CELL >::iterator> range;
			range = elementsIDMap.equal_range( std::string( name ) );
			std::multimap< std::string, DAE_STL_DATABASE_CELL >::iterator i = range.first;
			while ( i != range.second )
			{
				DAE_STL_DATABASE_CELL e = (*i).second;
				if ( col == e.document && !strcmp(name, e.name) )
				{
					count++;
				}
				++i;
			}
			return count;
		}
		else 
		{ 
			//no file specified
			std::pair< std::multimap< std::string, DAE_STL_DATABASE_CELL >::iterator, std::multimap< std::string, DAE_STL_DATABASE_CELL >::iterator> range;
			range = elementsIDMap.equal_range( std::string( name ) );
			std::multimap< std::string, DAE_STL_DATABASE_CELL >::iterator i = range.first;
			while ( i != range.second )
			{
				++i;
				count++;
			}
			return count;
		}
	}

	std::pair< std::vector<DAE_STL_DATABASE_CELL>::iterator, std::vector<DAE_STL_DATABASE_CELL>::iterator> range;
	int sz = 0;
	// The database is sorted by type so if type was one of the keys, find the range of items with that type
	if ( type )	
	{ 
		// if a type was specified, range = elements of that type
		DAE_STL_DATABASE_CELL a;
		a.type = type;
		a.name = NULL;
		a.document = NULL;
		range = std::equal_range(elements.begin(),elements.end(),a,daeSTLDatabaseTypeLess());
		sz = (int)(range.second - range.first);
	}
	else 
	{ 
		//no type specified, range = all elements
		range.first = elements.begin();
		range.second = elements.end();
		sz = (int)elements.size();
	}
	
	//no name specified
	if ( file ) 
	{ 
		// If a document URI was a search key (in file) resolve it to a text URI with no fragment
		daeURI tempURI(file,true);
		daeDocument *col = getDocument( tempURI.getURI() );
		if ( col == NULL ) {
			return 0;
		}
		//a document was specified
		int count = 0;
		std::vector< DAE_STL_DATABASE_CELL >::iterator i = range.first;
		while ( i != range.second )
		{
			DAE_STL_DATABASE_CELL e = *(i);
			if( col == e.document )
			{
				count++;
			}
			++i;
		}
		return count;
	}
	//if you get to this point only a type was specified
	return sz;
}

daeInt daeSTLDatabase::getElement(daeElement** pElement,daeInt index,daeString name,daeString type,daeString file)
{	
	// this sorts the vector if necessary
	validate(); 
	
	// If the index is out of range, there can be no match
	if ( index < 0 || index >= (int)elements.size() ) 
	{
		return DAE_ERR_QUERY_NO_MATCH;
	}

	// If no name, type or file was specified we return the element at "index"
	if ( !name && !type && !file ) 
	{
		*pElement = elements[index].element;
		return DAE_OK;
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
				return DAE_ERR_QUERY_NO_MATCH;
			}
			//a document was specified
			std::pair< std::multimap< std::string, DAE_STL_DATABASE_CELL >::iterator, std::multimap< std::string, DAE_STL_DATABASE_CELL >::iterator> range;
			range = elementsIDMap.equal_range( std::string( name ) );
			std::multimap< std::string, DAE_STL_DATABASE_CELL >::iterator i = range.first;
			while ( i != range.second )
			{
				DAE_STL_DATABASE_CELL e = (*i).second;
				if ( col == e.document && !strcmp(name, e.name) )  
				{
					if ( count == index )
					{
						*pElement = e.element;
						return DAE_OK;
					}
					count++;
				}
				++i;
			}
			return DAE_ERR_QUERY_NO_MATCH;
		}
		else 
		{ 
			//no document specified
			std::pair< std::multimap< std::string, DAE_STL_DATABASE_CELL >::iterator, std::multimap< std::string, DAE_STL_DATABASE_CELL >::iterator> range;
			range = elementsIDMap.equal_range( std::string( name ) );
			std::multimap< std::string, DAE_STL_DATABASE_CELL >::iterator i = range.first;
			while ( i != range.second )
			{
				DAE_STL_DATABASE_CELL e = (*i).second;
				if ( count == index ) 
				{
					*pElement = e.element;
					return DAE_OK;
				}
				count++;
				++i;
			}
			return DAE_ERR_QUERY_NO_MATCH;
		}
	}
	
	std::pair< std::vector<DAE_STL_DATABASE_CELL>::iterator, std::vector<DAE_STL_DATABASE_CELL>::iterator> range;
	int sz = 0;
	if ( type )	
	{ 
		//a type was specified, range = elements of that type
		DAE_STL_DATABASE_CELL a;
		a.type = type;
		a.name = NULL;
		a.document = NULL;
		range = std::equal_range(elements.begin(),elements.end(),a,daeSTLDatabaseTypeLess());
		sz = (int)(range.second - range.first);
		if ( index >= sz ) 
		{
			return DAE_ERR_QUERY_NO_MATCH;
		}
	}
	else 
	{ 
		//no type specified, range = all elements
		range.first = elements.begin();
		range.second = elements.end();
		sz = (int)elements.size();
	}


	//no name specified
	if ( file ) 
	{ 
		// If a document URI was a search key (in file) resolve it to a text URI with no fragment
		daeURI tempURI(file,true);
		daeDocument *col = getDocument( tempURI.getURI() );
		if ( col == NULL ) {
			return DAE_ERR_QUERY_NO_MATCH;
		}
		//a document was specified
		int count = 0;
		std::vector< DAE_STL_DATABASE_CELL >::iterator i = range.first;
		while ( i != range.second )
		{
			DAE_STL_DATABASE_CELL e = *(i);
			if( col == e.document ) 
			{
				if( count == index ) 
				{
					*pElement = e.element;
					return DAE_OK;
				}
				count++;
			}
			++i;
		}
		return DAE_ERR_QUERY_NO_MATCH;
	}

	//if you get to this point only a type was specified
	*pElement = (*(range.first+index)).element;
	return DAE_OK;
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
			//removeDocument( tmp );
			//insertDocument( tmp );
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

