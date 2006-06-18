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

#ifndef __DAE_DATABASE__
#define __DAE_DATABASE__

#include <dae/daeTypes.h>
#include <dae/daeElement.h>
#include <dae/daeURI.h>
#include <dae/daeDocument.h>


/**
 * The @c daeDatabase class defines the COLLADA runtime database interface.
 */
class daeDatabase
{
public:
	/**
	*  Destructor.
	 */
	virtual ~daeDatabase() {}
	
	/** @name Documents */
	//@{
	/**
	* Creates a new document, defining its root as the <tt><i>dom</i></tt> object; returns an error if the document name already exists.
	* @param name Name of the new document, must be a valid URI.
	* @param dom Existing @c domCOLLADA root element of the document
	* @param document Pointer to a @c daeDocument pointer that receives the document created 
	* @return Returns @c DAE_OK if the document was created successfully, otherwise returns a negative value as defined in daeError.h.
	* @note The @c daeElement passed in as <tt><i>dom</i></tt> should always be a @c domCOLLADA object, the API may enforce this in the future.
	* @deprecated This function will be removed in future versions. Please use createDocument.
	*/
	virtual daeInt insertDocument(daeString name, daeElement* dom, daeDocument** document = NULL) = 0;
	/**
	* Creates a new @c domCOLLADA root element and a new document; returns an error if the document name already exists.
	* @param name Name of the new document, must be a valid URI.
	* @param document Pointer to a @c daeDocument pointer that receives the document created 
	* @return Returns DAE_OK if the document was created successfully, otherwise returns a negative value as defined in daeError.h.
	* @deprecated This function will be removed in future versions. Please use createDocument.
	*/
	virtual daeInt insertDocument(daeString name, daeDocument** document = NULL) = 0;
	/**
	* Creates a new document, defining its root as the <tt><i>dom</i></tt> object; returns an error if the document name already exists.
	* @param name Name of the new document, must be a valid URI.
	* @param dom Existing @c domCOLLADA root element of the document
	* @param document Pointer to a @c daeDocument pointer that receives the document created 
	* @return Returns @c DAE_OK if the document was created successfully, otherwise returns a negative value as defined in daeError.h.
	* @note The @c daeElement passed in as <tt><i>dom</i></tt> should always be a @c domCOLLADA object, the API may enforce this in the future.
	*/
	virtual daeInt createDocument(daeString name, daeElement* dom, daeDocument** document = NULL) = 0;
	/**
	* Creates a new @c domCOLLADA root element and a new document; returns an error if the document name already exists.
	* @param name Name of the new document, must be a valid URI.
	* @param document Pointer to a @c daeDocument pointer that receives the document created 
	* @return Returns DAE_OK if the document was created successfully, otherwise returns a negative value as defined in daeError.h.
	*/
	virtual daeInt createDocument(daeString name, daeDocument** document = NULL) = 0;

	/**
	 * Inserts an already existing document into the database.
	 * @param c The document to insert.
	 * @return Returns DAE_OK if the document was inserted successfully, otherwise returns a negative value as defined in daeError.h.
	 */
	virtual daeInt insertDocument( daeDocument *c ) = 0;

	/**
	* Removes a document from the database.
	* @param document Document to remove from the database
	* @return Returns DAE_OK if the document was successfully removed, otherwise returns a negative value as defined in daeError.h. 
	*/
	virtual daeInt removeDocument(daeDocument* document) = 0;
	/**
	* Gets the number of documents.
	* @return Returns the number of documents.
	*/
	virtual daeUInt getDocumentCount() = 0;
	/**
	* Gets a document based on the document index.
	* @param index Index of the document to get.
	* @return Returns a pointer on the document, or NULL if not found. 
	*/
	virtual daeDocument* getDocument(daeUInt index) = 0;
	/**
	* Gets a document based on the document name.
	* @param name The name of the document as a URI.
	* @return Returns a pointer to the document, or NULL if not found. 
	* @note If the URI contains a fragment, the fragment is stripped off.
	*/
	virtual daeDocument* getDocument(daeString name) = 0;
	/**
	* Gets a document name.
	* @param index Index of the document to get.
	* @return Returns the name of the document at the given index. 
	*/
	virtual daeString getDocumentName(daeUInt index) = 0;
	/**
	* Indicates if a document is loaded or not.
	* @param name Name of the document  as a URI.
	* @return Returns true if the document is loaded, false otherwise.
	* @note If the URI contains a fragment, the fragment is stripped off.
	*/
	virtual daeBool isDocumentLoaded(daeString name) = 0;
	//@}
	
	/** @name Elements */ 
	//@{
	/**
	* Gets the number of types in the database.
	* @return Returns the number of different types of objects inserted in the database.
	*/
	virtual daeUInt getTypeCount() = 0;
	/**
	* Retrieves the name of a type of object inserted in the database.
	* @param index Index of the type; must be between 0 and <tt> daeDatabase::getTypeCount()-1 </tt>
	* @return Returns the name of the type, NULL if the index is invalid.
	*/
	virtual daeString getTypeName(daeUInt index) = 0;
	/**
	* Inserts a @c daeElement into the runtime database.
	* @param document Document in which the @c daeElement lives.
	* @param element @c daeElement to insert in the database
	* @return Returns @c DAE_OK if element successfully inserted, otherwise returns a negative value as defined in daeError.h.
	*/
	virtual daeInt insertElement(daeDocument* document,
	                             daeElement* element) = 0;
	/**
	* Removes a @c daeElement from the runtime database; not implemented in the reference STL implementation.
	* @param document Document in which the @c daeElement lives.
	* @param element Element to remove.
	* @return Returns @c DAE_OK if element successfully removed, otherwise returns a negative value as defined in daeError.h.
	* @note This function is not implemented in the reference STL implementation.
	*/
	virtual daeInt removeElement(daeDocument* document,
	                           daeElement* element) = 0;
	/**
	* Unloads all of the documents of the runtime database.
	* This function frees all the @c dom* objects and integration objects created so far,
	* except any objects on which you still have a smart pointer reference (@c daeSmartRef).
	* @return Returns @c DAE_OK if all documents successfully unloaded, otherwise returns a negative value as defined in daeError.h.
	*/
	virtual daeInt clear() = 0;
	/**
	* Optimizes the database.
	* This function takes time; it is called by the interface at the end of a load operation.
	* Some databases cannot be queried when items are being inserted; for instance, they may
	* need to be sorted. All database search functions call @c validate(); you should not need to 
	* call this function directly.
	*/
	virtual void validate() = 0;
	//@}

	/** @name Queries */
	//@{
	/**
	* Gets the number of daeElement objects that match the search criteria
	* Any combination of search criteria can be NULL, if a criterion is NULL all 
	* the parameters will match for this criterion.
	* Hence @c getElementCount() called without parameters returns the total number of @c daeElement objects in the database.
	* Criteria can not be specified with wildcards, either a criterion is set and it will have
	* to match, or it is not set and all @c daeElements match for this criterion.
	* @param name Name or id of the @c daeElement, for example, "mycube1", can be NULL
	* @param type Type of @c daeElement to find, this can be any COLLADA tag such as <geometry> or <library>, can be NULL
	* @param file Name of the document or file, for example, "myDocument.xml", can be NULL
	* @return Returns the number of elements matching this query.
	*/
	virtual daeUInt getElementCount(daeString name = NULL,
								  daeString type = NULL,
								  daeString file = NULL) = 0;
	/**
	* Returns the @c daeElement which matches the search criteria.
	* Any combination of search criteria can be NULL, if a criterion is NULL all 
	* the parameters will match for this criterion.
	* The function operates on the set of assets that match the <tt><i>name, type</i></tt> and <tt><i>file</i></tt> search criteria, 
	* with the <tt><i>index</i></tt> parameter indicating which asset within the set is returned.
	* Calling @c daeElement(&pElement,index) without search criteria returns the @c daeElement number <tt><i>index</i></tt> in the database without
	* any consideration of name, type or document.
	* Criteria can not be specified with wildcards, either a criterion is set and it will have
	* to match, or it is not set and all @c daeElements match for this criterion.
	* The default database search is roughly in log2(n). Maximum performance is obtained when querying 
	* by type and a name. Any other combination results in a slight overhead, but the overall search time
	* remains around log2(n).
	* @param pElement Pointer of a @c daeElement* which receives the found @c daeElement if the search succeeds
	* @param index Index within the set of @c daeElements that match the search criteria
	* @param name Name or id of the @c daeElement, for example "mycube1", can be NULL
	* @param type Type of the @c daeElement to get, this can be any COLLADA tag such as <geometry> or <library>, can be NULL
	* @param file Name of the document or file, for example, "myDocument.xml", can be NULL
	* @return Returns DAE_OK upon success, returns DAE_ERR_QUERY_NO_MATCH if there is no match, otherwise, returns a negative value as defined in daeError.h.
	*/
	virtual daeInt getElement(daeElement** pElement,
							daeInt index,
							daeString name = NULL,
							daeString type = NULL,
							daeString file = NULL ) = 0;
	/**
	* Returns the @c daeElement which matches the <tt><i>genericQuery</i></tt> parameter; not implemented.
	* @param pElement Element to return.
	* @param genericQuery Generic query
	* @return Returns DAE_OK if it succeeds, returns DAE_ERR_QUERY_NO_MATCH if there is no match, otherwise returns a negative value as defined in daeError.h.
	* @note This function is not implemented.
	*/
	virtual daeInt queryElement(daeElement** pElement, daeString genericQuery) = 0;
	//@}
	
	/** 
	* Sets the top meta object.
	* Called by @c dae::setDatabase() when the database changes. It passes to this function the
	* top meta object, which is the root of a 
    * hierarchy of @c daeMetaElement objects. This top meta object is capable of creating
	* any of the root objects in the DOM tree.
	* @param _topMeta Top meta object to use to create objects to fill the database.
	* @return Returns DAE_OK if successful, otherwise returns a negative value defined in daeError.h.
	*/
	virtual daeInt setMeta(daeMetaElement *_topMeta) = 0;

public: //Depricated methods
	inline daeInt insertCollection(daeString name, daeElement* dom, daeDocument** document = NULL) {
		return insertDocument( name, dom, document );
	}
	inline daeInt insertCollection(daeString name, daeDocument** document = NULL) {
		return insertDocument( name, document );
	}
	inline daeInt createCollection(daeString name, daeElement* dom, daeDocument** document = NULL) {
		return createDocument( name, dom, document );
	}
	inline daeInt createCollection(daeString name, daeDocument** document = NULL) {
		return createDocument( name, document );
	}
	inline daeInt insertCollection( daeDocument *c ) {
		return insertDocument( c );
	}
	inline daeInt removeCollection(daeDocument* document) {
		return removeDocument( document );
	}
	inline daeUInt getCollectionCount() {
		return getDocumentCount();
	}
	inline daeDocument* getCollection(daeUInt index) {
		return getDocument( index );
	}
	inline daeDocument* getCollection(daeString name) {
		return getDocument( name );
	}
	inline daeString getCollectionName(daeUInt index) {
		return getDocumentName( index );
	}
	inline daeBool isCollectionLoaded(daeString name) {
		return isDocumentLoaded( name );
	}

};

#endif //__DAE_DATABASE__

