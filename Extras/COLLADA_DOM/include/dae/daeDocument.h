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

#ifndef __DAE_DOCUMENT__
#define __DAE_DOCUMENT__

#include <dae/daeTypes.h>
#include <dae/daeElement.h>
#include <dae/daeURI.h>
#include <dae/daeStringRef.h>

/**
 * The @c daeDocument class implements a COLLADA runtime database entry.
 */
class daeDocument
{
public:
    // sthomas
    /**
    * Destructor
    */
    DLLSPEC ~daeDocument();

	/**
	* Accessor to get the @c domCollada associated with this document.
	* @return A @c daeElementRef for the @c domCollada that is the root of this document.
	* @note This function should really return a domColladaRef,
	* but we're trying to avoid having @c dae classes depend on generated dom classes.
	*/
	daeElement* getDomRoot() const {return(dom);}
	/**
	* Accessor to set the domCollada associated with this document
	* @param domRoot the domCollada that is the root of this document
	* @remarks Should really require a domColladaRef but we're trying to avoid having dae classes depend on generated dom classes.
	*/
	void setDomRoot(daeElement* domRoot) {dom = domRoot; domRoot->setDocument(this); }
	/**
	* Accessor to get the URI associated with the document in this document; 
	* this is currently set to the URI from which the document was loaded, but
	* is blank if the document was created with @c insertDocument().
	* @return Returns a pointer to the URI for this document.
	* @note This is the full URI of the document and not the document base URI.
	*/
	daeURI* getDocumentURI() {return (&uri);}

	/**
	* Const accessor to get the URI associated with the document in this collection; 
	* this is currently set to the URI from which the collection was loaded, but
	* is blank if the collection was created with @c insertCollection().
	* @return Returns a pointer to the URI for this collection.
	* @note This is the full URI of the document and not the document base URI.
	*/
	const daeURI* getDocumentURI() const {return (&uri);}

	/**
	 * Accessor to get if this document has been modified since the last time the database was validated.
	 * @return Returns true if the document was modified, false otherwise.
	 */
	daeBool getModified() const {return modified;}
	/**
	 * Sets if this document has been modified since the last time the database was validated.
	 * @param A boolean value specifying if the document was modified.
	 */
	void setModified( daeBool mod ) { if (!mod) { insertedElements.clear(); removedElements.clear(); } modified = mod;}

	/**
	 * This function is used to track how a document gets modified. It gets called internally.
	 * @param element The element that was added to this document.
	 * @note This function is called internally and not meant to be called by the client application.
	 * Calling this function from the client application may result in unexpected behavior.
	 */
	DLLSPEC void insertElement( daeElementRef element );
	/**
	 * This function is used to track how a document gets modified. It gets called internally.
	 * @param element The element that was removed from this document.
	 * @note This function is called internally and not meant to be called by the client application.
	 * Calling this function from the client application may result in unexpected behavior.
	 */
	DLLSPEC void removeElement( daeElementRef element );

	/**
	 * This function is used to track how a document gets modified. It gets called internally.
	 * @return Returns an array of elements that have been added since the last database update.
	 */
	const daeElementRefArray &getInsertedArray() const { return insertedElements; }
	/**
	 * This function is used to track how a document gets modified. It gets called internally.
	 * @return Returns an array of elements that have been removed since the last database update.
	 */
	const daeElementRefArray &getRemovedArray() const { return removedElements; }
	/**
	 * Adds a URI to the list of external references in this document.
	 * @param uri The URI that is the external reference.
	 * @note This function gets called internally from daeURI upon trying to resolve an element.
	 * Calling this function in your client code my result in unexpected behavior.
	 */
	DLLSPEC void addExternalReference( daeURI &uri );
	/**
	 * Removes a URI to the list of external references in this document.
	 * @param uri The URI that was the external reference.
	 * @note This function gets called internally from daeURI upon trying to resolve an element.
	 * Calling this function in your client code my result in unexpected behavior.
	 */
	DLLSPEC void removeExternalReference( daeURI &uri );
	/**
	 * Gets a list of all the documents that are referenced from URI contained within this document.
	 * @return Returns a list of URI strings, each being a URI which is referenced from within this document.
	 */
	const daeStringRefArray &getReferencedDocuments() const { return referencedDocuments; }
	/**
	 * Resolves the URIs that reference the document specified by docURI.
	 * @param docURI The URI string of the document that you want to resolve against.
	 * @note This function is called internally whenever a new document is loaded.
	 */
	DLLSPEC void resolveExternals( daeString docURI);

	DLLSPEC const daeTArray<daeURI*> *getExternalURIs(daeStringRef docURI) const;

private:
	/**
	* Top Level element for of the document, always a domCollada
	* @remarks This member will eventually be taken private, use getDomRoot() to access it.
	*/
	daeElementRef dom;
	
	/** 
	* The URI of the document, may be blank if the document wasn't loaded from a URI
	* @remarks This member will eventually be taken private, use getDocumentURI() to access it.
	*/
	daeURI uri;

	/**
	 * A flag that indicates if this document has been modified.
	 */
	daeBool modified;

	daeElementRefArray insertedElements;
	daeElementRefArray removedElements;

	daeStringRefArray referencedDocuments;
	daeTArray< daeTArray<daeURI*>* > externalURIs;
};

typedef daeDocument daeCollection;

#endif

