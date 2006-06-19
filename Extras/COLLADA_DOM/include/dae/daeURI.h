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

#ifndef __DAE_URI_H__
#define __DAE_URI_H__

#include <dae/daeTypes.h>
#include <dae/daeElement.h>

/**
 * The @c daeURI is a simple class designed to aid in the parsing and resolution
 * of URI references inside COLLADA elements.
 * A @c daeURI is created for every @c anyURL and @c IDREF in the COLLADA schema.
 * For example, the <instance> element has the url= attribute of type @c anyURL, and the 
 * <controller> element has the target= attribute of type @c IDREF.
 * The @c daeURI class contains a URI string; the @c setURI() method breaks the string into
 * its components including protocol, authority, path (directory), and ID.
 * It also has the capability to attempt to resolve this reference
 * into a @c daeElement, through the method @c resolveElement().  
 * If a @c daeURI is stored within a @c daeElement, it fills
 * its container field to point to the containing element.
 *
 * The main API on the @c daeURI, @c resolveElement(), uses a @c daeURIResolver
 * to search for the @c daeElement inside a @c daeDatabase.
 *
 * URIs are resolved hierarchically, where each URI is resolved based on
 * the following criteria via itself and its element's base URI, which represents the
 * URI of the document that contains the element, retrieved by 
 * <tt>daeElement::getBaseURI().</tt>
 * If no base URI is provided, then the application URI
 * is used as a base.
 *
 * The URI resolution order for the COLLADA DOM is as follows:
 * - Absolute URI is specified (see definition below):
 *   The URI ignores its parent/base URI when validating. 
 * - Relative URI is specified:
 *   The URI uses the base URI to provide the protocol, authority, and base path.
 *    This URI's path is appended to the path given the the base URI.
 *    This URI's file and ID are used.
 * - Each level of URI is resolved in this way against the base URI of the
 *    containing file until the top level is reached.  Then the application URI
 *    is used as the default.
 *
 * <b>Definition of Absolute URI:</b>  
 * For the purposes of the COLLADA DOM, a URI is considered absolute
 * if it starts by specifying a protocol.
 * For example, 
 * - file://c:/data/foo.dae#myScene is an absolute URI.
 * - foo.dae#myScene is relative.
 * - foo.dae is a top-level file reference and is relative.
 * If the URI does not include a pound sign (#), the <tt><i>id</i></tt> is empty.
 */
class daeURI
{
private:
	void internalSetURI(daeString uri);
	
public:
	/**
	 * An enum describing the status of the URI resolution process.
	 */
	enum ResolveState{
		/** No URI specified */
		uri_empty,
		/** URI specified but unresolved */
		uri_loaded,
		/** Resolution pending */
		uri_pending,
		/** Resolution successful */
		uri_success,
		/** Failure due to unsupported URI scheme */
		uri_failed_unsupported_protocol,
		/** Failure because the file was not found */
		uri_failed_file_not_found,
		/** Failure because the ID was not found */
		uri_failed_id_not_found,
		/** Failure due to an invalid ID */
		uri_failed_invalid_id,
		/** A flag specifying that the URI should be resolved locally to its own document */
		uri_resolve_local,
		/** A flag specifying that the URI should be resolved using this relative URI */
		uri_resolve_relative,
		/** A flag specifying that the URI should be resolved using this absolute URI */
		uri_resolve_absolute,
		/** Failure due to an invalid reference */
		uri_failed_invalid_reference,
		/** Failure due to an external error */
		uri_failed_externalization,
		/** Failure due to missing document */
		uri_failed_missing_container,
		/** Failure because autmoatic loading of a document is turned off */
		uri_failed_external_document
	};
	
private:	
	/** Resolved version of the URI */
	daeString uriString;

	/** Original URI before resolution */
	daeString originalURIString;
	
	// Parceled out of storage as const char*'s
	/** Protocol substring parsed from the URI */
	daeString protocol;
	/** authority substring parsed from the URI */
	daeString authority;
	/** Path substring parsed from the URI */
	daeString filepath;
	/** File name substring parsed from the URI */
	daeString file;
	/** Id substring parsed from the URI */
	daeString id;
	/** Extension parsed from the filename in the URI */
	daeString extension;
	/** Reference to the element that the URI resolves to in memory */
	daeElementRef element;
	/** Pointer to the element that owns this URI */
	daeElement* container;
	/** Current resolver state of the URI */
	ResolveState state;
	/** Flag for if this URI references an external element. */
	daeBool external;
	
public:
	/**
	 * Constructs a daeURI object that contains no URI reference.
	 */
	daeURI();
	/**
	 * Destructor
	 */
	~daeURI();

	/**
	 * Constructs a daeURI object that points to the application's current working
	 * directory.
	 * @param dummy An integer value that has no meaning.
	 * @note This is used only to initialize the Application URI.  It's a simple
	 * workaround to insure that the ApplicationURI is initialized only once and before the user can call
	 * daeURI::setBaseURI() (so when we initialize ApplicationURI there is no chance of wiping out a user value).
	 */
	daeURI(int dummy);

	/**
	 * Constructs a daeURI object from a URI passed in as a string.
	 * @param URIString Passed to setURI() automatically.
	 * @param nofrag If true, the fragment part of the URI is stripped off before construction.
	 */
	daeURI(daeString URIString, daeBool nofrag = false);
	
	/**
	 * Constructs a daeURI object using a <tt><i>baseURI</i></tt> and a <tt><i>uriString.</i></tt> 
	 * Calls setURI(URIString), and @c validate(baseURI).
	 * @param baseURI Base URI to resolve against.
	 * @param URIString String designating this URI.
	 */
	daeURI(daeURI& baseURI, daeString URIString);

	/**
	 * Constructs a daeURI object based on a simple copy from an existing @c daeURI. 
	 * @param constructFromURI  URI to copy into this one.
	 */
	daeURI(daeURI& constructFromURI);

	/**
	 * Gets the ID string parsed from the URI.
	 * @return Returns a pointer to the string.
	 */
	inline daeString getID(){return(id);};

	/**
	 * Gets the file string parsed from the URI.
	 * @return Returns a pointer to the string.
	 */
	inline daeString getFile(){return(file);};

	/**
	 * Gets the path string to the file, without the path name, parsed from the URI.
	 * @return Returns a pointer to the string.
	 */
	inline daeString getFilepath(){return(filepath);};

	/**
	 * Gets the protocol string parsed from the URI.
	 * @return Returns a pointer to the string.
	 */
	inline daeString getProtocol(){return(protocol);};

	/**
	 * Gets the authority string parsed from the URI.
	 * @return Returns a pointer to the string.
	 */
	inline daeString getAuthority(){return(authority);};

	/**
	 * Gets the extension string parsed from the URI.
	 * @return Returns a pointer to the string.
	 */
	inline daeString getExtension(){return(extension);};

	/** 
	 * Gets the element that this URI resolves to in memory.
	 * @return Returns a ref to the element.
	 */
	inline daeElementRef getElement(){return(element);};
	
	/** 
	 * Gets the element that this URI resolves to in memory.
	 * @return Returns a ref to the element.
	 */
	inline daeElementConstRef getElement() const {return(element);};
	
	/** 
	 * Sets the element that this URI resolves to in memory.
	 * @param newref A ref to the element.
	 */
	inline void setElement(daeElementRef newref){element=newref;};

	/**
	 * Gets the resolve state of the URI.
	 * @return Returns the current state.
	 * @note This will be removed when daeURI starts managing its state internally.
	 */
	inline ResolveState getState() const {return(state);};

	/** 
	 * Sets the resolve state of the URI.
	 * @param newState The new state.
	 * @note This will be removed when daeURI starts managing its state internally.
	 */
	inline void setState(ResolveState newState){state=newState;};

	/**
	 * Gets a pointer to the @c daeElement that contains this URI.
	 * @return Returns the pointer to the containing daeElmement.
	 */
	inline daeElement* getContainer() const {return(container);};

	/**
	 * Sets the pointer to the @c daeElement that contains this URI.
	 * @param element Pointer to the containing @c daeElmement.
	 */
	inline void setContainer(daeElement* element){container=element;};

	/**
	 * Copies parameter <tt><i>uri</i></tt> into data member <tt><i>uriString,</i></tt> and then decomposes each of
	 * <tt><i>protocol, authority, filepath, file,</i></tt> and <tt><i>id.</i></tt>
	 * After @c setURI(), the <tt><i>state</i></tt> is set to @c uri_loaded.
	 * @param uri String to use to configure this URI.
	 */
	void setURI(daeString uri);

	/**
	 * Gets the URI stored in the daeURI.
	 * @return Returns the full URI String, from <tt><i>uriString.</i></tt> 
	 */
	daeString getURI() const;

	/**
	 * Gets the original URI String as originally set, not flattened against the base URI.
	 * @return Returns the original URI String as originally set, not flattened against the base URI.
	 */
	daeString getOriginalURI() const;

	/**
	 * Gets if this URI resolves to an element that is not contained in the same document as the URI.
	 * @return Returns true if the URI references an external element. False otherwise.
	 */
	daeBool isExternalReference() const { return external; }
	 
	/**
	 * Uses the @c daeURIResolver static API to try to resolve this URI
	 * into a @c daeElement reference, placing the resolved element into <tt><i>element.</i></tt> 
	 * This function can effectively force a load of a file, perform
	 * a database query, and so on, based on the @c daeURIResolver plugins implemented.
	 */
	void resolveElement(daeString typeNameHint = NULL);

	/**
	 * Configures the <tt><i>uriString</i></tt> for this @c daeURI based on the element set in <tt><i>element.</i></tt> 
	 * Uses the element's base URI and ID information to configure
	 * the URI string.
	 */
	void resolveURI();

	/**
	 * Flattens this URI with base URI to obtain a useable
	 * complete URI for resolution.
	 * @param baseURI Base URI to flatten against if this URI is
	 * relative.
	 * @note After @c validate(), state is @c uri_pending as it is awaiting a call to
	 * @c resolveElement().
	 */
	void validate(daeURI* baseURI = NULL);

	/**
	 * Copies the URI specified in <tt><i>from</i></tt> into @c this.
	 * Performs a simple copy without validating the URI.
	 * @param from URI to copy from.
	 */
	void copyFrom(daeURI& from);

	/**
	 * Outputs all components of this URI to stderr.
	 * Useful for debugging URIs, this outputs each part of the URI separately.
	 */
	void print();
	
	/**
	 * Makes the "originalURI" in this URI relative to some other uri
	 * @param uri the URI to make "this" relative to.
	 * @note this is experimental and not fully tested, please don't use in critical code yet.
	 */
	int makeRelativeTo(daeURI* uri);

	/**
	 * Comparison operator.
	 * @return Returns true if URI's are equal.
	 */
	inline bool operator==(const daeURI& other) const{
		return (!strcmp(other.getURI(), getURI())); }

private:
	/**
	 * Resets this URI; frees all string references
	 * and returns <tt><i>state</i></tt> to @c empty.
	 */
	void reset();

	/**
	 * Provides a shared initialization for all constructors
	 */
	void initialize();
public:
	/**
	* Gets the path part of the URI, including everything from immediately after the authority up to and
	* including the file name, but not the query or fragment.
	* @param dest The user allocated buffer that will receive the path.
	* @param size The size of the buffer.
	* @return Returns true for success, false if the path exceeded the size of the user provided buffer.
	*/
	daeBool getPath(daeChar *dest, daeInt size);

public:
	/**
	 * Sets the application's default base URI.  This is effectively the default protocol,
	 * authority, and path in the case of top-level relative URIs.
	 * @param uri Base URI to use as the default application URI.
	 */
	static void setBaseURI(daeURI& uri);

	/**
	 * Gets the application's default base URI.
	 * @return Returns the base URI used in the case of top-level relative URIs.
	 */
	static daeURI* getBaseURI();

	/**
	 * Performs RFC2396 path normalization.
	 * @param path Path to be normalized.
	 */
	static void normalizeURIPath(char *path); 

};

class daeURIResolver;
typedef daeTArray<daeURIResolver*> daeURIResolverPtrArray;

/**
 * The @c daeURIResolver class is the plugin point for URI resolution.
 * This class is an abstract base class that defines an interface for
 * resolving URIs.
 * All instances of @c daeURIResolvers are tracked centrally.
 * Every URI is passed through this list of @c daeURIResolvers for resolution.
 * Before a @c daeURIResolver receives a URI, the API checks whether it supports
 * the protocol.
 * The list is ordered on a first come, first serve basis, and resolution
 * terminates after any resolver instance resolves the URI.
 */
class daeURIResolver
{
public:
	/**
	 * This base constructor appends @c this to KnownResolvers list.
	 */
	daeURIResolver();

	/**
	 * Destructor
	 */
	virtual ~daeURIResolver();
	
protected:
	static daeURIResolverPtrArray _KnownResolvers;

	static daeBool _loadExternalDocuments;
	
public:
	/**
	 * Iterates through known resolvers
	 * calling @c isProtocolSupported() and, if it is supported, calling
	 * @c resolveElement().
	 * @param uri @c daeURI to resolve.
	 */
	static void attemptResolveElement(daeURI &uri, daeString typeNameHint = NULL);

	/**
	 * Iterates through known resolvers
	 * calling @c isProtocolSupported() and, if it is supported, calling
	 * @c resolveURI().
	 * @param uri @c daeURI to resolve.
	 */
	static void	attemptResolveURI(daeURI &uri);

	/**
	 * Sets a flag that tells the URI resolver whether or not to load a separate document if a URI
	 * being resolved points to one.
	 * @param load Set to true if you want the URI Resolver to automatically load other documents to
	 * resolve URIs.
	 */
	static void setAutoLoadExternalDocuments( daeBool load ) { _loadExternalDocuments = load; }

	/**
	 * Gets a flag that tells if the URI resolver is set to load an external document if a URI
	 * being resolved points to one.
	 * @return Returns true if the resolver will automatically load documents to resolve a URI. 
	 * False otherwise.
	 */
	static daeBool getAutoLoadExternalDocuments() { return _loadExternalDocuments; }

public: // Abstract Interface
	/**
	 * Provides an abstract interface for converting a @c daeURI into a @c daeElement
	 * @param uri @c daeURI to resolve.
	 * @return Returns true if the @c daeURIResolver successfully resolved the URI,
	 * returns false otherwise.
	 */
	virtual daeBool resolveElement(daeURI& uri, daeString typeNameHint = NULL) = 0;
	/**
	 * Provides an abstract interface for converting a @c daeElement into a @c daeURI
	 * @param uri @c daeURI to resolve.
	 * @return Returns true if the @c daeURIResolver successfully resolved the element
	 * into a URI, returns  false otherwise.
	 */
	virtual daeBool resolveURI(daeURI& uri) = 0;

	/**
	 * Gets the name of this resolver.
	 * @return Returns the resolver name as a string.
	 */
	virtual daeString getName() = 0;

	/**
	 * Determines whether this resolver supports a particular protocol
	 * for resolution.
	 * @param protocol Determine whether the resolver supports this protocol.
	 * @return Returns true if this @c daeURIResolver understands how to resolve using this protocol, returns
	 * false otherwise
	 */
	virtual daeBool isProtocolSupported(daeString protocol) = 0;

	/**
	 * Determines whether this resolver supports the given extension. 
	 * This keeps parsers from trying to process incompatible
	 * file formats.
	 * @param extension Extension string found after the '.' in the file name.
	 * @return Returns true if the given extension is supported, returns false otherwise.
	 */
	virtual daeBool isExtensionSupported(daeString extension) = 0;
	
};


#endif //__DAE_URI_H__



