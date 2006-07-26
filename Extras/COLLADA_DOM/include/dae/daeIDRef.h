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

#ifndef __DAE_IDREF_H__
#define __DAE_IDREF_H__

#include <dae/daeTypes.h>
#include <dae/daeElement.h>

/**
 * The @c daeIDRef is a simple class designed to aid in the parsing and resolution of
 * ID references inside of COLLADA elements.
 * A @c daeIDRef is created for every IDREF data type in the COLLADA schema.
 * It also has the capability to attempt to resolve this reference
 * into a @c daeElement.  If a @c daeIDRef is stored within a @c daeElement it fills
 * in its container field to point to the containing element.
 *
 * The main API is the @c daeIDRef::resolveElement() will use a @c daeIDRefResolver
 * to search for the @c daeElement inside of a @c daeDatabase.
 *
 */
class daeIDRef
{
public:
	/**
	 * An enum describing the status of the ID resolution process.
	 */
	enum ResolveState{
		/** No ID specified */
		id_empty,
		/** ID specified but not resolved */
		id_loaded,
		/** ID resolution pending */
		id_pending,
		/** ID resolved correctly */
		id_success,
		/** Resolution failed because ID was not found */
		id_failed_id_not_found,
		/** Resolution failed because ID was invalid */
		id_failed_invalid_id,
		/** Resoltion failed due to invalid reference */
		id_failed_invalid_reference,
		/** Resolution failed due to an external error */
		id_failed_externalization
	};
	
private:	
	/** Id used to refer to another element */
	daeString id;

	/** Reference to the actual element the ID refers to */
	daeElementRef element;

	/** Element that owns this ID (if any) */
	daeElement* container;
	
	/** Current state of this id's resolution */ 
	ResolveState state;

public:
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

public:
	/**
	 * Simple Constructor
	 */
	daeIDRef();
	/**
	 * Destructor
	 */
	~daeIDRef();

	/**
	 * Constructs an id reference via a string, using @c setID(); loads the status.
	 * @param id ID to construct a reference for, passed to @c setID() automatically.
	 */
	daeIDRef(daeString id);
	
	/**
	 * Constructs a new id reference by copying an existing one. 
	 * @param constructFromIDRef @c daeIDRef to copy into this one.
	 */
	daeIDRef(daeIDRef& constructFromIDRef);

	/**
	 * Copies <tt><i>ID</i></tt> into the  <tt><i>id	</i></tt> data member.
	 * After the call to @c setID(), the <tt><i>state</i></tt> is set to @c id_loaded
	 * @param ID String to use to configure this @c daeIDRef.
	 */
	void setID(daeString ID);

	/**
	 * Gets the ID string
	 * @return Returns the full ID string from <tt><i>id.</i></tt> 
	 */
	daeString getID() const;

	/**
	 * Uses the @c daeIDRefResolver static API to try to resolve this ID
	 * into a @c daeElement reference.
	 * This function can effectively force a load of a file, perform
	 * a database query, et cetera based on the @c daeIDRefResolver plugins
	 * implemented.
	 */
	void resolveElement( daeString typeNameHint = NULL );

	/**
	 * Configures the <tt><i>id</i></tt> string of this @c daeIDRef based on the element set its <tt><i>element</i></tt> data member.
	 * Uses  @c daeElement::getID() to get the element's ID information to configure
	 * the <tt><i>id</i></tt> string.
	 */
	void resolveID();

	/**
	 * Sets the <tt><i>state</i></tt> of this @c daeIDRef to @c id_pending, as it is awaiting a call to
	 * @c resolveElement().
	 */
	void validate();

	/**
	 * Copies <tt><i>from</i></tt> into <tt><i>this.</i></tt> 
	 * The function does a simple copy, and not "base validation".
	 * @param from  @c daeIDRef to copy from.
	 */
	void copyFrom(daeIDRef& from);

	/**
	 * Outputs all components of this @c daeIDRef to stderr.
	 */
	void print();

	/**
	 * Resets this @c daeIDRef; frees all string references
	 * and returns <tt><i>state</i></tt> to @c empty.
	 */
	void reset();

	/**
	 * Initializes the @c daeIDREf, setting <tt><i>id, element,</i></tt>  and <tt><i>container</i></tt> to NULL.
	 */
	void initialize();

	//Backwards Compatibility
	daeIDRef &get( daeUInt idx ) { (void)idx; return *this; }
	size_t getCount() const { return 1; }
	daeIDRef& operator[](size_t index) { (void)index; return *this; }
};

class daeIDRefResolver;
typedef daeTArray<daeIDRefResolver*> daeIDRefResolverPtrArray;

/**
 * The @c daeIDRefResolver class is the plugin point for @c daeIDRef resolution.
 * This class is an abstract base class that defines an interface for
 * resolving @c daeIDRefs.
 * All instances of @c daeIDRefResolvers are tracked centrally.
 * Every @c daeIDRef is passed through this list of @c aeIDRefResolvers for resolution.
 * The list is ordered on a first come, first serve basis, and resolution
 * terminates after any resolver instance is able to resolve the ID.
 */
class daeIDRefResolver
{
public:
	/**
	 * Constructor; base constructor appends @c this to <tt><i>_KnownResolvers</i></tt> list.
	 */
	daeIDRefResolver();

	/**
	 * Destructor
	 */
	virtual ~daeIDRefResolver();
	
protected:
	static daeIDRefResolverPtrArray _KnownResolvers;
	
public:
	/**
	 * Iterates through known resolvers
	 * calling @c resolveElement().
	 * @param id @c daeIDRef to resolve.
	 */
	static void attemptResolveElement(daeIDRef &id, daeString typeNameHint = NULL );

	/**
	 * attemptResolveID iterates through known resolvers
	 * calling resolveID().
	 * @param id @c daeIDRef to resolve.
	 */
	static void	attemptResolveID(daeIDRef &id);

public: // Abstract Interface
	/**
	 * Provides an abstract interface to convert a @c daeIDRef into a @c daeElement.
	 * @param IDRef @c daeIDRef to resolve.
	 * @return Returns true if the @c daeIDRefResolver successfully resolved the IDRef,
	 * returns false otherwise.
	 */
	virtual daeBool resolveElement(daeIDRef& IDRef, daeString typeNameHint = NULL ) = 0;
	/**
	 * Provides an abstract interface to convert a @c daeElement into a @c daeIDRef.
	 * @param IDRef @c daeIDRef to resolve.
	 * @return Returns true if the @c daeIDRefResolver successfully resolved the element
	 * into a @c daeIDRef, returns false otherwise. 
	 */
	virtual daeBool resolveID(daeIDRef& IDRef) = 0;

	/**
	 * Gets the name of this resolver.
	 * @return Returns the string name.
	 */
	virtual daeString getName() = 0;

};

class daeDatabase;

/**
 * The @c daeDefaultIDRefResolver resolves a @c daeIDRef by checking with a database.
 * It is a concrete implementation for @c daeIDRefResolver.
 */
class daeDefaultIDRefResolver : public daeIDRefResolver
{
public:
	/**
	 * Constructor
	 * @param database @c daeDatabase for this implementation.
	 */
	daeDefaultIDRefResolver(daeDatabase* database);

	/**
	 * Destructor
	 */
	~daeDefaultIDRefResolver();

protected:
	daeDatabase* _database;
	
public: // Abstract Interface
	/*
	 * Implements base class abstract routine from @c daeIDRefResolver.
	 */
	virtual daeBool resolveElement(daeIDRef& id, daeString typeNameHint = NULL );
	
	/*
	 * Implements base class abstract routine from @c daeIDRefResolver.
	 */
	virtual daeBool resolveID(daeIDRef& id);
	
	/*
	 * Implements base class abstract routine from @c daeIDRefResolver.
	 */
	virtual daeString getName();
};

#endif //__DAE_IDREF_H__












