/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FCDObject.h
	This file contains the FCDObject and the FCDObjectWithId classes.
*/

#ifndef __FCD_OBJECT_H_
#define __FCD_OBJECT_H_

#include "FUtils/FUObject.h"
#include "FCDocument/FCDocument.h"

/**
	A basic COLLADA document object.
	All the objects owned by the COLLADA document derive from this class.
	The FCDocument object is accessible through this interface to all the object which it owns.

	@ingroup FCDocument
*/
class FCOLLADA_EXPORT FCDObject : public FUObject
{
private:
	// Don't use this constructor directly.
	FCDObject(FUObjectContainer* container);

public:
	/** Constructor: sets the COLLADA document object and the informative name.
		The name of the class is used only for debugging purposes and is
		not accessible in release and retail builds. Therefore, it is not meant to replace or implement RTTI.
		@param document The COLLADA document which owns this object.
		@param className A information name to identify the class of the object. */
	FCDObject(FCDocument* document, const char* className);

	/** Destructor. */
	virtual ~FCDObject() {}

	/** Retrieves the COLLADA document which owns this object.
		@return The COLLADA document. */
	inline FCDocument* GetDocument() { return (FCDocument*) GetContainer(); }
	inline FCDocument* GetDocument() const { return (FCDocument*) GetContainer(); } /**< See above. */
};

/**
	A basic COLLADA object which has a unique COLLADA id.
	
	Many COLLADA structures such as entities and sources need a unique COLLADA id.
	The COLLADA document contains a map of all the COLLADA ids known in its scope.
	The interface of the FCDObjectWithId class allows for the retrieval and the modification
	of the unique COLLADA id attached to these objects.

	A unique COLLADA id is built, if none are provided, using the 'baseId' field of the constructor.
	A unique COLLADA id is generated only on demand.

	@ingroup FCDocument
*/
class FCOLLADA_EXPORT FCDObjectWithId : public FCDObject
{
private:
	string daeId;
	bool hasUniqueId;

public:
	/** Constructor: sets the prefix COLLADA id to be used if no COLLADA id is provided.
		@param document The COLLADA document which owns this object.
		@param baseId The prefix COLLADA id to be used if no COLLADA id is provided. */
	FCDObjectWithId(FCDocument* document, const char* baseId = "ObjectWithID");

	/** Destructor. */
	virtual ~FCDObjectWithId();

	/** Retrieves the unique COLLADA id for this object.
		If no unique COLLADA id has been previously generated or provided, this function
		has the side-effect of generating a unique COLLADA id.
		@return The unique COLLADA id. */
	const string& GetDaeId() const;

	/** Sets the COLLADA id for this object.
		There is no guarantee that the given COLLADA id will be used, as it may not be unique.
		You can call the GetDaeId function after this call to retrieve the final, unique COLLADA id.
		@param id The wanted COLLADA id for this object. This COLLADA id does not need to be unique.
			If the COLLADA id is not unique, a new unique COLLADA id will be generated. */
	void SetDaeId(const string& id);

	/** Sets the COLLADA id for this object.
		There is no guarantee that the given COLLADA id will be used, as it may not be unique.
		@param id The wanted COLLADA id for this object. This COLLADA id does not need to be unique.
			If the COLLADA id is not unique, a new unique COLLADA id will be generated and
			this formal variable will be modified to contain the new COLLADA id. */
	void SetDaeId(string& id);

	/** [INTERNAL] Release the unique COLLADA id of an object.
		Use this function wisely, as it leaves the object id-less and without a way to automatically
		generate a COLLADA id. */
	void RemoveDaeId();

	/** [INTERNAL] Clones the object. The unique COLLADA id will be copied over to the clone object.
		Use carefully: when a cloned object with an id is released, it
		does remove the unique COLLADA id from the unique name map.
		@param clone The object clone. */
	void Clone(FCDObjectWithId* clone) const;
};

#endif // __FCD_OBJECT_H_
