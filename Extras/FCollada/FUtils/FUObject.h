/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/
/*
	The idea is simple: the object registers itself in the constructor, with the container.
	It keeps a pointer to this container, so that the parent can easily access it
	In the destructor, the object unregisters itself with the container.
*/

/**
	@file FUObject.h
	This file contains the FUObject class and the FUObjectContaimer class.
*/

#ifndef _FU_OBJECT_H_
#define _FU_OBJECT_H_

class FUObjectContainer;

/**
	A contained object.
	Each object holds a pointer to the container that contains it.
	This pointer is useful so that the container can be notified if the object
	gets released by someone else.
*/
class FCOLLADA_EXPORT FUObject
{
private:
	FUObjectContainer* container;
#ifdef _DEBUG
	const char* className;
#endif // _DEBUG

protected:
	/** Retrieves the container of the object.
		@return A pointer to the container. */
	inline FUObjectContainer* GetContainer() { return container; }
	inline const FUObjectContainer* GetContainer() const { return container; } /**< See above. */

public:
#ifndef _DEBUG
	/** Constructor.
		Although it is not an abstract class, class is
		not meant to be used directly. This constructor
		informs the container of the object's creation.
		@param container The container that contains this object. */
	FUObject(FUObjectContainer* container);

	/** Overwrites the token name of the object.
		This token name is used for debugging purposes only.
		@param UNUSED The token name for the class of the object. */
	inline void SetClassName(const char* UNUSED(_className)) {};
#else 

	FUObject(FUObjectContainer* container, const char* className);
	inline void SetClassName(const char* _className) {className = _className;}
#endif

	/** Destructor.
		This function informs the container of this object's release. */
	~FUObject();
};

/** A dynamically-sized array of contained objects. */
typedef vector<FUObject*> FUObjectList;

/**
	An object container
	Each container holds a list of contained objects.
	It will release all the objects when it is released and the
	objects inform it of their creation/destruction.
*/
class FCOLLADA_EXPORT FUObjectContainer
{
private:
	friend class FUObject;
	FUObjectList objects;

	void RegisterObject(FUObject* object);
	void UnregisterObject(FUObject* object);

public:
	/** Constructor: empty. */
	FUObjectContainer() {}

	/** Destructor.
		Releases all the objects contained within this container. */
	virtual ~FUObjectContainer();
};

#ifndef _DEBUG
/** Encapsulates the FUObject class constructor.
	This macro is useful to rid ourselves of the 'className' parameter in retail builds.
	@param container The container that contains the object.
	@param className The token name used only for debugging purposes. */
#define FUObject_Construct(container, className) FUObject(container)
#else
#define FUObject_Construct(container, className) FUObject(container, className)
#endif

#endif // _FU_OBJECT_H_
