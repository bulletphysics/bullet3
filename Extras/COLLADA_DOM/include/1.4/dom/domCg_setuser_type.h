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
#ifndef __domCg_setuser_type_h__
#define __domCg_setuser_type_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domCg_param_type.h>
#include <dom/domCg_setarray_type.h>
#include <dom/domCg_setuser_type.h>
#include <dom/domCg_connect_param.h>

/**
 * Creates an instance of a structured class.
 */
class domCg_setuser_type_complexType 
{
protected:  // Attribute
	domCg_identifier attrName;

protected:  // Elements
	domCg_param_type_Array elemCg_param_type_array;
	domCg_setarray_type_Array elemArray_array;
	domCg_setuser_type_Array elemUsertype_array;
	domCg_connect_param_Array elemConnect_param_array;
	/**
	 * Used to preserve order in elements that do not specify strict sequencing of sub-elements.
	 */
	daeElementRefArray _contents;


public:	//Accessors and Mutators
	/**
	 * Gets the name attribute.
	 * @return Returns a domCg_identifier of the name attribute.
	 */
	domCg_identifier getName() const { return attrName; }
	/**
	 * Sets the name attribute.
	 * @param atName The new value for the name attribute.
	 */
	void setName( domCg_identifier atName ) { attrName = atName; }

	/**
	 * Gets the cg_param_type element array.
	 * @return Returns a reference to the array of cg_param_type elements.
	 */
	domCg_param_type_Array &getCg_param_type_array() { return elemCg_param_type_array; }
	/**
	 * Gets the cg_param_type element array.
	 * @return Returns a constant reference to the array of cg_param_type elements.
	 */
	const domCg_param_type_Array &getCg_param_type_array() const { return elemCg_param_type_array; }
	/**
	 * Gets the array element array.
	 * @return Returns a reference to the array of array elements.
	 */
	domCg_setarray_type_Array &getArray_array() { return elemArray_array; }
	/**
	 * Gets the array element array.
	 * @return Returns a constant reference to the array of array elements.
	 */
	const domCg_setarray_type_Array &getArray_array() const { return elemArray_array; }
	/**
	 * Gets the usertype element array.
	 * @return Returns a reference to the array of usertype elements.
	 */
	domCg_setuser_type_Array &getUsertype_array() { return elemUsertype_array; }
	/**
	 * Gets the usertype element array.
	 * @return Returns a constant reference to the array of usertype elements.
	 */
	const domCg_setuser_type_Array &getUsertype_array() const { return elemUsertype_array; }
	/**
	 * Gets the connect_param element array.
	 * @return Returns a reference to the array of connect_param elements.
	 */
	domCg_connect_param_Array &getConnect_param_array() { return elemConnect_param_array; }
	/**
	 * Gets the connect_param element array.
	 * @return Returns a constant reference to the array of connect_param elements.
	 */
	const domCg_connect_param_Array &getConnect_param_array() const { return elemConnect_param_array; }
	/**
	 * Gets the _contents array.
	 * @return Returns a reference to the _contents element array.
	 */
	daeElementRefArray &getContents() { return _contents; }
	/**
	 * Gets the _contents array.
	 * @return Returns a constant reference to the _contents element array.
	 */
	const daeElementRefArray &getContents() const { return _contents; }

protected:
	/**
	 * Constructor
	 */
	domCg_setuser_type_complexType() : attrName(), elemCg_param_type_array(), elemArray_array(), elemUsertype_array(), elemConnect_param_array() {}
	/**
	 * Destructor
	 */
	virtual ~domCg_setuser_type_complexType() {}
	/**
	 * Copy Constructor
	 */
	domCg_setuser_type_complexType( const domCg_setuser_type_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domCg_setuser_type_complexType &operator=( const domCg_setuser_type_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domCg_setuser_type_complexType.
 */
class domCg_setuser_type : public daeElement, public domCg_setuser_type_complexType
{
protected:
	/**
	 * Constructor
	 */
	domCg_setuser_type() {}
	/**
	 * Destructor
	 */
	virtual ~domCg_setuser_type() {}
	/**
	 * Copy Constructor
	 */
	domCg_setuser_type( const domCg_setuser_type &cpy ) : daeElement(), domCg_setuser_type_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domCg_setuser_type &operator=( const domCg_setuser_type &cpy ) { (void)cpy; return *this; }

public: // STATIC METHODS
	/**
	 * Creates an instance of this class and returns a daeElementRef referencing it.
	 * @param bytes The size allocated for this instance.
	 * @return a daeElementRef referencing an instance of this object.
	 */
	static daeElementRef create(daeInt bytes);
	/**
	 * Creates a daeMetaElement object that describes this element in the meta object reflection framework.
	 * If a daeMetaElement already exists it will return that instead of creating a new one. 
	 * @return A daeMetaElement describing this COLLADA element.
	 */
	static daeMetaElement* registerElement();

public: // STATIC MEMBERS
	/**
	 * The daeMetaElement that describes this element in the meta object reflection framework.
	 */
	static daeMetaElement* _Meta;
};


#endif
