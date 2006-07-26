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
#ifndef __domCg_newarray_type_h__
#define __domCg_newarray_type_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domCg_param_type.h>
#include <dom/domCg_newarray_type.h>
#include <dom/domCg_setuser_type.h>
#include <dom/domCg_connect_param.h>

/**
 * Creates a parameter of a one-dimensional array type.
 */
class domCg_newarray_type_complexType 
{
protected:  // Attribute
/**
 *  The length attribute specifies the length of the array. 
 */
	xsPositiveInteger attrLength;

protected:  // Elements
	domCg_param_type_Array elemCg_param_type_array;
/**
 * Nested array elements allow you to create multidemensional arrays. @see
 * domArray
 */
	domCg_newarray_type_Array elemArray_array;
/**
 * The usertype element allows you to create arrays of usertypes. @see domUsertype
 */
	domCg_setuser_type_Array elemUsertype_array;
	domCg_connect_param_Array elemConnect_param_array;
	/**
	 * Used to preserve order in elements that do not specify strict sequencing of sub-elements.
	 */
	daeElementRefArray _contents;
	/**
	 * Used to preserve order in elements that have a complex content model.
	 */
	daeUIntArray       _contentsOrder;


public:	//Accessors and Mutators
	/**
	 * Gets the length attribute.
	 * @return Returns a xsPositiveInteger of the length attribute.
	 */
	xsPositiveInteger getLength() const { return attrLength; }
	/**
	 * Sets the length attribute.
	 * @param atLength The new value for the length attribute.
	 */
	void setLength( xsPositiveInteger atLength ) { attrLength = atLength; }

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
	domCg_newarray_type_Array &getArray_array() { return elemArray_array; }
	/**
	 * Gets the array element array.
	 * @return Returns a constant reference to the array of array elements.
	 */
	const domCg_newarray_type_Array &getArray_array() const { return elemArray_array; }
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
	domCg_newarray_type_complexType() : attrLength(), elemCg_param_type_array(), elemArray_array(), elemUsertype_array(), elemConnect_param_array() {}
	/**
	 * Destructor
	 */
	virtual ~domCg_newarray_type_complexType() {}
	/**
	 * Copy Constructor
	 */
	domCg_newarray_type_complexType( const domCg_newarray_type_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domCg_newarray_type_complexType &operator=( const domCg_newarray_type_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domCg_newarray_type_complexType.
 */
class domCg_newarray_type : public daeElement, public domCg_newarray_type_complexType
{

public:	//Accessors and Mutators
	/**
	 * Gets the length attribute.
	 * @return Returns a xsPositiveInteger of the length attribute.
	 */
	xsPositiveInteger getLength() const { return attrLength; }
	/**
	 * Sets the length attribute.
	 * @param atLength The new value for the length attribute.
	 */
	void setLength( xsPositiveInteger atLength ) { attrLength = atLength;
	 _validAttributeArray[0] = true; }

protected:
	/**
	 * Constructor
	 */
	domCg_newarray_type() {}
	/**
	 * Destructor
	 */
	virtual ~domCg_newarray_type() {}
	/**
	 * Copy Constructor
	 */
	domCg_newarray_type( const domCg_newarray_type &cpy ) : daeElement(), domCg_newarray_type_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domCg_newarray_type &operator=( const domCg_newarray_type &cpy ) { (void)cpy; return *this; }

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
