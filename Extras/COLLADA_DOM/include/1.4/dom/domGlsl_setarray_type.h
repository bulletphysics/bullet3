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
#ifndef __domGlsl_setarray_type_h__
#define __domGlsl_setarray_type_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domGlsl_param_type.h>
#include <dom/domGlsl_setarray_type.h>

/**
 * The glsl_newarray_type is used to creates a parameter of a one-dimensional
 * array type.
 */
class domGlsl_setarray_type_complexType 
{
protected:  // Attribute
/**
 *  The length attribute specifies the length of the array. 
 */
	xsPositiveInteger attrLength;

protected:  // Elements
	domGlsl_param_type_Array elemGlsl_param_type_array;
/**
 * You may recursively nest glsl_newarray elements to create multidimensional
 * arrays. @see domArray
 */
	domGlsl_setarray_type_Array elemArray_array;
	/**
	 * Used to preserve order in elements that do not specify strict sequencing of sub-elements.
	 */
	daeElementRefArray _contents;


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
	 * Gets the glsl_param_type element array.
	 * @return Returns a reference to the array of glsl_param_type elements.
	 */
	domGlsl_param_type_Array &getGlsl_param_type_array() { return elemGlsl_param_type_array; }
	/**
	 * Gets the glsl_param_type element array.
	 * @return Returns a constant reference to the array of glsl_param_type elements.
	 */
	const domGlsl_param_type_Array &getGlsl_param_type_array() const { return elemGlsl_param_type_array; }
	/**
	 * Gets the array element array.
	 * @return Returns a reference to the array of array elements.
	 */
	domGlsl_setarray_type_Array &getArray_array() { return elemArray_array; }
	/**
	 * Gets the array element array.
	 * @return Returns a constant reference to the array of array elements.
	 */
	const domGlsl_setarray_type_Array &getArray_array() const { return elemArray_array; }
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
	domGlsl_setarray_type_complexType() : attrLength(), elemGlsl_param_type_array(), elemArray_array() {}
	/**
	 * Destructor
	 */
	virtual ~domGlsl_setarray_type_complexType() {}
	/**
	 * Copy Constructor
	 */
	domGlsl_setarray_type_complexType( const domGlsl_setarray_type_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGlsl_setarray_type_complexType &operator=( const domGlsl_setarray_type_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domGlsl_setarray_type_complexType.
 */
class domGlsl_setarray_type : public daeElement, public domGlsl_setarray_type_complexType
{
protected:
	/**
	 * Constructor
	 */
	domGlsl_setarray_type() {}
	/**
	 * Destructor
	 */
	virtual ~domGlsl_setarray_type() {}
	/**
	 * Copy Constructor
	 */
	domGlsl_setarray_type( const domGlsl_setarray_type &cpy ) : daeElement(), domGlsl_setarray_type_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGlsl_setarray_type &operator=( const domGlsl_setarray_type &cpy ) { (void)cpy; return *this; }

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
