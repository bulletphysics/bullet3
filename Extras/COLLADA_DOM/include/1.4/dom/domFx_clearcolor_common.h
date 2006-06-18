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
#ifndef __domFx_clearcolor_common_h__
#define __domFx_clearcolor_common_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>


class domFx_clearcolor_common_complexType 
{
protected:  // Attribute
	xsNonNegativeInteger attrIndex;

protected:  // Value
	/**
	 * The domFx_color_common value of the text data of this element. 
	 */
	domFx_color_common _value;

public:	//Accessors and Mutators
	/**
	 * Gets the index attribute.
	 * @return Returns a xsNonNegativeInteger of the index attribute.
	 */
	xsNonNegativeInteger getIndex() const { return attrIndex; }
	/**
	 * Sets the index attribute.
	 * @param atIndex The new value for the index attribute.
	 */
	void setIndex( xsNonNegativeInteger atIndex ) { attrIndex = atIndex; }

	/**
	 * Gets the _value array.
	 * @return Returns a domFx_color_common reference of the _value array.
	 */
	domFx_color_common &getValue() { return _value; }
	/**
	 * Gets the _value array.
	 * @return Returns a constant domFx_color_common reference of the _value array.
	 */
	const domFx_color_common &getValue() const { return _value; }
	/**
	 * Sets the _value array.
	 * @param val The new value for the _value array.
	 */
	void setValue( const domFx_color_common &val ) { _value = val; }

protected:
	/**
	 * Constructor
	 */
	domFx_clearcolor_common_complexType() : attrIndex(), _value() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_clearcolor_common_complexType() {}
	/**
	 * Copy Constructor
	 */
	domFx_clearcolor_common_complexType( const domFx_clearcolor_common_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_clearcolor_common_complexType &operator=( const domFx_clearcolor_common_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domFx_clearcolor_common_complexType.
 */
class domFx_clearcolor_common : public daeElement, public domFx_clearcolor_common_complexType
{
protected:
	/**
	 * Constructor
	 */
	domFx_clearcolor_common() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_clearcolor_common() {}
	/**
	 * Copy Constructor
	 */
	domFx_clearcolor_common( const domFx_clearcolor_common &cpy ) : daeElement(), domFx_clearcolor_common_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_clearcolor_common &operator=( const domFx_clearcolor_common &cpy ) { (void)cpy; return *this; }

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
