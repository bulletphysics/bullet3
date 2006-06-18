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
#ifndef __domFx_stenciltarget_common_h__
#define __domFx_stenciltarget_common_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>


class domFx_stenciltarget_common_complexType 
{
protected:  // Attributes
	xsNonNegativeInteger attrIndex;
	xsNonNegativeInteger attrSlice;

protected:  // Value
	/**
	 * The xsNCName value of the text data of this element. 
	 */
	xsNCName _value;

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
	 * Gets the slice attribute.
	 * @return Returns a xsNonNegativeInteger of the slice attribute.
	 */
	xsNonNegativeInteger getSlice() const { return attrSlice; }
	/**
	 * Sets the slice attribute.
	 * @param atSlice The new value for the slice attribute.
	 */
	void setSlice( xsNonNegativeInteger atSlice ) { attrSlice = atSlice; }

	/**
	 * Gets the value of this element.
	 * @return a xsNCName of the value.
	 */
	xsNCName getValue() const { return _value; }
	/**
	 * Sets the _value of this element.
	 * @param val The new value for this element.
	 */
	void setValue( xsNCName val ) { _value = val; }

protected:
	/**
	 * Constructor
	 */
	domFx_stenciltarget_common_complexType() : attrIndex(), attrSlice(), _value() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_stenciltarget_common_complexType() {}
	/**
	 * Copy Constructor
	 */
	domFx_stenciltarget_common_complexType( const domFx_stenciltarget_common_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_stenciltarget_common_complexType &operator=( const domFx_stenciltarget_common_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domFx_stenciltarget_common_complexType.
 */
class domFx_stenciltarget_common : public daeElement, public domFx_stenciltarget_common_complexType
{
protected:
	/**
	 * Constructor
	 */
	domFx_stenciltarget_common() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_stenciltarget_common() {}
	/**
	 * Copy Constructor
	 */
	domFx_stenciltarget_common( const domFx_stenciltarget_common &cpy ) : daeElement(), domFx_stenciltarget_common_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_stenciltarget_common &operator=( const domFx_stenciltarget_common &cpy ) { (void)cpy; return *this; }

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
