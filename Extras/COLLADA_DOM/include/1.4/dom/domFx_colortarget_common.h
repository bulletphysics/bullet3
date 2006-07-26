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
#ifndef __domFx_colortarget_common_h__
#define __domFx_colortarget_common_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>


class domFx_colortarget_common_complexType 
{
protected:  // Attributes
	xsNonNegativeInteger attrIndex;
	domFx_surface_face_enum attrFace;
	xsNonNegativeInteger attrMip;
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
	 * Gets the face attribute.
	 * @return Returns a domFx_surface_face_enum of the face attribute.
	 */
	domFx_surface_face_enum getFace() const { return attrFace; }
	/**
	 * Sets the face attribute.
	 * @param atFace The new value for the face attribute.
	 */
	void setFace( domFx_surface_face_enum atFace ) { attrFace = atFace; }

	/**
	 * Gets the mip attribute.
	 * @return Returns a xsNonNegativeInteger of the mip attribute.
	 */
	xsNonNegativeInteger getMip() const { return attrMip; }
	/**
	 * Sets the mip attribute.
	 * @param atMip The new value for the mip attribute.
	 */
	void setMip( xsNonNegativeInteger atMip ) { attrMip = atMip; }

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
	 * @return Returns a xsNCName of the value.
	 */
	xsNCName getValue() const { return _value; }
	/**
	 * Sets the _value of this element.
	 * @param val The new value for this element.
	 */
	void setValue( xsNCName val ) { *(daeStringRef*)&_value = val; }

protected:
	/**
	 * Constructor
	 */
	domFx_colortarget_common_complexType() : attrIndex(), attrFace(), attrMip(), attrSlice(), _value() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_colortarget_common_complexType() {}
	/**
	 * Copy Constructor
	 */
	domFx_colortarget_common_complexType( const domFx_colortarget_common_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_colortarget_common_complexType &operator=( const domFx_colortarget_common_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domFx_colortarget_common_complexType.
 */
class domFx_colortarget_common : public daeElement, public domFx_colortarget_common_complexType
{

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
	void setIndex( xsNonNegativeInteger atIndex ) { attrIndex = atIndex;
	 _validAttributeArray[0] = true; }

	/**
	 * Gets the face attribute.
	 * @return Returns a domFx_surface_face_enum of the face attribute.
	 */
	domFx_surface_face_enum getFace() const { return attrFace; }
	/**
	 * Sets the face attribute.
	 * @param atFace The new value for the face attribute.
	 */
	void setFace( domFx_surface_face_enum atFace ) { attrFace = atFace;
	 _validAttributeArray[1] = true; }

	/**
	 * Gets the mip attribute.
	 * @return Returns a xsNonNegativeInteger of the mip attribute.
	 */
	xsNonNegativeInteger getMip() const { return attrMip; }
	/**
	 * Sets the mip attribute.
	 * @param atMip The new value for the mip attribute.
	 */
	void setMip( xsNonNegativeInteger atMip ) { attrMip = atMip;
	 _validAttributeArray[2] = true; }

	/**
	 * Gets the slice attribute.
	 * @return Returns a xsNonNegativeInteger of the slice attribute.
	 */
	xsNonNegativeInteger getSlice() const { return attrSlice; }
	/**
	 * Sets the slice attribute.
	 * @param atSlice The new value for the slice attribute.
	 */
	void setSlice( xsNonNegativeInteger atSlice ) { attrSlice = atSlice;
	 _validAttributeArray[3] = true; }

protected:
	/**
	 * Constructor
	 */
	domFx_colortarget_common() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_colortarget_common() {}
	/**
	 * Copy Constructor
	 */
	domFx_colortarget_common( const domFx_colortarget_common &cpy ) : daeElement(), domFx_colortarget_common_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_colortarget_common &operator=( const domFx_colortarget_common &cpy ) { (void)cpy; return *this; }

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
