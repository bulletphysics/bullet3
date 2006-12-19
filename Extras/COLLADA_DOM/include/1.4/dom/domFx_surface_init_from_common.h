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
#ifndef __domFx_surface_init_from_common_h__
#define __domFx_surface_init_from_common_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>


/**
 * This element is an IDREF which specifies the image to use to initialize
 * a specific mip of a 1D or 2D surface, 3D slice, or Cube face.
 */
class domFx_surface_init_from_common_complexType 
{
protected:  // Attributes
	xsUnsignedInt attrMip;
	xsUnsignedInt attrSlice;
	domFx_surface_face_enum attrFace;

protected:  // Value
	/**
	 * The xsIDREF value of the text data of this element. 
	 */
	xsIDREF _value;

public:	//Accessors and Mutators
	/**
	 * Gets the mip attribute.
	 * @return Returns a xsUnsignedInt of the mip attribute.
	 */
	xsUnsignedInt getMip() const { return attrMip; }
	/**
	 * Sets the mip attribute.
	 * @param atMip The new value for the mip attribute.
	 */
	void setMip( xsUnsignedInt atMip ) { attrMip = atMip; }

	/**
	 * Gets the slice attribute.
	 * @return Returns a xsUnsignedInt of the slice attribute.
	 */
	xsUnsignedInt getSlice() const { return attrSlice; }
	/**
	 * Sets the slice attribute.
	 * @param atSlice The new value for the slice attribute.
	 */
	void setSlice( xsUnsignedInt atSlice ) { attrSlice = atSlice; }

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
	 * Gets the value of this element.
	 * @return Returns a xsIDREF of the value.
	 */
	xsIDREF &getValue() { return _value; }
	/**
	 * Gets the value of this element.
	 * @return Returns a constant xsIDREF of the value.
	 */
	const xsIDREF &getValue() const { return _value; }
	/**
	 * Sets the _value of this element.
	 * @param val The new value for this element.
	 */
	void setValue( const xsIDREF &val ) { _value = val; }

protected:
	/**
	 * Constructor
	 */
	domFx_surface_init_from_common_complexType() : attrMip(), attrSlice(), attrFace(), _value() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_surface_init_from_common_complexType() {}
	/**
	 * Copy Constructor
	 */
	domFx_surface_init_from_common_complexType( const domFx_surface_init_from_common_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_surface_init_from_common_complexType &operator=( const domFx_surface_init_from_common_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domFx_surface_init_from_common_complexType.
 */
class domFx_surface_init_from_common : public daeElement, public domFx_surface_init_from_common_complexType
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FX_SURFACE_INIT_FROM_COMMON; }

public:	//Accessors and Mutators
	/**
	 * Gets the mip attribute.
	 * @return Returns a xsUnsignedInt of the mip attribute.
	 */
	xsUnsignedInt getMip() const { return attrMip; }
	/**
	 * Sets the mip attribute.
	 * @param atMip The new value for the mip attribute.
	 */
	void setMip( xsUnsignedInt atMip ) { attrMip = atMip;
	 _validAttributeArray[0] = true; }

	/**
	 * Gets the slice attribute.
	 * @return Returns a xsUnsignedInt of the slice attribute.
	 */
	xsUnsignedInt getSlice() const { return attrSlice; }
	/**
	 * Sets the slice attribute.
	 * @param atSlice The new value for the slice attribute.
	 */
	void setSlice( xsUnsignedInt atSlice ) { attrSlice = atSlice;
	 _validAttributeArray[1] = true; }

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
	 _validAttributeArray[2] = true; }

protected:
	/**
	 * Constructor
	 */
	domFx_surface_init_from_common() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_surface_init_from_common() {}
	/**
	 * Copy Constructor
	 */
	domFx_surface_init_from_common( const domFx_surface_init_from_common &cpy ) : daeElement(), domFx_surface_init_from_common_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_surface_init_from_common &operator=( const domFx_surface_init_from_common &cpy ) { (void)cpy; return *this; }

public: // STATIC METHODS
	/**
	 * Creates an instance of this class and returns a daeElementRef referencing it.
	 * @param bytes The size allocated for this instance.
	 * @return a daeElementRef referencing an instance of this object.
	 */
	static DLLSPEC daeElementRef create(daeInt bytes);
	/**
	 * Creates a daeMetaElement object that describes this element in the meta object reflection framework.
	 * If a daeMetaElement already exists it will return that instead of creating a new one. 
	 * @return A daeMetaElement describing this COLLADA element.
	 */
	static DLLSPEC daeMetaElement* registerElement();

public: // STATIC MEMBERS
	/**
	 * The daeMetaElement that describes this element in the meta object reflection framework.
	 */
	static DLLSPEC daeMetaElement* _Meta;
};


#endif
