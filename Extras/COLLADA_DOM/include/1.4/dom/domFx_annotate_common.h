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
#ifndef __domFx_annotate_common_h__
#define __domFx_annotate_common_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domFx_annotate_type_common.h>

class domFx_annotate_common_complexType 
{
protected:  // Attribute
	xsNCName attrName;

protected:  // Element
	domFx_annotate_type_commonRef elemFx_annotate_type_common;

public:	//Accessors and Mutators
	/**
	 * Gets the name attribute.
	 * @return Returns a xsNCName of the name attribute.
	 */
	xsNCName getName() const { return attrName; }
	/**
	 * Sets the name attribute.
	 * @param atName The new value for the name attribute.
	 */
	void setName( xsNCName atName ) { *(daeStringRef*)&attrName = atName; }

	/**
	 * Gets the fx_annotate_type_common element.
	 * @return a daeSmartRef to the fx_annotate_type_common element.
	 */
	const domFx_annotate_type_commonRef getFx_annotate_type_common() const { return elemFx_annotate_type_common; }
protected:
	/**
	 * Constructor
	 */
	domFx_annotate_common_complexType() : attrName(), elemFx_annotate_type_common() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_annotate_common_complexType() {}
	/**
	 * Copy Constructor
	 */
	domFx_annotate_common_complexType( const domFx_annotate_common_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_annotate_common_complexType &operator=( const domFx_annotate_common_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domFx_annotate_common_complexType.
 */
class domFx_annotate_common : public daeElement, public domFx_annotate_common_complexType
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FX_ANNOTATE_COMMON; }

public:	//Accessors and Mutators
	/**
	 * Gets the name attribute.
	 * @return Returns a xsNCName of the name attribute.
	 */
	xsNCName getName() const { return attrName; }
	/**
	 * Sets the name attribute.
	 * @param atName The new value for the name attribute.
	 */
	void setName( xsNCName atName ) { *(daeStringRef*)&attrName = atName;
	 _validAttributeArray[0] = true; }

protected:
	/**
	 * Constructor
	 */
	domFx_annotate_common() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_annotate_common() {}
	/**
	 * Copy Constructor
	 */
	domFx_annotate_common( const domFx_annotate_common &cpy ) : daeElement(), domFx_annotate_common_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_annotate_common &operator=( const domFx_annotate_common &cpy ) { (void)cpy; return *this; }

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
