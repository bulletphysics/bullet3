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
#ifndef __domGles_texture_constant_type_h__
#define __domGles_texture_constant_type_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>


class domGles_texture_constant_type_complexType 
{
protected:  // Attributes
	domFloat4 attrValue;
	xsNCName attrParam;


public:	//Accessors and Mutators
	/**
	 * Gets the value array attribute.
	 * @return Returns a domFloat4 reference of the value array attribute.
	 */
	domFloat4 &getValue() { return attrValue; }
	/**
	 * Gets the value array attribute.
	 * @return Returns a constant domFloat4 reference of the value array attribute.
	 */
	const domFloat4 &getValue() const { return attrValue; }
	/**
	 * Sets the value array attribute.
	 * @param atValue The new value for the value array attribute.
	 */
	void setValue( const domFloat4 &atValue ) { attrValue = atValue; }

	/**
	 * Gets the param attribute.
	 * @return Returns a xsNCName of the param attribute.
	 */
	xsNCName getParam() const { return attrParam; }
	/**
	 * Sets the param attribute.
	 * @param atParam The new value for the param attribute.
	 */
	void setParam( xsNCName atParam ) { *(daeStringRef*)&attrParam = atParam; }

protected:
	/**
	 * Constructor
	 */
	domGles_texture_constant_type_complexType() : attrValue(), attrParam() {}
	/**
	 * Destructor
	 */
	virtual ~domGles_texture_constant_type_complexType() {}
	/**
	 * Copy Constructor
	 */
	domGles_texture_constant_type_complexType( const domGles_texture_constant_type_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGles_texture_constant_type_complexType &operator=( const domGles_texture_constant_type_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domGles_texture_constant_type_complexType.
 */
class domGles_texture_constant_type : public daeElement, public domGles_texture_constant_type_complexType
{

public:	//Accessors and Mutators
	/**
	 * Gets the value array attribute.
	 * @return Returns a domFloat4 reference of the value array attribute.
	 */
	domFloat4 &getValue() { return attrValue; }
	/**
	 * Gets the value array attribute.
	 * @return Returns a constant domFloat4 reference of the value array attribute.
	 */
	const domFloat4 &getValue() const { return attrValue; }
	/**
	 * Sets the value array attribute.
	 * @param atValue The new value for the value array attribute.
	 */
	void setValue( const domFloat4 &atValue ) { attrValue = atValue;
	 _validAttributeArray[0] = true; }

	/**
	 * Gets the param attribute.
	 * @return Returns a xsNCName of the param attribute.
	 */
	xsNCName getParam() const { return attrParam; }
	/**
	 * Sets the param attribute.
	 * @param atParam The new value for the param attribute.
	 */
	void setParam( xsNCName atParam ) { *(daeStringRef*)&attrParam = atParam;
	 _validAttributeArray[1] = true; }

protected:
	/**
	 * Constructor
	 */
	domGles_texture_constant_type() {}
	/**
	 * Destructor
	 */
	virtual ~domGles_texture_constant_type() {}
	/**
	 * Copy Constructor
	 */
	domGles_texture_constant_type( const domGles_texture_constant_type &cpy ) : daeElement(), domGles_texture_constant_type_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGles_texture_constant_type &operator=( const domGles_texture_constant_type &cpy ) { (void)cpy; return *this; }

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
