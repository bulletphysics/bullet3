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
#ifndef __domGles_texenv_command_type_h__
#define __domGles_texenv_command_type_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domGles_texture_constant_type.h>

class domGles_texenv_command_type_complexType 
{
protected:  // Attributes
	domGles_texenv_mode_enums attrOperator;
	xsNCName attrUnit;

protected:  // Element
	domGles_texture_constant_typeRef elemConstant;

public:	//Accessors and Mutators
	/**
	 * Gets the operator attribute.
	 * @return Returns a domGles_texenv_mode_enums of the operator attribute.
	 */
	domGles_texenv_mode_enums getOperator() const { return attrOperator; }
	/**
	 * Sets the operator attribute.
	 * @param atOperator The new value for the operator attribute.
	 */
	void setOperator( domGles_texenv_mode_enums atOperator ) { attrOperator = atOperator; }

	/**
	 * Gets the unit attribute.
	 * @return Returns a xsNCName of the unit attribute.
	 */
	xsNCName getUnit() const { return attrUnit; }
	/**
	 * Sets the unit attribute.
	 * @param atUnit The new value for the unit attribute.
	 */
	void setUnit( xsNCName atUnit ) { *(daeStringRef*)&attrUnit = atUnit; }

	/**
	 * Gets the constant element.
	 * @return a daeSmartRef to the constant element.
	 */
	const domGles_texture_constant_typeRef getConstant() const { return elemConstant; }
protected:
	/**
	 * Constructor
	 */
	domGles_texenv_command_type_complexType() : attrOperator(), attrUnit(), elemConstant() {}
	/**
	 * Destructor
	 */
	virtual ~domGles_texenv_command_type_complexType() {}
	/**
	 * Copy Constructor
	 */
	domGles_texenv_command_type_complexType( const domGles_texenv_command_type_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGles_texenv_command_type_complexType &operator=( const domGles_texenv_command_type_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domGles_texenv_command_type_complexType.
 */
class domGles_texenv_command_type : public daeElement, public domGles_texenv_command_type_complexType
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::GLES_TEXENV_COMMAND_TYPE; }

public:	//Accessors and Mutators
	/**
	 * Gets the operator attribute.
	 * @return Returns a domGles_texenv_mode_enums of the operator attribute.
	 */
	domGles_texenv_mode_enums getOperator() const { return attrOperator; }
	/**
	 * Sets the operator attribute.
	 * @param atOperator The new value for the operator attribute.
	 */
	void setOperator( domGles_texenv_mode_enums atOperator ) { attrOperator = atOperator;
	 _validAttributeArray[0] = true; }

	/**
	 * Gets the unit attribute.
	 * @return Returns a xsNCName of the unit attribute.
	 */
	xsNCName getUnit() const { return attrUnit; }
	/**
	 * Sets the unit attribute.
	 * @param atUnit The new value for the unit attribute.
	 */
	void setUnit( xsNCName atUnit ) { *(daeStringRef*)&attrUnit = atUnit;
	 _validAttributeArray[1] = true; }

protected:
	/**
	 * Constructor
	 */
	domGles_texenv_command_type() {}
	/**
	 * Destructor
	 */
	virtual ~domGles_texenv_command_type() {}
	/**
	 * Copy Constructor
	 */
	domGles_texenv_command_type( const domGles_texenv_command_type &cpy ) : daeElement(), domGles_texenv_command_type_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGles_texenv_command_type &operator=( const domGles_texenv_command_type &cpy ) { (void)cpy; return *this; }

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
