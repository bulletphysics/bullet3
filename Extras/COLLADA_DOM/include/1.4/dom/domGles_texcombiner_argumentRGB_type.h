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
#ifndef __domGles_texcombiner_argumentRGB_type_h__
#define __domGles_texcombiner_argumentRGB_type_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>


class domGles_texcombiner_argumentRGB_type_complexType 
{
protected:  // Attributes
	domGles_texcombiner_source_enums attrSource;
	domGles_texcombiner_operandRGB_enums attrOperand;
	xsNCName attrUnit;


public:	//Accessors and Mutators
	/**
	 * Gets the source attribute.
	 * @return Returns a domGles_texcombiner_source_enums of the source attribute.
	 */
	domGles_texcombiner_source_enums getSource() const { return attrSource; }
	/**
	 * Sets the source attribute.
	 * @param atSource The new value for the source attribute.
	 */
	void setSource( domGles_texcombiner_source_enums atSource ) { attrSource = atSource; }

	/**
	 * Gets the operand attribute.
	 * @return Returns a domGles_texcombiner_operandRGB_enums of the operand attribute.
	 */
	domGles_texcombiner_operandRGB_enums getOperand() const { return attrOperand; }
	/**
	 * Sets the operand attribute.
	 * @param atOperand The new value for the operand attribute.
	 */
	void setOperand( domGles_texcombiner_operandRGB_enums atOperand ) { attrOperand = atOperand; }

	/**
	 * Gets the unit attribute.
	 * @return Returns a xsNCName of the unit attribute.
	 */
	xsNCName getUnit() const { return attrUnit; }
	/**
	 * Sets the unit attribute.
	 * @param atUnit The new value for the unit attribute.
	 */
	void setUnit( xsNCName atUnit ) { attrUnit = atUnit; }

protected:
	/**
	 * Constructor
	 */
	domGles_texcombiner_argumentRGB_type_complexType() : attrSource(), attrOperand(), attrUnit() {}
	/**
	 * Destructor
	 */
	virtual ~domGles_texcombiner_argumentRGB_type_complexType() {}
	/**
	 * Copy Constructor
	 */
	domGles_texcombiner_argumentRGB_type_complexType( const domGles_texcombiner_argumentRGB_type_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGles_texcombiner_argumentRGB_type_complexType &operator=( const domGles_texcombiner_argumentRGB_type_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domGles_texcombiner_argumentRGB_type_complexType.
 */
class domGles_texcombiner_argumentRGB_type : public daeElement, public domGles_texcombiner_argumentRGB_type_complexType
{
protected:
	/**
	 * Constructor
	 */
	domGles_texcombiner_argumentRGB_type() {}
	/**
	 * Destructor
	 */
	virtual ~domGles_texcombiner_argumentRGB_type() {}
	/**
	 * Copy Constructor
	 */
	domGles_texcombiner_argumentRGB_type( const domGles_texcombiner_argumentRGB_type &cpy ) : daeElement(), domGles_texcombiner_argumentRGB_type_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGles_texcombiner_argumentRGB_type &operator=( const domGles_texcombiner_argumentRGB_type &cpy ) { (void)cpy; return *this; }

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
