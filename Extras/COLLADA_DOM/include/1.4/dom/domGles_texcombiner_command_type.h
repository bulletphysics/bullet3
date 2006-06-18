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
#ifndef __domGles_texcombiner_command_type_h__
#define __domGles_texcombiner_command_type_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domGles_texture_constant_type.h>
#include <dom/domGles_texcombiner_commandRGB_type.h>
#include <dom/domGles_texcombiner_commandAlpha_type.h>

class domGles_texcombiner_command_type_complexType 
{

protected:  // Elements
	domGles_texture_constant_typeRef elemConstant;
	domGles_texcombiner_commandRGB_typeRef elemRGB;
	domGles_texcombiner_commandAlpha_typeRef elemAlpha;

public:	//Accessors and Mutators
	/**
	 * Gets the constant element.
	 * @return a daeSmartRef to the constant element.
	 */
	const domGles_texture_constant_typeRef getConstant() const { return elemConstant; }
	/**
	 * Gets the RGB element.
	 * @return a daeSmartRef to the RGB element.
	 */
	const domGles_texcombiner_commandRGB_typeRef getRGB() const { return elemRGB; }
	/**
	 * Gets the alpha element.
	 * @return a daeSmartRef to the alpha element.
	 */
	const domGles_texcombiner_commandAlpha_typeRef getAlpha() const { return elemAlpha; }
protected:
	/**
	 * Constructor
	 */
	domGles_texcombiner_command_type_complexType() : elemConstant(), elemRGB(), elemAlpha() {}
	/**
	 * Destructor
	 */
	virtual ~domGles_texcombiner_command_type_complexType() {}
	/**
	 * Copy Constructor
	 */
	domGles_texcombiner_command_type_complexType( const domGles_texcombiner_command_type_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGles_texcombiner_command_type_complexType &operator=( const domGles_texcombiner_command_type_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domGles_texcombiner_command_type_complexType.
 */
class domGles_texcombiner_command_type : public daeElement, public domGles_texcombiner_command_type_complexType
{
protected:
	/**
	 * Constructor
	 */
	domGles_texcombiner_command_type() {}
	/**
	 * Destructor
	 */
	virtual ~domGles_texcombiner_command_type() {}
	/**
	 * Copy Constructor
	 */
	domGles_texcombiner_command_type( const domGles_texcombiner_command_type &cpy ) : daeElement(), domGles_texcombiner_command_type_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGles_texcombiner_command_type &operator=( const domGles_texcombiner_command_type &cpy ) { (void)cpy; return *this; }

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
