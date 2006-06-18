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
#ifndef __domGles_texture_pipeline_h__
#define __domGles_texture_pipeline_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domExtra.h>
#include <dom/domGles_texcombiner_command_type.h>
#include <dom/domGles_texenv_command_type.h>

/**
 * Defines a set of texturing commands that will be converted into multitexturing
 * operations using glTexEnv in regular and combiner mode.
 */
class domGles_texture_pipeline_complexType 
{
protected:  // Attribute
/**
 *  The sid attribute is a text string value containing the sub-identifier
 * of this element.  This value must be unique within the scope of the parent
 * element. Optional attribute. 
 */
	xsNCName attrSid;

protected:  // Elements
/**
 * Defines a texture_pipeline command. This is a combiner-mode texturing operation.
 * @see domTexcombiner
 */
	domGles_texcombiner_command_type_Array elemTexcombiner_array;
/**
 * Defines a texture_pipeline command. It is a simple noncombiner mode of
 * texturing operations. @see domTexenv
 */
	domGles_texenv_command_type_Array elemTexenv_array;
/**
 *  The extra element may appear any number of times. OpenGL ES extensions
 * may be used here.  @see domExtra
 */
	domExtra_Array elemExtra_array;
	/**
	 * Used to preserve order in elements that do not specify strict sequencing of sub-elements.
	 */
	daeElementRefArray _contents;


public:	//Accessors and Mutators
	/**
	 * Gets the sid attribute.
	 * @return Returns a xsNCName of the sid attribute.
	 */
	xsNCName getSid() const { return attrSid; }
	/**
	 * Sets the sid attribute.
	 * @param atSid The new value for the sid attribute.
	 */
	void setSid( xsNCName atSid ) { attrSid = atSid; }

	/**
	 * Gets the texcombiner element array.
	 * @return Returns a reference to the array of texcombiner elements.
	 */
	domGles_texcombiner_command_type_Array &getTexcombiner_array() { return elemTexcombiner_array; }
	/**
	 * Gets the texcombiner element array.
	 * @return Returns a constant reference to the array of texcombiner elements.
	 */
	const domGles_texcombiner_command_type_Array &getTexcombiner_array() const { return elemTexcombiner_array; }
	/**
	 * Gets the texenv element array.
	 * @return Returns a reference to the array of texenv elements.
	 */
	domGles_texenv_command_type_Array &getTexenv_array() { return elemTexenv_array; }
	/**
	 * Gets the texenv element array.
	 * @return Returns a constant reference to the array of texenv elements.
	 */
	const domGles_texenv_command_type_Array &getTexenv_array() const { return elemTexenv_array; }
	/**
	 * Gets the extra element array.
	 * @return Returns a reference to the array of extra elements.
	 */
	domExtra_Array &getExtra_array() { return elemExtra_array; }
	/**
	 * Gets the extra element array.
	 * @return Returns a constant reference to the array of extra elements.
	 */
	const domExtra_Array &getExtra_array() const { return elemExtra_array; }
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
	domGles_texture_pipeline_complexType() : attrSid(), elemTexcombiner_array(), elemTexenv_array(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domGles_texture_pipeline_complexType() {}
	/**
	 * Copy Constructor
	 */
	domGles_texture_pipeline_complexType( const domGles_texture_pipeline_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGles_texture_pipeline_complexType &operator=( const domGles_texture_pipeline_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domGles_texture_pipeline_complexType.
 */
class domGles_texture_pipeline : public daeElement, public domGles_texture_pipeline_complexType
{
protected:
	/**
	 * Constructor
	 */
	domGles_texture_pipeline() {}
	/**
	 * Destructor
	 */
	virtual ~domGles_texture_pipeline() {}
	/**
	 * Copy Constructor
	 */
	domGles_texture_pipeline( const domGles_texture_pipeline &cpy ) : daeElement(), domGles_texture_pipeline_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGles_texture_pipeline &operator=( const domGles_texture_pipeline &cpy ) { (void)cpy; return *this; }

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
