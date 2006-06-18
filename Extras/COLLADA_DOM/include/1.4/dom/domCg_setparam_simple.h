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
#ifndef __domCg_setparam_simple_h__
#define __domCg_setparam_simple_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domCg_param_type.h>
#include <dom/domFx_annotate_common.h>

class domCg_setparam_simple_complexType 
{
protected:  // Attribute
	domCg_identifier attrRef;

protected:  // Elements
	domFx_annotate_common_Array elemAnnotate_array;
	domCg_param_typeRef elemCg_param_type;

public:	//Accessors and Mutators
	/**
	 * Gets the ref attribute.
	 * @return Returns a domCg_identifier of the ref attribute.
	 */
	domCg_identifier getRef() const { return attrRef; }
	/**
	 * Sets the ref attribute.
	 * @param atRef The new value for the ref attribute.
	 */
	void setRef( domCg_identifier atRef ) { attrRef = atRef; }

	/**
	 * Gets the annotate element array.
	 * @return Returns a reference to the array of annotate elements.
	 */
	domFx_annotate_common_Array &getAnnotate_array() { return elemAnnotate_array; }
	/**
	 * Gets the annotate element array.
	 * @return Returns a constant reference to the array of annotate elements.
	 */
	const domFx_annotate_common_Array &getAnnotate_array() const { return elemAnnotate_array; }
	/**
	 * Gets the cg_param_type element.
	 * @return a daeSmartRef to the cg_param_type element.
	 */
	const domCg_param_typeRef getCg_param_type() const { return elemCg_param_type; }
protected:
	/**
	 * Constructor
	 */
	domCg_setparam_simple_complexType() : attrRef(), elemAnnotate_array(), elemCg_param_type() {}
	/**
	 * Destructor
	 */
	virtual ~domCg_setparam_simple_complexType() {}
	/**
	 * Copy Constructor
	 */
	domCg_setparam_simple_complexType( const domCg_setparam_simple_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domCg_setparam_simple_complexType &operator=( const domCg_setparam_simple_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domCg_setparam_simple_complexType.
 */
class domCg_setparam_simple : public daeElement, public domCg_setparam_simple_complexType
{
protected:
	/**
	 * Constructor
	 */
	domCg_setparam_simple() {}
	/**
	 * Destructor
	 */
	virtual ~domCg_setparam_simple() {}
	/**
	 * Copy Constructor
	 */
	domCg_setparam_simple( const domCg_setparam_simple &cpy ) : daeElement(), domCg_setparam_simple_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domCg_setparam_simple &operator=( const domCg_setparam_simple &cpy ) { (void)cpy; return *this; }

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
