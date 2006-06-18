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
#ifndef __domFx_setparam_common_h__
#define __domFx_setparam_common_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domFx_basic_type_common.h>
#include <dom/domFx_annotate_common.h>

class domFx_setparam_common_complexType 
{
protected:  // Attribute
	xsNCName attrRef;

protected:  // Elements
	domFx_annotate_common_Array elemAnnotate_array;
	domFx_basic_type_commonRef elemFx_basic_type_common;

public:	//Accessors and Mutators
	/**
	 * Gets the ref attribute.
	 * @return Returns a xsNCName of the ref attribute.
	 */
	xsNCName getRef() const { return attrRef; }
	/**
	 * Sets the ref attribute.
	 * @param atRef The new value for the ref attribute.
	 */
	void setRef( xsNCName atRef ) { attrRef = atRef; }

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
	 * Gets the fx_basic_type_common element.
	 * @return a daeSmartRef to the fx_basic_type_common element.
	 */
	const domFx_basic_type_commonRef getFx_basic_type_common() const { return elemFx_basic_type_common; }
protected:
	/**
	 * Constructor
	 */
	domFx_setparam_common_complexType() : attrRef(), elemAnnotate_array(), elemFx_basic_type_common() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_setparam_common_complexType() {}
	/**
	 * Copy Constructor
	 */
	domFx_setparam_common_complexType( const domFx_setparam_common_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_setparam_common_complexType &operator=( const domFx_setparam_common_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domFx_setparam_common_complexType.
 */
class domFx_setparam_common : public daeElement, public domFx_setparam_common_complexType
{
protected:
	/**
	 * Constructor
	 */
	domFx_setparam_common() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_setparam_common() {}
	/**
	 * Copy Constructor
	 */
	domFx_setparam_common( const domFx_setparam_common &cpy ) : daeElement(), domFx_setparam_common_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_setparam_common &operator=( const domFx_setparam_common &cpy ) { (void)cpy; return *this; }

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
