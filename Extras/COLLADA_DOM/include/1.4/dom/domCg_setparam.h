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
#ifndef __domCg_setparam_h__
#define __domCg_setparam_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domCg_param_type.h>
#include <dom/domCg_setuser_type.h>
#include <dom/domCg_setarray_type.h>
#include <dom/domCg_connect_param.h>

/**
 * Assigns a new value to a previously defined parameter.
 */
class domCg_setparam_complexType 
{
protected:  // Attributes
	domCg_identifier attrRef;
	xsNCName attrProgram;

protected:  // Elements
	domCg_param_typeRef elemCg_param_type;
	domCg_setuser_typeRef elemUsertype;
	domCg_setarray_typeRef elemArray;
	domCg_connect_paramRef elemConnect_param;
	/**
	 * Used to preserve order in elements that do not specify strict sequencing of sub-elements.
	 */
	daeElementRefArray _contents;


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
	 * Gets the program attribute.
	 * @return Returns a xsNCName of the program attribute.
	 */
	xsNCName getProgram() const { return attrProgram; }
	/**
	 * Sets the program attribute.
	 * @param atProgram The new value for the program attribute.
	 */
	void setProgram( xsNCName atProgram ) { attrProgram = atProgram; }

	/**
	 * Gets the cg_param_type element.
	 * @return a daeSmartRef to the cg_param_type element.
	 */
	const domCg_param_typeRef getCg_param_type() const { return elemCg_param_type; }
	/**
	 * Gets the usertype element.
	 * @return a daeSmartRef to the usertype element.
	 */
	const domCg_setuser_typeRef getUsertype() const { return elemUsertype; }
	/**
	 * Gets the array element.
	 * @return a daeSmartRef to the array element.
	 */
	const domCg_setarray_typeRef getArray() const { return elemArray; }
	/**
	 * Gets the connect_param element.
	 * @return a daeSmartRef to the connect_param element.
	 */
	const domCg_connect_paramRef getConnect_param() const { return elemConnect_param; }
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
	domCg_setparam_complexType() : attrRef(), attrProgram(), elemCg_param_type(), elemUsertype(), elemArray(), elemConnect_param() {}
	/**
	 * Destructor
	 */
	virtual ~domCg_setparam_complexType() {}
	/**
	 * Copy Constructor
	 */
	domCg_setparam_complexType( const domCg_setparam_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domCg_setparam_complexType &operator=( const domCg_setparam_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domCg_setparam_complexType.
 */
class domCg_setparam : public daeElement, public domCg_setparam_complexType
{
protected:
	/**
	 * Constructor
	 */
	domCg_setparam() {}
	/**
	 * Destructor
	 */
	virtual ~domCg_setparam() {}
	/**
	 * Copy Constructor
	 */
	domCg_setparam( const domCg_setparam &cpy ) : daeElement(), domCg_setparam_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domCg_setparam &operator=( const domCg_setparam &cpy ) { (void)cpy; return *this; }

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
