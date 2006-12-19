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
#ifndef __domGlsl_setparam_h__
#define __domGlsl_setparam_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domGlsl_param_type.h>
#include <dom/domFx_annotate_common.h>
#include <dom/domGlsl_setarray_type.h>

class domGlsl_setparam_complexType 
{
protected:  // Attributes
	domGlsl_identifier attrRef;
	xsNCName attrProgram;

protected:  // Elements
	domFx_annotate_common_Array elemAnnotate_array;
	domGlsl_param_typeRef elemGlsl_param_type;
	domGlsl_setarray_typeRef elemArray;
	/**
	 * Used to preserve order in elements that do not specify strict sequencing of sub-elements.
	 */
	daeElementRefArray _contents;
	/**
	 * Used to preserve order in elements that have a complex content model.
	 */
	daeUIntArray       _contentsOrder;


public:	//Accessors and Mutators
	/**
	 * Gets the ref attribute.
	 * @return Returns a domGlsl_identifier of the ref attribute.
	 */
	domGlsl_identifier getRef() const { return attrRef; }
	/**
	 * Sets the ref attribute.
	 * @param atRef The new value for the ref attribute.
	 */
	void setRef( domGlsl_identifier atRef ) { attrRef = atRef; }

	/**
	 * Gets the program attribute.
	 * @return Returns a xsNCName of the program attribute.
	 */
	xsNCName getProgram() const { return attrProgram; }
	/**
	 * Sets the program attribute.
	 * @param atProgram The new value for the program attribute.
	 */
	void setProgram( xsNCName atProgram ) { *(daeStringRef*)&attrProgram = atProgram; }

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
	 * Gets the glsl_param_type element.
	 * @return a daeSmartRef to the glsl_param_type element.
	 */
	const domGlsl_param_typeRef getGlsl_param_type() const { return elemGlsl_param_type; }
	/**
	 * Gets the array element.
	 * @return a daeSmartRef to the array element.
	 */
	const domGlsl_setarray_typeRef getArray() const { return elemArray; }
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
	domGlsl_setparam_complexType() : attrRef(), attrProgram(), elemAnnotate_array(), elemGlsl_param_type(), elemArray() {}
	/**
	 * Destructor
	 */
	virtual ~domGlsl_setparam_complexType() {}
	/**
	 * Copy Constructor
	 */
	domGlsl_setparam_complexType( const domGlsl_setparam_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGlsl_setparam_complexType &operator=( const domGlsl_setparam_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domGlsl_setparam_complexType.
 */
class domGlsl_setparam : public daeElement, public domGlsl_setparam_complexType
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::GLSL_SETPARAM; }

public:	//Accessors and Mutators
	/**
	 * Gets the ref attribute.
	 * @return Returns a domGlsl_identifier of the ref attribute.
	 */
	domGlsl_identifier getRef() const { return attrRef; }
	/**
	 * Sets the ref attribute.
	 * @param atRef The new value for the ref attribute.
	 */
	void setRef( domGlsl_identifier atRef ) { attrRef = atRef;
	 _validAttributeArray[0] = true; }

	/**
	 * Gets the program attribute.
	 * @return Returns a xsNCName of the program attribute.
	 */
	xsNCName getProgram() const { return attrProgram; }
	/**
	 * Sets the program attribute.
	 * @param atProgram The new value for the program attribute.
	 */
	void setProgram( xsNCName atProgram ) { *(daeStringRef*)&attrProgram = atProgram;
	 _validAttributeArray[1] = true; }

protected:
	/**
	 * Constructor
	 */
	domGlsl_setparam() {}
	/**
	 * Destructor
	 */
	virtual ~domGlsl_setparam() {}
	/**
	 * Copy Constructor
	 */
	domGlsl_setparam( const domGlsl_setparam &cpy ) : daeElement(), domGlsl_setparam_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGlsl_setparam &operator=( const domGlsl_setparam &cpy ) { (void)cpy; return *this; }

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
