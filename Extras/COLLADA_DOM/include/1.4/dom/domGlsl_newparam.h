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
#ifndef __domGlsl_newparam_h__
#define __domGlsl_newparam_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domGlsl_param_type.h>
#include <dom/domFx_annotate_common.h>
#include <dom/domGlsl_newarray_type.h>

class domGlsl_newparam_complexType 
{
public:
	class domSemantic;

	typedef daeSmartRef<domSemantic> domSemanticRef;
	typedef daeTArray<domSemanticRef> domSemantic_Array;

	class domSemantic : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::SEMANTIC; }

	protected:  // Value
		/**
		 * The xsNCName value of the text data of this element. 
		 */
		xsNCName _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return Returns a xsNCName of the value.
		 */
		xsNCName getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( xsNCName val ) { *(daeStringRef*)&_value = val; }

	protected:
		/**
		 * Constructor
		 */
		domSemantic() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domSemantic() {}
		/**
		 * Copy Constructor
		 */
		domSemantic( const domSemantic &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domSemantic &operator=( const domSemantic &cpy ) { (void)cpy; return *this; }

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

	class domModifier;

	typedef daeSmartRef<domModifier> domModifierRef;
	typedef daeTArray<domModifierRef> domModifier_Array;

	class domModifier : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::MODIFIER; }

	protected:  // Value
		/**
		 * The domFx_modifier_enum_common value of the text data of this element. 
		 */
		domFx_modifier_enum_common _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a domFx_modifier_enum_common of the value.
		 */
		domFx_modifier_enum_common getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( domFx_modifier_enum_common val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domModifier() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domModifier() {}
		/**
		 * Copy Constructor
		 */
		domModifier( const domModifier &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domModifier &operator=( const domModifier &cpy ) { (void)cpy; return *this; }

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


protected:  // Attribute
	domGlsl_identifier attrSid;

protected:  // Elements
	domFx_annotate_common_Array elemAnnotate_array;
	domSemanticRef elemSemantic;
	domModifierRef elemModifier;
	domGlsl_param_typeRef elemGlsl_param_type;
	domGlsl_newarray_typeRef elemArray;
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
	 * Gets the sid attribute.
	 * @return Returns a domGlsl_identifier of the sid attribute.
	 */
	domGlsl_identifier getSid() const { return attrSid; }
	/**
	 * Sets the sid attribute.
	 * @param atSid The new value for the sid attribute.
	 */
	void setSid( domGlsl_identifier atSid ) { attrSid = atSid; }

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
	 * Gets the semantic element.
	 * @return a daeSmartRef to the semantic element.
	 */
	const domSemanticRef getSemantic() const { return elemSemantic; }
	/**
	 * Gets the modifier element.
	 * @return a daeSmartRef to the modifier element.
	 */
	const domModifierRef getModifier() const { return elemModifier; }
	/**
	 * Gets the glsl_param_type element.
	 * @return a daeSmartRef to the glsl_param_type element.
	 */
	const domGlsl_param_typeRef getGlsl_param_type() const { return elemGlsl_param_type; }
	/**
	 * Gets the array element.
	 * @return a daeSmartRef to the array element.
	 */
	const domGlsl_newarray_typeRef getArray() const { return elemArray; }
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
	domGlsl_newparam_complexType() : attrSid(), elemAnnotate_array(), elemSemantic(), elemModifier(), elemGlsl_param_type(), elemArray() {}
	/**
	 * Destructor
	 */
	virtual ~domGlsl_newparam_complexType() {}
	/**
	 * Copy Constructor
	 */
	domGlsl_newparam_complexType( const domGlsl_newparam_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGlsl_newparam_complexType &operator=( const domGlsl_newparam_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domGlsl_newparam_complexType.
 */
class domGlsl_newparam : public daeElement, public domGlsl_newparam_complexType
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::GLSL_NEWPARAM; }

public:	//Accessors and Mutators
	/**
	 * Gets the sid attribute.
	 * @return Returns a domGlsl_identifier of the sid attribute.
	 */
	domGlsl_identifier getSid() const { return attrSid; }
	/**
	 * Sets the sid attribute.
	 * @param atSid The new value for the sid attribute.
	 */
	void setSid( domGlsl_identifier atSid ) { attrSid = atSid;
	 _validAttributeArray[0] = true; }

protected:
	/**
	 * Constructor
	 */
	domGlsl_newparam() {}
	/**
	 * Destructor
	 */
	virtual ~domGlsl_newparam() {}
	/**
	 * Copy Constructor
	 */
	domGlsl_newparam( const domGlsl_newparam &cpy ) : daeElement(), domGlsl_newparam_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGlsl_newparam &operator=( const domGlsl_newparam &cpy ) { (void)cpy; return *this; }

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
