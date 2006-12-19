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
#ifndef __domFx_newparam_common_h__
#define __domFx_newparam_common_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domFx_basic_type_common.h>
#include <dom/domFx_annotate_common.h>

/**
 * This element creates a new, named param object in the FX Runtime, assigns
 * it a type, an initial value, and additional attributes at declaration time.
 */
class domFx_newparam_common_complexType 
{
public:
	class domSemantic;

	typedef daeSmartRef<domSemantic> domSemanticRef;
	typedef daeTArray<domSemanticRef> domSemantic_Array;

/**
 * The semantic element allows you to specify a semantic for this new param.
 */
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

/**
 * The modifier element allows you to specify a modifier for this new param.
 */
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
/**
 *  The sid attribute is a text string value containing the sub-identifier
 * of this element.  This value must be unique within the scope of the parent
 * element. Optional attribute. 
 */
	xsNCName attrSid;

protected:  // Elements
/**
 * The annotate element allows you to specify an annotation for this new param.
 * @see domAnnotate
 */
	domFx_annotate_common_Array elemAnnotate_array;
/**
 * The semantic element allows you to specify a semantic for this new param.
 * @see domSemantic
 */
	domSemanticRef elemSemantic;
/**
 * The modifier element allows you to specify a modifier for this new param.
 * @see domModifier
 */
	domModifierRef elemModifier;
	domFx_basic_type_commonRef elemFx_basic_type_common;

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
	void setSid( xsNCName atSid ) { *(daeStringRef*)&attrSid = atSid; }

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
	 * Gets the fx_basic_type_common element.
	 * @return a daeSmartRef to the fx_basic_type_common element.
	 */
	const domFx_basic_type_commonRef getFx_basic_type_common() const { return elemFx_basic_type_common; }
protected:
	/**
	 * Constructor
	 */
	domFx_newparam_common_complexType() : attrSid(), elemAnnotate_array(), elemSemantic(), elemModifier(), elemFx_basic_type_common() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_newparam_common_complexType() {}
	/**
	 * Copy Constructor
	 */
	domFx_newparam_common_complexType( const domFx_newparam_common_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_newparam_common_complexType &operator=( const domFx_newparam_common_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domFx_newparam_common_complexType.
 */
class domFx_newparam_common : public daeElement, public domFx_newparam_common_complexType
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FX_NEWPARAM_COMMON; }

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
	void setSid( xsNCName atSid ) { *(daeStringRef*)&attrSid = atSid;
	 _validAttributeArray[0] = true; }

protected:
	/**
	 * Constructor
	 */
	domFx_newparam_common() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_newparam_common() {}
	/**
	 * Copy Constructor
	 */
	domFx_newparam_common( const domFx_newparam_common &cpy ) : daeElement(), domFx_newparam_common_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_newparam_common &operator=( const domFx_newparam_common &cpy ) { (void)cpy; return *this; }

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
