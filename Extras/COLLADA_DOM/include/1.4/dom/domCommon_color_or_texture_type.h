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
#ifndef __domCommon_color_or_texture_type_h__
#define __domCommon_color_or_texture_type_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domExtra.h>

class domCommon_color_or_texture_type_complexType 
{
public:
	class domColor;

	typedef daeSmartRef<domColor> domColorRef;
	typedef daeTArray<domColorRef> domColor_Array;

	class domColor : public daeElement
	{
	protected:  // Attribute
		xsNCName attrSid;

	protected:  // Value
		/**
		 * The domFx_color_common value of the text data of this element. 
		 */
		domFx_color_common _value;

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

		/**
		 * Gets the _value array.
		 * @return Returns a domFx_color_common reference of the _value array.
		 */
		domFx_color_common &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domFx_color_common reference of the _value array.
		 */
		const domFx_color_common &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domFx_color_common &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domColor() : attrSid(), _value() {}
		/**
		 * Destructor
		 */
		virtual ~domColor() {}
		/**
		 * Copy Constructor
		 */
		domColor( const domColor &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domColor &operator=( const domColor &cpy ) { (void)cpy; return *this; }

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

	class domParam;

	typedef daeSmartRef<domParam> domParamRef;
	typedef daeTArray<domParamRef> domParam_Array;

	class domParam : public daeElement
	{
	protected:  // Attribute
		xsNCName attrRef;


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
		void setRef( xsNCName atRef ) { *(daeStringRef*)&attrRef = atRef;	
	 _validAttributeArray[0] = true; }

	protected:
		/**
		 * Constructor
		 */
		domParam() : attrRef() {}
		/**
		 * Destructor
		 */
		virtual ~domParam() {}
		/**
		 * Copy Constructor
		 */
		domParam( const domParam &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domParam &operator=( const domParam &cpy ) { (void)cpy; return *this; }

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

	class domTexture;

	typedef daeSmartRef<domTexture> domTextureRef;
	typedef daeTArray<domTextureRef> domTexture_Array;

	class domTexture : public daeElement
	{
	protected:  // Attributes
		xsNCName attrTexture;
		xsNCName attrTexcoord;

	protected:  // Element
		domExtraRef elemExtra;

	public:	//Accessors and Mutators
		/**
		 * Gets the texture attribute.
		 * @return Returns a xsNCName of the texture attribute.
		 */
		xsNCName getTexture() const { return attrTexture; }
		/**
		 * Sets the texture attribute.
		 * @param atTexture The new value for the texture attribute.
		 */
		void setTexture( xsNCName atTexture ) { *(daeStringRef*)&attrTexture = atTexture;	
	 _validAttributeArray[0] = true; }

		/**
		 * Gets the texcoord attribute.
		 * @return Returns a xsNCName of the texcoord attribute.
		 */
		xsNCName getTexcoord() const { return attrTexcoord; }
		/**
		 * Sets the texcoord attribute.
		 * @param atTexcoord The new value for the texcoord attribute.
		 */
		void setTexcoord( xsNCName atTexcoord ) { *(daeStringRef*)&attrTexcoord = atTexcoord;	
	 _validAttributeArray[1] = true; }

		/**
		 * Gets the extra element.
		 * @return a daeSmartRef to the extra element.
		 */
		const domExtraRef getExtra() const { return elemExtra; }
	protected:
		/**
		 * Constructor
		 */
		domTexture() : attrTexture(), attrTexcoord(), elemExtra() {}
		/**
		 * Destructor
		 */
		virtual ~domTexture() {}
		/**
		 * Copy Constructor
		 */
		domTexture( const domTexture &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domTexture &operator=( const domTexture &cpy ) { (void)cpy; return *this; }

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


protected:  // Elements
	domColorRef elemColor;
	domParamRef elemParam;
	domTextureRef elemTexture;
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
	 * Gets the color element.
	 * @return a daeSmartRef to the color element.
	 */
	const domColorRef getColor() const { return elemColor; }
	/**
	 * Gets the param element.
	 * @return a daeSmartRef to the param element.
	 */
	const domParamRef getParam() const { return elemParam; }
	/**
	 * Gets the texture element.
	 * @return a daeSmartRef to the texture element.
	 */
	const domTextureRef getTexture() const { return elemTexture; }
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
	domCommon_color_or_texture_type_complexType() : elemColor(), elemParam(), elemTexture() {}
	/**
	 * Destructor
	 */
	virtual ~domCommon_color_or_texture_type_complexType() {}
	/**
	 * Copy Constructor
	 */
	domCommon_color_or_texture_type_complexType( const domCommon_color_or_texture_type_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domCommon_color_or_texture_type_complexType &operator=( const domCommon_color_or_texture_type_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domCommon_color_or_texture_type_complexType.
 */
class domCommon_color_or_texture_type : public daeElement, public domCommon_color_or_texture_type_complexType
{
protected:
	/**
	 * Constructor
	 */
	domCommon_color_or_texture_type() {}
	/**
	 * Destructor
	 */
	virtual ~domCommon_color_or_texture_type() {}
	/**
	 * Copy Constructor
	 */
	domCommon_color_or_texture_type( const domCommon_color_or_texture_type &cpy ) : daeElement(), domCommon_color_or_texture_type_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domCommon_color_or_texture_type &operator=( const domCommon_color_or_texture_type &cpy ) { (void)cpy; return *this; }

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
