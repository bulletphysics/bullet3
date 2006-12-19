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
#ifndef __domGles_sampler_state_h__
#define __domGles_sampler_state_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domExtra.h>

/**
 * Two-dimensional texture sampler state for profile_GLES. This is a bundle
 * of sampler-specific states that will be referenced by one or more texture_units.
 */
class domGles_sampler_state_complexType 
{
public:
	class domWrap_s;

	typedef daeSmartRef<domWrap_s> domWrap_sRef;
	typedef daeTArray<domWrap_sRef> domWrap_s_Array;

	class domWrap_s : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::WRAP_S; }

	protected:  // Value
		/**
		 * The domGles_sampler_wrap value of the text data of this element. 
		 */
		domGles_sampler_wrap _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a domGles_sampler_wrap of the value.
		 */
		domGles_sampler_wrap getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( domGles_sampler_wrap val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domWrap_s() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domWrap_s() {}
		/**
		 * Copy Constructor
		 */
		domWrap_s( const domWrap_s &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domWrap_s &operator=( const domWrap_s &cpy ) { (void)cpy; return *this; }

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

	class domWrap_t;

	typedef daeSmartRef<domWrap_t> domWrap_tRef;
	typedef daeTArray<domWrap_tRef> domWrap_t_Array;

	class domWrap_t : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::WRAP_T; }

	protected:  // Value
		/**
		 * The domGles_sampler_wrap value of the text data of this element. 
		 */
		domGles_sampler_wrap _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a domGles_sampler_wrap of the value.
		 */
		domGles_sampler_wrap getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( domGles_sampler_wrap val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domWrap_t() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domWrap_t() {}
		/**
		 * Copy Constructor
		 */
		domWrap_t( const domWrap_t &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domWrap_t &operator=( const domWrap_t &cpy ) { (void)cpy; return *this; }

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

	class domMinfilter;

	typedef daeSmartRef<domMinfilter> domMinfilterRef;
	typedef daeTArray<domMinfilterRef> domMinfilter_Array;

	class domMinfilter : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::MINFILTER; }

	protected:  // Value
		/**
		 * The domFx_sampler_filter_common value of the text data of this element. 
		 */
		domFx_sampler_filter_common _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a domFx_sampler_filter_common of the value.
		 */
		domFx_sampler_filter_common getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( domFx_sampler_filter_common val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domMinfilter() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domMinfilter() {}
		/**
		 * Copy Constructor
		 */
		domMinfilter( const domMinfilter &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domMinfilter &operator=( const domMinfilter &cpy ) { (void)cpy; return *this; }

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

	class domMagfilter;

	typedef daeSmartRef<domMagfilter> domMagfilterRef;
	typedef daeTArray<domMagfilterRef> domMagfilter_Array;

	class domMagfilter : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::MAGFILTER; }

	protected:  // Value
		/**
		 * The domFx_sampler_filter_common value of the text data of this element. 
		 */
		domFx_sampler_filter_common _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a domFx_sampler_filter_common of the value.
		 */
		domFx_sampler_filter_common getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( domFx_sampler_filter_common val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domMagfilter() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domMagfilter() {}
		/**
		 * Copy Constructor
		 */
		domMagfilter( const domMagfilter &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domMagfilter &operator=( const domMagfilter &cpy ) { (void)cpy; return *this; }

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

	class domMipfilter;

	typedef daeSmartRef<domMipfilter> domMipfilterRef;
	typedef daeTArray<domMipfilterRef> domMipfilter_Array;

	class domMipfilter : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::MIPFILTER; }

	protected:  // Value
		/**
		 * The domFx_sampler_filter_common value of the text data of this element. 
		 */
		domFx_sampler_filter_common _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a domFx_sampler_filter_common of the value.
		 */
		domFx_sampler_filter_common getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( domFx_sampler_filter_common val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domMipfilter() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domMipfilter() {}
		/**
		 * Copy Constructor
		 */
		domMipfilter( const domMipfilter &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domMipfilter &operator=( const domMipfilter &cpy ) { (void)cpy; return *this; }

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

	class domMipmap_maxlevel;

	typedef daeSmartRef<domMipmap_maxlevel> domMipmap_maxlevelRef;
	typedef daeTArray<domMipmap_maxlevelRef> domMipmap_maxlevel_Array;

	class domMipmap_maxlevel : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::MIPMAP_MAXLEVEL; }

	protected:  // Value
		/**
		 * The xsUnsignedByte value of the text data of this element. 
		 */
		xsUnsignedByte _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a xsUnsignedByte of the value.
		 */
		xsUnsignedByte getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( xsUnsignedByte val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domMipmap_maxlevel() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domMipmap_maxlevel() {}
		/**
		 * Copy Constructor
		 */
		domMipmap_maxlevel( const domMipmap_maxlevel &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domMipmap_maxlevel &operator=( const domMipmap_maxlevel &cpy ) { (void)cpy; return *this; }

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

	class domMipmap_bias;

	typedef daeSmartRef<domMipmap_bias> domMipmap_biasRef;
	typedef daeTArray<domMipmap_biasRef> domMipmap_bias_Array;

	class domMipmap_bias : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::MIPMAP_BIAS; }

	protected:  // Value
		/**
		 * The xsFloat value of the text data of this element. 
		 */
		xsFloat _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a xsFloat of the value.
		 */
		xsFloat getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( xsFloat val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domMipmap_bias() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domMipmap_bias() {}
		/**
		 * Copy Constructor
		 */
		domMipmap_bias( const domMipmap_bias &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domMipmap_bias &operator=( const domMipmap_bias &cpy ) { (void)cpy; return *this; }

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
	domWrap_sRef elemWrap_s;
	domWrap_tRef elemWrap_t;
	domMinfilterRef elemMinfilter;
	domMagfilterRef elemMagfilter;
	domMipfilterRef elemMipfilter;
	domMipmap_maxlevelRef elemMipmap_maxlevel;
	domMipmap_biasRef elemMipmap_bias;
/**
 *  The extra element may appear any number of times. OpenGL ES extensions
 * may be used here.  @see domExtra
 */
	domExtra_Array elemExtra_array;

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
	 * Gets the wrap_s element.
	 * @return a daeSmartRef to the wrap_s element.
	 */
	const domWrap_sRef getWrap_s() const { return elemWrap_s; }
	/**
	 * Gets the wrap_t element.
	 * @return a daeSmartRef to the wrap_t element.
	 */
	const domWrap_tRef getWrap_t() const { return elemWrap_t; }
	/**
	 * Gets the minfilter element.
	 * @return a daeSmartRef to the minfilter element.
	 */
	const domMinfilterRef getMinfilter() const { return elemMinfilter; }
	/**
	 * Gets the magfilter element.
	 * @return a daeSmartRef to the magfilter element.
	 */
	const domMagfilterRef getMagfilter() const { return elemMagfilter; }
	/**
	 * Gets the mipfilter element.
	 * @return a daeSmartRef to the mipfilter element.
	 */
	const domMipfilterRef getMipfilter() const { return elemMipfilter; }
	/**
	 * Gets the mipmap_maxlevel element.
	 * @return a daeSmartRef to the mipmap_maxlevel element.
	 */
	const domMipmap_maxlevelRef getMipmap_maxlevel() const { return elemMipmap_maxlevel; }
	/**
	 * Gets the mipmap_bias element.
	 * @return a daeSmartRef to the mipmap_bias element.
	 */
	const domMipmap_biasRef getMipmap_bias() const { return elemMipmap_bias; }
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
protected:
	/**
	 * Constructor
	 */
	domGles_sampler_state_complexType() : attrSid(), elemWrap_s(), elemWrap_t(), elemMinfilter(), elemMagfilter(), elemMipfilter(), elemMipmap_maxlevel(), elemMipmap_bias(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domGles_sampler_state_complexType() {}
	/**
	 * Copy Constructor
	 */
	domGles_sampler_state_complexType( const domGles_sampler_state_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGles_sampler_state_complexType &operator=( const domGles_sampler_state_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domGles_sampler_state_complexType.
 */
class domGles_sampler_state : public daeElement, public domGles_sampler_state_complexType
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::GLES_SAMPLER_STATE; }

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
	domGles_sampler_state() {}
	/**
	 * Destructor
	 */
	virtual ~domGles_sampler_state() {}
	/**
	 * Copy Constructor
	 */
	domGles_sampler_state( const domGles_sampler_state &cpy ) : daeElement(), domGles_sampler_state_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGles_sampler_state &operator=( const domGles_sampler_state &cpy ) { (void)cpy; return *this; }

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
