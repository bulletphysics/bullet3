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
#ifndef __domFx_sampler2D_common_h__
#define __domFx_sampler2D_common_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domExtra.h>

/**
 * A two-dimensional texture sampler.
 */
class domFx_sampler2D_common_complexType 
{
public:
	class domSource;

	typedef daeSmartRef<domSource> domSourceRef;
	typedef daeTArray<domSourceRef> domSource_Array;

	class domSource : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::SOURCE; }

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
		domSource() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domSource() {}
		/**
		 * Copy Constructor
		 */
		domSource( const domSource &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domSource &operator=( const domSource &cpy ) { (void)cpy; return *this; }

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

	class domWrap_s;

	typedef daeSmartRef<domWrap_s> domWrap_sRef;
	typedef daeTArray<domWrap_sRef> domWrap_s_Array;

	class domWrap_s : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::WRAP_S; }

	protected:  // Value
		/**
		 * The domFx_sampler_wrap_common value of the text data of this element. 
		 */
		domFx_sampler_wrap_common _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a domFx_sampler_wrap_common of the value.
		 */
		domFx_sampler_wrap_common getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( domFx_sampler_wrap_common val ) { _value = val; }

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
		 * The domFx_sampler_wrap_common value of the text data of this element. 
		 */
		domFx_sampler_wrap_common _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a domFx_sampler_wrap_common of the value.
		 */
		domFx_sampler_wrap_common getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( domFx_sampler_wrap_common val ) { _value = val; }

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

	class domBorder_color;

	typedef daeSmartRef<domBorder_color> domBorder_colorRef;
	typedef daeTArray<domBorder_colorRef> domBorder_color_Array;

	class domBorder_color : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BORDER_COLOR; }

	protected:  // Value
		/**
		 * The domFx_color_common value of the text data of this element. 
		 */
		domFx_color_common _value;

	public:	//Accessors and Mutators
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
		domBorder_color() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domBorder_color() {}
		/**
		 * Copy Constructor
		 */
		domBorder_color( const domBorder_color &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBorder_color &operator=( const domBorder_color &cpy ) { (void)cpy; return *this; }

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



protected:  // Elements
	domSourceRef elemSource;
	domWrap_sRef elemWrap_s;
	domWrap_tRef elemWrap_t;
	domMinfilterRef elemMinfilter;
	domMagfilterRef elemMagfilter;
	domMipfilterRef elemMipfilter;
	domBorder_colorRef elemBorder_color;
	domMipmap_maxlevelRef elemMipmap_maxlevel;
	domMipmap_biasRef elemMipmap_bias;
	domExtra_Array elemExtra_array;

public:	//Accessors and Mutators
	/**
	 * Gets the source element.
	 * @return a daeSmartRef to the source element.
	 */
	const domSourceRef getSource() const { return elemSource; }
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
	 * Gets the border_color element.
	 * @return a daeSmartRef to the border_color element.
	 */
	const domBorder_colorRef getBorder_color() const { return elemBorder_color; }
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
	domFx_sampler2D_common_complexType() : elemSource(), elemWrap_s(), elemWrap_t(), elemMinfilter(), elemMagfilter(), elemMipfilter(), elemBorder_color(), elemMipmap_maxlevel(), elemMipmap_bias(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_sampler2D_common_complexType() {}
	/**
	 * Copy Constructor
	 */
	domFx_sampler2D_common_complexType( const domFx_sampler2D_common_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_sampler2D_common_complexType &operator=( const domFx_sampler2D_common_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domFx_sampler2D_common_complexType.
 */
class domFx_sampler2D_common : public daeElement, public domFx_sampler2D_common_complexType
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FX_SAMPLER2D_COMMON; }
protected:
	/**
	 * Constructor
	 */
	domFx_sampler2D_common() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_sampler2D_common() {}
	/**
	 * Copy Constructor
	 */
	domFx_sampler2D_common( const domFx_sampler2D_common &cpy ) : daeElement(), domFx_sampler2D_common_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_sampler2D_common &operator=( const domFx_sampler2D_common &cpy ) { (void)cpy; return *this; }

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
