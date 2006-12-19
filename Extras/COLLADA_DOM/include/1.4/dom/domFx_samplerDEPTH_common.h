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
#ifndef __domFx_samplerDEPTH_common_h__
#define __domFx_samplerDEPTH_common_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domExtra.h>

/**
 * A texture sampler for depth maps.
 */
class domFx_samplerDEPTH_common_complexType 
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



protected:  // Elements
	domSourceRef elemSource;
	domWrap_sRef elemWrap_s;
	domWrap_tRef elemWrap_t;
	domMinfilterRef elemMinfilter;
	domMagfilterRef elemMagfilter;
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
	domFx_samplerDEPTH_common_complexType() : elemSource(), elemWrap_s(), elemWrap_t(), elemMinfilter(), elemMagfilter(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_samplerDEPTH_common_complexType() {}
	/**
	 * Copy Constructor
	 */
	domFx_samplerDEPTH_common_complexType( const domFx_samplerDEPTH_common_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_samplerDEPTH_common_complexType &operator=( const domFx_samplerDEPTH_common_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domFx_samplerDEPTH_common_complexType.
 */
class domFx_samplerDEPTH_common : public daeElement, public domFx_samplerDEPTH_common_complexType
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FX_SAMPLERDEPTH_COMMON; }
protected:
	/**
	 * Constructor
	 */
	domFx_samplerDEPTH_common() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_samplerDEPTH_common() {}
	/**
	 * Copy Constructor
	 */
	domFx_samplerDEPTH_common( const domFx_samplerDEPTH_common &cpy ) : daeElement(), domFx_samplerDEPTH_common_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_samplerDEPTH_common &operator=( const domFx_samplerDEPTH_common &cpy ) { (void)cpy; return *this; }

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
