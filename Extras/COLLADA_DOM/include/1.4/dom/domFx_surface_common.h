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
#ifndef __domFx_surface_common_h__
#define __domFx_surface_common_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>


/**
 * The fx_surface_common type is used to declare a resource that can be used
 * both as the source for texture samples and as the target of a rendering
 * pass.
 */
class domFx_surface_common_complexType 
{
public:
	class domInit_from;

	typedef daeSmartRef<domInit_from> domInit_fromRef;
	typedef daeTArray<domInit_fromRef> domInit_from_Array;

/**
 * The init_from element is a list of IDREFs which specify the images to use
 * to initialize this surface.
 */
	class domInit_from : public daeElement
	{
	protected:  // Attributes
		xsUnsignedInt attrMip;
		xsUnsignedInt attrSlice;
		domFx_surface_face_enum attrFace;

	protected:  // Value
		/**
		 * The xsIDREFS value of the text data of this element. 
		 */
		xsIDREFS _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the mip attribute.
		 * @return Returns a xsUnsignedInt of the mip attribute.
		 */
		xsUnsignedInt getMip() const { return attrMip; }
		/**
		 * Sets the mip attribute.
		 * @param atMip The new value for the mip attribute.
		 */
		void setMip( xsUnsignedInt atMip ) { attrMip = atMip; }

		/**
		 * Gets the slice attribute.
		 * @return Returns a xsUnsignedInt of the slice attribute.
		 */
		xsUnsignedInt getSlice() const { return attrSlice; }
		/**
		 * Sets the slice attribute.
		 * @param atSlice The new value for the slice attribute.
		 */
		void setSlice( xsUnsignedInt atSlice ) { attrSlice = atSlice; }

		/**
		 * Gets the face attribute.
		 * @return Returns a domFx_surface_face_enum of the face attribute.
		 */
		domFx_surface_face_enum getFace() const { return attrFace; }
		/**
		 * Sets the face attribute.
		 * @param atFace The new value for the face attribute.
		 */
		void setFace( domFx_surface_face_enum atFace ) { attrFace = atFace; }

		/**
		 * Gets the _value array.
		 * @return Returns a xsIDREFS reference of the _value array.
		 */
		xsIDREFS &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant xsIDREFS reference of the _value array.
		 */
		const xsIDREFS &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const xsIDREFS &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domInit_from() : attrMip(), attrSlice(), attrFace(), _value() {}
		/**
		 * Destructor
		 */
		virtual ~domInit_from() {}
		/**
		 * Copy Constructor
		 */
		domInit_from( const domInit_from &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInit_from &operator=( const domInit_from &cpy ) { (void)cpy; return *this; }

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

	class domFormat;

	typedef daeSmartRef<domFormat> domFormatRef;
	typedef daeTArray<domFormatRef> domFormat_Array;

	class domFormat : public daeElement
	{

	protected:  // Value
		/**
		 * The xsString value of the text data of this element. 
		 */
		xsString _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a xsString of the value.
		 */
		xsString getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( xsString val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFormat() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFormat() {}
		/**
		 * Copy Constructor
		 */
		domFormat( const domFormat &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFormat &operator=( const domFormat &cpy ) { (void)cpy; return *this; }

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

	class domSize;

	typedef daeSmartRef<domSize> domSizeRef;
	typedef daeTArray<domSizeRef> domSize_Array;

	class domSize : public daeElement
	{

	protected:  // Value
		/**
		 * The domInt3 value of the text data of this element. 
		 */
		domInt3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domInt3 reference of the _value array.
		 */
		domInt3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domInt3 reference of the _value array.
		 */
		const domInt3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domInt3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domSize() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domSize() {}
		/**
		 * Copy Constructor
		 */
		domSize( const domSize &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domSize &operator=( const domSize &cpy ) { (void)cpy; return *this; }

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

	class domViewport_ratio;

	typedef daeSmartRef<domViewport_ratio> domViewport_ratioRef;
	typedef daeTArray<domViewport_ratioRef> domViewport_ratio_Array;

	class domViewport_ratio : public daeElement
	{

	protected:  // Value
		/**
		 * The domFloat2 value of the text data of this element. 
		 */
		domFloat2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domFloat2 reference of the _value array.
		 */
		domFloat2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domFloat2 reference of the _value array.
		 */
		const domFloat2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domFloat2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domViewport_ratio() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domViewport_ratio() {}
		/**
		 * Copy Constructor
		 */
		domViewport_ratio( const domViewport_ratio &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domViewport_ratio &operator=( const domViewport_ratio &cpy ) { (void)cpy; return *this; }

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

	class domMip_levels;

	typedef daeSmartRef<domMip_levels> domMip_levelsRef;
	typedef daeTArray<domMip_levelsRef> domMip_levels_Array;

	class domMip_levels : public daeElement
	{

	protected:  // Value
		/**
		 * The xsUnsignedInt value of the text data of this element. 
		 */
		xsUnsignedInt _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a xsUnsignedInt of the value.
		 */
		xsUnsignedInt getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( xsUnsignedInt val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domMip_levels() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domMip_levels() {}
		/**
		 * Copy Constructor
		 */
		domMip_levels( const domMip_levels &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domMip_levels &operator=( const domMip_levels &cpy ) { (void)cpy; return *this; }

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

	class domMipmap_generate;

	typedef daeSmartRef<domMipmap_generate> domMipmap_generateRef;
	typedef daeTArray<domMipmap_generateRef> domMipmap_generate_Array;

	class domMipmap_generate : public daeElement
	{

	protected:  // Value
		/**
		 * The xsBoolean value of the text data of this element. 
		 */
		xsBoolean _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a xsBoolean of the value.
		 */
		xsBoolean getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( xsBoolean val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domMipmap_generate() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domMipmap_generate() {}
		/**
		 * Copy Constructor
		 */
		domMipmap_generate( const domMipmap_generate &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domMipmap_generate &operator=( const domMipmap_generate &cpy ) { (void)cpy; return *this; }

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


protected:  // Attribute
	domFx_surface_type_enum attrType;

protected:  // Elements
/**
 * The init_from element is a list of IDREFs which specify the images to use
 * to initialize this surface. @see domInit_from
 */
	domInit_from_Array elemInit_from_array;
	domFormatRef elemFormat;
	domSizeRef elemSize;
	domViewport_ratioRef elemViewport_ratio;
	domMip_levelsRef elemMip_levels;
	domMipmap_generateRef elemMipmap_generate;
	/**
	 * Used to preserve order in elements that do not specify strict sequencing of sub-elements.
	 */
	daeElementRefArray _contents;


public:	//Accessors and Mutators
	/**
	 * Gets the type attribute.
	 * @return Returns a domFx_surface_type_enum of the type attribute.
	 */
	domFx_surface_type_enum getType() const { return attrType; }
	/**
	 * Sets the type attribute.
	 * @param atType The new value for the type attribute.
	 */
	void setType( domFx_surface_type_enum atType ) { attrType = atType; }

	/**
	 * Gets the init_from element array.
	 * @return Returns a reference to the array of init_from elements.
	 */
	domInit_from_Array &getInit_from_array() { return elemInit_from_array; }
	/**
	 * Gets the init_from element array.
	 * @return Returns a constant reference to the array of init_from elements.
	 */
	const domInit_from_Array &getInit_from_array() const { return elemInit_from_array; }
	/**
	 * Gets the format element.
	 * @return a daeSmartRef to the format element.
	 */
	const domFormatRef getFormat() const { return elemFormat; }
	/**
	 * Gets the size element.
	 * @return a daeSmartRef to the size element.
	 */
	const domSizeRef getSize() const { return elemSize; }
	/**
	 * Gets the viewport_ratio element.
	 * @return a daeSmartRef to the viewport_ratio element.
	 */
	const domViewport_ratioRef getViewport_ratio() const { return elemViewport_ratio; }
	/**
	 * Gets the mip_levels element.
	 * @return a daeSmartRef to the mip_levels element.
	 */
	const domMip_levelsRef getMip_levels() const { return elemMip_levels; }
	/**
	 * Gets the mipmap_generate element.
	 * @return a daeSmartRef to the mipmap_generate element.
	 */
	const domMipmap_generateRef getMipmap_generate() const { return elemMipmap_generate; }
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
	domFx_surface_common_complexType() : attrType(), elemInit_from_array(), elemFormat(), elemSize(), elemViewport_ratio(), elemMip_levels(), elemMipmap_generate() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_surface_common_complexType() {}
	/**
	 * Copy Constructor
	 */
	domFx_surface_common_complexType( const domFx_surface_common_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_surface_common_complexType &operator=( const domFx_surface_common_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domFx_surface_common_complexType.
 */
class domFx_surface_common : public daeElement, public domFx_surface_common_complexType
{
protected:
	/**
	 * Constructor
	 */
	domFx_surface_common() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_surface_common() {}
	/**
	 * Copy Constructor
	 */
	domFx_surface_common( const domFx_surface_common &cpy ) : daeElement(), domFx_surface_common_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_surface_common &operator=( const domFx_surface_common &cpy ) { (void)cpy; return *this; }

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
