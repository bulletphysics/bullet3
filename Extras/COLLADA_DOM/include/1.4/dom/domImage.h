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
#ifndef __domImage_h__
#define __domImage_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domAsset.h>
#include <dom/domExtra.h>

/**
 * The image element declares the storage for the graphical representation
 * of an object.  The image element best describes raster image data, but
 * can conceivably handle other  forms of imagery. The image elements allows
 * for specifying an external image file with  the init_from element or embed
 * image data with the data element.
 */
class domImage : public daeElement
{
public:
	class domData;

	typedef daeSmartRef<domData> domDataRef;
	typedef daeTArray<domDataRef> domData_Array;

/**
 * The data child element contains a sequence of hexadecimal encoded  binary
 * octets representing  the embedded image data.
 */
	class domData : public daeElement
	{

	protected:  // Value
		/**
		 * The domListOfHexBinary value of the text data of this element. 
		 */
		domListOfHexBinary _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domListOfHexBinary reference of the _value array.
		 */
		domListOfHexBinary &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domListOfHexBinary reference of the _value array.
		 */
		const domListOfHexBinary &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domListOfHexBinary &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domData() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domData() {}
		/**
		 * Copy Constructor
		 */
		domData( const domData &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domData &operator=( const domData &cpy ) { (void)cpy; return *this; }

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

	class domInit_from;

	typedef daeSmartRef<domInit_from> domInit_fromRef;
	typedef daeTArray<domInit_fromRef> domInit_from_Array;

/**
 * The init_from element allows you to specify an external image file to use
 * for the image element.
 */
	class domInit_from : public daeElement
	{

	protected:  // Value
		/**
		 * The xsAnyURI value of the text data of this element. 
		 */
		xsAnyURI _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return Returns a xsAnyURI of the value.
		 */
		xsAnyURI &getValue() { return _value; }
		/**
		 * Gets the value of this element.
		 * @return Returns a constant xsAnyURI of the value.
		 */
		const xsAnyURI &getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( const xsAnyURI &val ) { _value.setURI( val.getURI() ); }

	protected:
		/**
		 * Constructor
		 */
		domInit_from() : _value() {}
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


protected:  // Attributes
/**
 *  The id attribute is a text string containing the unique identifier of
 * this element. This value  must be unique within the instance document.
 * Optional attribute. 
 */
	xsID attrId;
/**
 *  The name attribute is the text string name of this element. Optional attribute.
 */
	xsNCName attrName;
/**
 *  The format attribute is a text string value that indicates the image format.
 * Optional attribute. 
 */
	xsToken attrFormat;
/**
 *  The height attribute is an integer value that indicates the height of
 * the image in pixel  units. Optional attribute. 
 */
	domUint attrHeight;
/**
 *  The width attribute is an integer value that indicates the width of the
 * image in pixel units.  Optional attribute. 
 */
	domUint attrWidth;
/**
 *  The depth attribute is an integer value that indicates the depth of the
 * image in pixel units.  A 2-D image has a depth of 1, which is also the
 * default value. Optional attribute. 
 */
	domUint attrDepth;

protected:  // Elements
/**
 *  The image element may contain an asset element.  @see domAsset
 */
	domAssetRef elemAsset;
/**
 * The data child element contains a sequence of hexadecimal encoded  binary
 * octets representing  the embedded image data. @see domData
 */
	domDataRef elemData;
/**
 * The init_from element allows you to specify an external image file to use
 * for the image element. @see domInit_from
 */
	domInit_fromRef elemInit_from;
/**
 *  The extra element may appear any number of times.  @see domExtra
 */
	domExtra_Array elemExtra_array;
	/**
	 * Used to preserve order in elements that do not specify strict sequencing of sub-elements.
	 */
	daeElementRefArray _contents;


public:	//Accessors and Mutators
	/**
	 * Gets the id attribute.
	 * @return Returns a xsID of the id attribute.
	 */
	xsID getId() const { return attrId; }
	/**
	 * Sets the id attribute.
	 * @param atId The new value for the id attribute.
	 */
	void setId( xsID atId ) { attrId = atId; }

	/**
	 * Gets the name attribute.
	 * @return Returns a xsNCName of the name attribute.
	 */
	xsNCName getName() const { return attrName; }
	/**
	 * Sets the name attribute.
	 * @param atName The new value for the name attribute.
	 */
	void setName( xsNCName atName ) { attrName = atName; }

	/**
	 * Gets the format attribute.
	 * @return Returns a xsToken of the format attribute.
	 */
	xsToken getFormat() const { return attrFormat; }
	/**
	 * Sets the format attribute.
	 * @param atFormat The new value for the format attribute.
	 */
	void setFormat( xsToken atFormat ) { attrFormat = atFormat; }

	/**
	 * Gets the height attribute.
	 * @return Returns a domUint of the height attribute.
	 */
	domUint getHeight() const { return attrHeight; }
	/**
	 * Sets the height attribute.
	 * @param atHeight The new value for the height attribute.
	 */
	void setHeight( domUint atHeight ) { attrHeight = atHeight; }

	/**
	 * Gets the width attribute.
	 * @return Returns a domUint of the width attribute.
	 */
	domUint getWidth() const { return attrWidth; }
	/**
	 * Sets the width attribute.
	 * @param atWidth The new value for the width attribute.
	 */
	void setWidth( domUint atWidth ) { attrWidth = atWidth; }

	/**
	 * Gets the depth attribute.
	 * @return Returns a domUint of the depth attribute.
	 */
	domUint getDepth() const { return attrDepth; }
	/**
	 * Sets the depth attribute.
	 * @param atDepth The new value for the depth attribute.
	 */
	void setDepth( domUint atDepth ) { attrDepth = atDepth; }

	/**
	 * Gets the asset element.
	 * @return a daeSmartRef to the asset element.
	 */
	const domAssetRef getAsset() const { return elemAsset; }
	/**
	 * Gets the data element.
	 * @return a daeSmartRef to the data element.
	 */
	const domDataRef getData() const { return elemData; }
	/**
	 * Gets the init_from element.
	 * @return a daeSmartRef to the init_from element.
	 */
	const domInit_fromRef getInit_from() const { return elemInit_from; }
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
	domImage() : attrId(), attrName(), attrFormat(), attrHeight(), attrWidth(), attrDepth(), elemAsset(), elemData(), elemInit_from(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domImage() {}
	/**
	 * Copy Constructor
	 */
	domImage( const domImage &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domImage &operator=( const domImage &cpy ) { (void)cpy; return *this; }

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
