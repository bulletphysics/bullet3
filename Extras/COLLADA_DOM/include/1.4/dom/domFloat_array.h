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
#ifndef __domFloat_array_h__
#define __domFloat_array_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>


/**
 * The float_array element declares the storage for a homogenous array of
 * floating point values.
 */
class domFloat_array : public daeElement
{
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
 *  The count attribute indicates the number of values in the array. Required
 * attribute. 
 */
	domUint attrCount;
/**
 *  The digits attribute indicates the number of significant decimal digits
 * of the float values that  can be contained in the array. The default value
 * is 6. Optional attribute. 
 */
	xsShort attrDigits;
/**
 *  The magnitude attribute indicates the largest exponent of the float values
 * that can be contained  in the array. The default value is 38. Optional
 * attribute. 
 */
	xsShort attrMagnitude;

protected:  // Value
	/**
	 * The domListOfFloats value of the text data of this element. 
	 */
	domListOfFloats _value;

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
	 * Gets the count attribute.
	 * @return Returns a domUint of the count attribute.
	 */
	domUint getCount() const { return attrCount; }
	/**
	 * Sets the count attribute.
	 * @param atCount The new value for the count attribute.
	 */
	void setCount( domUint atCount ) { attrCount = atCount; }

	/**
	 * Gets the digits attribute.
	 * @return Returns a xsShort of the digits attribute.
	 */
	xsShort getDigits() const { return attrDigits; }
	/**
	 * Sets the digits attribute.
	 * @param atDigits The new value for the digits attribute.
	 */
	void setDigits( xsShort atDigits ) { attrDigits = atDigits; }

	/**
	 * Gets the magnitude attribute.
	 * @return Returns a xsShort of the magnitude attribute.
	 */
	xsShort getMagnitude() const { return attrMagnitude; }
	/**
	 * Sets the magnitude attribute.
	 * @param atMagnitude The new value for the magnitude attribute.
	 */
	void setMagnitude( xsShort atMagnitude ) { attrMagnitude = atMagnitude; }

	/**
	 * Gets the _value array.
	 * @return Returns a domListOfFloats reference of the _value array.
	 */
	domListOfFloats &getValue() { return _value; }
	/**
	 * Gets the _value array.
	 * @return Returns a constant domListOfFloats reference of the _value array.
	 */
	const domListOfFloats &getValue() const { return _value; }
	/**
	 * Sets the _value array.
	 * @param val The new value for the _value array.
	 */
	void setValue( const domListOfFloats &val ) { _value = val; }

protected:
	/**
	 * Constructor
	 */
	domFloat_array() : attrId(), attrName(), attrCount(), attrDigits(), attrMagnitude(), _value() {}
	/**
	 * Destructor
	 */
	virtual ~domFloat_array() {}
	/**
	 * Copy Constructor
	 */
	domFloat_array( const domFloat_array &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFloat_array &operator=( const domFloat_array &cpy ) { (void)cpy; return *this; }

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
