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
#ifndef __domAccessor_h__
#define __domAccessor_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domParam.h>

/**
 * The accessor element declares an access pattern to one of the array elements:
 * float_array,  int_array, Name_array, bool_array, and IDREF_array. The accessor
 * element describes access  to arrays that are organized in either an interleaved
 * or non-interleaved manner, depending  on the offset and stride attributes.
 */
class domAccessor : public daeElement
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::ACCESSOR; }
protected:  // Attributes
/**
 *  The count attribute indicates the number of times the array is accessed.
 * Required attribute. 
 */
	domUint attrCount;
/**
 *  The offset attribute indicates the index of the first value to be read
 * from the array.  The default value is 0. Optional attribute. 
 */
	domUint attrOffset;
/**
 *  The source attribute indicates the location of the array to access using
 * a URL expression. Required attribute. 
 */
	xsAnyURI attrSource;
/**
 *  The stride attribute indicates number of values to be considered a unit
 * during each access to  the array. The default value is 1, indicating that
 * a single value is accessed. Optional attribute. 
 */
	domUint attrStride;

protected:  // Element
/**
 *  The accessor element may have any number of param elements.  @see domParam
 */
	domParam_Array elemParam_array;

public:	//Accessors and Mutators
	/**
	 * Gets the count attribute.
	 * @return Returns a domUint of the count attribute.
	 */
	domUint getCount() const { return attrCount; }
	/**
	 * Sets the count attribute.
	 * @param atCount The new value for the count attribute.
	 */
	void setCount( domUint atCount ) { attrCount = atCount;
	 _validAttributeArray[0] = true; }

	/**
	 * Gets the offset attribute.
	 * @return Returns a domUint of the offset attribute.
	 */
	domUint getOffset() const { return attrOffset; }
	/**
	 * Sets the offset attribute.
	 * @param atOffset The new value for the offset attribute.
	 */
	void setOffset( domUint atOffset ) { attrOffset = atOffset;
	 _validAttributeArray[1] = true; }

	/**
	 * Gets the source attribute.
	 * @return Returns a xsAnyURI reference of the source attribute.
	 */
	xsAnyURI &getSource() { return attrSource; }
	/**
	 * Gets the source attribute.
	 * @return Returns a constant xsAnyURI reference of the source attribute.
	 */
	const xsAnyURI &getSource() const { return attrSource; }
	/**
	 * Sets the source attribute.
	 * @param atSource The new value for the source attribute.
	 */
	void setSource( const xsAnyURI &atSource ) { attrSource = atSource;
	 _validAttributeArray[2] = true; }

	/**
	 * Gets the stride attribute.
	 * @return Returns a domUint of the stride attribute.
	 */
	domUint getStride() const { return attrStride; }
	/**
	 * Sets the stride attribute.
	 * @param atStride The new value for the stride attribute.
	 */
	void setStride( domUint atStride ) { attrStride = atStride;
	 _validAttributeArray[3] = true; }

	/**
	 * Gets the param element array.
	 * @return Returns a reference to the array of param elements.
	 */
	domParam_Array &getParam_array() { return elemParam_array; }
	/**
	 * Gets the param element array.
	 * @return Returns a constant reference to the array of param elements.
	 */
	const domParam_Array &getParam_array() const { return elemParam_array; }
protected:
	/**
	 * Constructor
	 */
	domAccessor() : attrCount(), attrOffset(), attrSource(), attrStride(), elemParam_array() {}
	/**
	 * Destructor
	 */
	virtual ~domAccessor() {}
	/**
	 * Copy Constructor
	 */
	domAccessor( const domAccessor &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domAccessor &operator=( const domAccessor &cpy ) { (void)cpy; return *this; }

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
