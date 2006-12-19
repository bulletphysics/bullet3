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
#ifndef __domSkew_h__
#define __domSkew_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>


/**
 * The skew element contains an angle and two mathematical vectors that represent
 * the axis of  rotation and the axis of translation.
 */
class domSkew : public daeElement
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::SKEW; }
protected:  // Attribute
/**
 *  The sid attribute is a text string value containing the sub-identifier
 * of this element.  This value must be unique within the scope of the parent
 * element. Optional attribute. 
 */
	xsNCName attrSid;

protected:  // Value
	/**
	 * The domFloat7 value of the text data of this element. 
	 */
	domFloat7 _value;

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
	 * @return Returns a domFloat7 reference of the _value array.
	 */
	domFloat7 &getValue() { return _value; }
	/**
	 * Gets the _value array.
	 * @return Returns a constant domFloat7 reference of the _value array.
	 */
	const domFloat7 &getValue() const { return _value; }
	/**
	 * Sets the _value array.
	 * @param val The new value for the _value array.
	 */
	void setValue( const domFloat7 &val ) { _value = val; }

protected:
	/**
	 * Constructor
	 */
	domSkew() : attrSid(), _value() {}
	/**
	 * Destructor
	 */
	virtual ~domSkew() {}
	/**
	 * Copy Constructor
	 */
	domSkew( const domSkew &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domSkew &operator=( const domSkew &cpy ) { (void)cpy; return *this; }

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
