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
#ifndef __domTargetableFloat3_h__
#define __domTargetableFloat3_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>


/**
 * The TargetableFloat3 type is used to represent elements which contain a
 * float3 value which can  be targeted for animation.
 */
class domTargetableFloat3_complexType 
{
protected:  // Attribute
/**
 *  The sid attribute is a text string value containing the sub-identifier
 * of this element.  This value must be unique within the scope of the parent
 * element. Optional attribute. 
 */
	xsNCName attrSid;

protected:  // Value
	/**
	 * The domFloat3 value of the text data of this element. 
	 */
	domFloat3 _value;

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
	 * Gets the _value array.
	 * @return Returns a domFloat3 reference of the _value array.
	 */
	domFloat3 &getValue() { return _value; }
	/**
	 * Gets the _value array.
	 * @return Returns a constant domFloat3 reference of the _value array.
	 */
	const domFloat3 &getValue() const { return _value; }
	/**
	 * Sets the _value array.
	 * @param val The new value for the _value array.
	 */
	void setValue( const domFloat3 &val ) { _value = val; }

protected:
	/**
	 * Constructor
	 */
	domTargetableFloat3_complexType() : attrSid(), _value() {}
	/**
	 * Destructor
	 */
	virtual ~domTargetableFloat3_complexType() {}
	/**
	 * Copy Constructor
	 */
	domTargetableFloat3_complexType( const domTargetableFloat3_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domTargetableFloat3_complexType &operator=( const domTargetableFloat3_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domTargetableFloat3_complexType.
 */
class domTargetableFloat3 : public daeElement, public domTargetableFloat3_complexType
{

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
	domTargetableFloat3() {}
	/**
	 * Destructor
	 */
	virtual ~domTargetableFloat3() {}
	/**
	 * Copy Constructor
	 */
	domTargetableFloat3( const domTargetableFloat3 &cpy ) : daeElement(), domTargetableFloat3_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domTargetableFloat3 &operator=( const domTargetableFloat3 &cpy ) { (void)cpy; return *this; }

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
