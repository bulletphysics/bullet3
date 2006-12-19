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
#ifndef __domP_h__
#define __domP_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>


/**
 * The p element represents primitive data for the primitive types (lines,
 * linestrips, polygons,  polylist, triangles, trifans, tristrips). The p
 * element contains indices that reference into  the parent's source elements
 * referenced by the input elements.
 */
class domP : public daeElement
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::P; }

protected:  // Value
	/**
	 * The domListOfUInts value of the text data of this element. 
	 */
	domListOfUInts _value;

public:	//Accessors and Mutators
	/**
	 * Gets the _value array.
	 * @return Returns a domListOfUInts reference of the _value array.
	 */
	domListOfUInts &getValue() { return _value; }
	/**
	 * Gets the _value array.
	 * @return Returns a constant domListOfUInts reference of the _value array.
	 */
	const domListOfUInts &getValue() const { return _value; }
	/**
	 * Sets the _value array.
	 * @param val The new value for the _value array.
	 */
	void setValue( const domListOfUInts &val ) { _value = val; }

protected:
	/**
	 * Constructor
	 */
	domP() : _value() {}
	/**
	 * Destructor
	 */
	virtual ~domP() {}
	/**
	 * Copy Constructor
	 */
	domP( const domP &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domP &operator=( const domP &cpy ) { (void)cpy; return *this; }

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
