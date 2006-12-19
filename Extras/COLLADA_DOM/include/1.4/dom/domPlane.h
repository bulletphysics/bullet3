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
#ifndef __domPlane_h__
#define __domPlane_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domExtra.h>

/**
 * An infinite plane primitive.
 */
class domPlane : public daeElement
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::PLANE; }
public:
	class domEquation;

	typedef daeSmartRef<domEquation> domEquationRef;
	typedef daeTArray<domEquationRef> domEquation_Array;

/**
 * 4 float values that represent the coefficients for the plane’s equation:
 * Ax + By + Cz + D = 0
 */
	class domEquation : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::EQUATION; }

	protected:  // Value
		/**
		 * The domFloat4 value of the text data of this element. 
		 */
		domFloat4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domFloat4 reference of the _value array.
		 */
		domFloat4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domFloat4 reference of the _value array.
		 */
		const domFloat4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domFloat4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domEquation() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domEquation() {}
		/**
		 * Copy Constructor
		 */
		domEquation( const domEquation &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domEquation &operator=( const domEquation &cpy ) { (void)cpy; return *this; }

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
/**
 * 4 float values that represent the coefficients for the plane’s equation:
 * Ax + By + Cz + D = 0 @see domEquation
 */
	domEquationRef elemEquation;
/**
 *  The extra element may appear any number of times.  @see domExtra
 */
	domExtra_Array elemExtra_array;

public:	//Accessors and Mutators
	/**
	 * Gets the equation element.
	 * @return a daeSmartRef to the equation element.
	 */
	const domEquationRef getEquation() const { return elemEquation; }
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
	domPlane() : elemEquation(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domPlane() {}
	/**
	 * Copy Constructor
	 */
	domPlane( const domPlane &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domPlane &operator=( const domPlane &cpy ) { (void)cpy; return *this; }

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
