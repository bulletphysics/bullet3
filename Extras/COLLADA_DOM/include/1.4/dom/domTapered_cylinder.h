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
#ifndef __domTapered_cylinder_h__
#define __domTapered_cylinder_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domExtra.h>

/**
 * A tapered cylinder primitive that is centered on and aligned with the local
 * Y axis.
 */
class domTapered_cylinder : public daeElement
{
public:
	class domHeight;

	typedef daeSmartRef<domHeight> domHeightRef;
	typedef daeTArray<domHeightRef> domHeight_Array;

/**
 * A float value that represents the length of the cylinder along the Y axis.
 */
	class domHeight : public daeElement
	{

	protected:  // Value
		/**
		 * The domFloat value of the text data of this element. 
		 */
		domFloat _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a domFloat of the value.
		 */
		domFloat getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( domFloat val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domHeight() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHeight() {}
		/**
		 * Copy Constructor
		 */
		domHeight( const domHeight &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHeight &operator=( const domHeight &cpy ) { (void)cpy; return *this; }

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

	class domRadius1;

	typedef daeSmartRef<domRadius1> domRadius1Ref;
	typedef daeTArray<domRadius1Ref> domRadius1_Array;

/**
 * Two float values that represent the radii of the tapered cylinder at the
 * positive (height/2)  Y value. Both ends of the tapered cylinder may be
 * elliptical.
 */
	class domRadius1 : public daeElement
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
		domRadius1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domRadius1() {}
		/**
		 * Copy Constructor
		 */
		domRadius1( const domRadius1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domRadius1 &operator=( const domRadius1 &cpy ) { (void)cpy; return *this; }

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

	class domRadius2;

	typedef daeSmartRef<domRadius2> domRadius2Ref;
	typedef daeTArray<domRadius2Ref> domRadius2_Array;

/**
 * Two float values that represent the radii of the tapered cylinder at the
 * negative (height/2)  Y value.Both ends of the tapered cylinder may be elliptical.
 */
	class domRadius2 : public daeElement
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
		domRadius2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domRadius2() {}
		/**
		 * Copy Constructor
		 */
		domRadius2( const domRadius2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domRadius2 &operator=( const domRadius2 &cpy ) { (void)cpy; return *this; }

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



protected:  // Elements
/**
 * A float value that represents the length of the cylinder along the Y axis.
 * @see domHeight
 */
	domHeightRef elemHeight;
/**
 * Two float values that represent the radii of the tapered cylinder at the
 * positive (height/2)  Y value. Both ends of the tapered cylinder may be
 * elliptical. @see domRadius1
 */
	domRadius1Ref elemRadius1;
/**
 * Two float values that represent the radii of the tapered cylinder at the
 * negative (height/2)  Y value.Both ends of the tapered cylinder may be elliptical.
 * @see domRadius2
 */
	domRadius2Ref elemRadius2;
/**
 *  The extra element may appear any number of times.  @see domExtra
 */
	domExtra_Array elemExtra_array;

public:	//Accessors and Mutators
	/**
	 * Gets the height element.
	 * @return a daeSmartRef to the height element.
	 */
	const domHeightRef getHeight() const { return elemHeight; }
	/**
	 * Gets the radius1 element.
	 * @return a daeSmartRef to the radius1 element.
	 */
	const domRadius1Ref getRadius1() const { return elemRadius1; }
	/**
	 * Gets the radius2 element.
	 * @return a daeSmartRef to the radius2 element.
	 */
	const domRadius2Ref getRadius2() const { return elemRadius2; }
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
	domTapered_cylinder() : elemHeight(), elemRadius1(), elemRadius2(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domTapered_cylinder() {}
	/**
	 * Copy Constructor
	 */
	domTapered_cylinder( const domTapered_cylinder &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domTapered_cylinder &operator=( const domTapered_cylinder &cpy ) { (void)cpy; return *this; }

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
