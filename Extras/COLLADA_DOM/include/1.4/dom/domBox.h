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
#ifndef __domBox_h__
#define __domBox_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domExtra.h>

/**
 * An axis-aligned, centered box primitive.
 */
class domBox : public daeElement
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BOX; }
public:
	class domHalf_extents;

	typedef daeSmartRef<domHalf_extents> domHalf_extentsRef;
	typedef daeTArray<domHalf_extentsRef> domHalf_extents_Array;

/**
 * 3 float values that represent the extents of the box
 */
	class domHalf_extents : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HALF_EXTENTS; }

	protected:  // Value
		/**
		 * The domFloat3 value of the text data of this element. 
		 */
		domFloat3 _value;

	public:	//Accessors and Mutators
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
		domHalf_extents() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHalf_extents() {}
		/**
		 * Copy Constructor
		 */
		domHalf_extents( const domHalf_extents &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHalf_extents &operator=( const domHalf_extents &cpy ) { (void)cpy; return *this; }

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
 * 3 float values that represent the extents of the box @see domHalf_extents
 */
	domHalf_extentsRef elemHalf_extents;
/**
 *  The extra element may appear any number of times.  @see domExtra
 */
	domExtra_Array elemExtra_array;

public:	//Accessors and Mutators
	/**
	 * Gets the half_extents element.
	 * @return a daeSmartRef to the half_extents element.
	 */
	const domHalf_extentsRef getHalf_extents() const { return elemHalf_extents; }
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
	domBox() : elemHalf_extents(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domBox() {}
	/**
	 * Copy Constructor
	 */
	domBox( const domBox &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domBox &operator=( const domBox &cpy ) { (void)cpy; return *this; }

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
