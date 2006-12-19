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
#ifndef __domPolygons_h__
#define __domPolygons_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domP.h>
#include <dom/domExtra.h>
#include <dom/domInputLocalOffset.h>

/**
 * The polygons element provides the information needed to bind vertex attributes
 * together and  then organize those vertices into individual polygons. The
 * polygons described can contain  arbitrary numbers of vertices. These polygons
 * may be self intersecting and may also contain holes.
 */
class domPolygons : public daeElement
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::POLYGONS; }
public:
	class domPh;

	typedef daeSmartRef<domPh> domPhRef;
	typedef daeTArray<domPhRef> domPh_Array;

/**
 * The ph element descripes a polygon with holes.
 */
	class domPh : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::PH; }
	public:
		class domH;

		typedef daeSmartRef<domH> domHRef;
		typedef daeTArray<domHRef> domH_Array;

/**
 * The h element represents a hole in the polygon specified. There must be
 * at least one h element.
 */
		class domH : public daeElement
		{
		public:
			COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::H; }

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
			domH() : _value() {}
			/**
			 * Destructor
			 */
			virtual ~domH() {}
			/**
			 * Copy Constructor
			 */
			domH( const domH &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domH &operator=( const domH &cpy ) { (void)cpy; return *this; }

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
 *  Theere may only be one p element.  @see domP
 */
		domPRef elemP;
/**
 * The h element represents a hole in the polygon specified. There must be
 * at least one h element. @see domH
 */
		domH_Array elemH_array;

	public:	//Accessors and Mutators
		/**
		 * Gets the p element.
		 * @return a daeSmartRef to the p element.
		 */
		const domPRef getP() const { return elemP; }
		/**
		 * Gets the h element array.
		 * @return Returns a reference to the array of h elements.
		 */
		domH_Array &getH_array() { return elemH_array; }
		/**
		 * Gets the h element array.
		 * @return Returns a constant reference to the array of h elements.
		 */
		const domH_Array &getH_array() const { return elemH_array; }
	protected:
		/**
		 * Constructor
		 */
		domPh() : elemP(), elemH_array() {}
		/**
		 * Destructor
		 */
		virtual ~domPh() {}
		/**
		 * Copy Constructor
		 */
		domPh( const domPh &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domPh &operator=( const domPh &cpy ) { (void)cpy; return *this; }

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


protected:  // Attributes
/**
 *  The name attribute is the text string name of this element. Optional attribute.
 */
	xsNCName attrName;
/**
 *  The count attribute indicates the number of polygon primitives. Required
 * attribute. 
 */
	domUint attrCount;
/**
 *  The material attribute declares a symbol for a material. This symbol is
 * bound to a material  at the time of instantiation. If the material attribute
 * is not specified then the lighting  and shading results are application
 * defined. Optional attribute.  
 */
	xsNCName attrMaterial;

protected:  // Elements
/**
 * The input element may occur any number of times. This input is a local
 * input with the  offset and set attributes. @see domInput
 */
	domInputLocalOffset_Array elemInput_array;
/**
 *  The p element may occur any number of times.  @see domP
 */
	domP_Array elemP_array;
/**
 * The ph element descripes a polygon with holes. @see domPh
 */
	domPh_Array elemPh_array;
/**
 *  The extra element may appear any number of times.  @see domExtra
 */
	domExtra_Array elemExtra_array;
	/**
	 * Used to preserve order in elements that do not specify strict sequencing of sub-elements.
	 */
	daeElementRefArray _contents;
	/**
	 * Used to preserve order in elements that have a complex content model.
	 */
	daeUIntArray       _contentsOrder;


public:	//Accessors and Mutators
	/**
	 * Gets the name attribute.
	 * @return Returns a xsNCName of the name attribute.
	 */
	xsNCName getName() const { return attrName; }
	/**
	 * Sets the name attribute.
	 * @param atName The new value for the name attribute.
	 */
	void setName( xsNCName atName ) { *(daeStringRef*)&attrName = atName;
	 _validAttributeArray[0] = true; }

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
	 _validAttributeArray[1] = true; }

	/**
	 * Gets the material attribute.
	 * @return Returns a xsNCName of the material attribute.
	 */
	xsNCName getMaterial() const { return attrMaterial; }
	/**
	 * Sets the material attribute.
	 * @param atMaterial The new value for the material attribute.
	 */
	void setMaterial( xsNCName atMaterial ) { *(daeStringRef*)&attrMaterial = atMaterial;
	 _validAttributeArray[2] = true; }

	/**
	 * Gets the input element array.
	 * @return Returns a reference to the array of input elements.
	 */
	domInputLocalOffset_Array &getInput_array() { return elemInput_array; }
	/**
	 * Gets the input element array.
	 * @return Returns a constant reference to the array of input elements.
	 */
	const domInputLocalOffset_Array &getInput_array() const { return elemInput_array; }
	/**
	 * Gets the p element array.
	 * @return Returns a reference to the array of p elements.
	 */
	domP_Array &getP_array() { return elemP_array; }
	/**
	 * Gets the p element array.
	 * @return Returns a constant reference to the array of p elements.
	 */
	const domP_Array &getP_array() const { return elemP_array; }
	/**
	 * Gets the ph element array.
	 * @return Returns a reference to the array of ph elements.
	 */
	domPh_Array &getPh_array() { return elemPh_array; }
	/**
	 * Gets the ph element array.
	 * @return Returns a constant reference to the array of ph elements.
	 */
	const domPh_Array &getPh_array() const { return elemPh_array; }
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
	domPolygons() : attrName(), attrCount(), attrMaterial(), elemInput_array(), elemP_array(), elemPh_array(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domPolygons() {}
	/**
	 * Copy Constructor
	 */
	domPolygons( const domPolygons &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domPolygons &operator=( const domPolygons &cpy ) { (void)cpy; return *this; }

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
