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
#ifndef __domCg_param_type_h__
#define __domCg_param_type_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domCg_surface_type.h>
#include <dom/domCg_sampler1D.h>
#include <dom/domCg_sampler2D.h>
#include <dom/domCg_sampler3D.h>
#include <dom/domCg_samplerRECT.h>
#include <dom/domCg_samplerCUBE.h>
#include <dom/domCg_samplerDEPTH.h>

/**
 * A group that specifies the allowable types for CG profile parameters.
 */
class domCg_param_type : public daeElement
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::CG_PARAM_TYPE; }
public:
	class domBool;

	typedef daeSmartRef<domBool> domBoolRef;
	typedef daeTArray<domBoolRef> domBool_Array;

	class domBool : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BOOL; }

	protected:  // Value
		/**
		 * The domCg_bool value of the text data of this element. 
		 */
		domCg_bool _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a domCg_bool of the value.
		 */
		domCg_bool getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( domCg_bool val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domBool() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domBool() {}
		/**
		 * Copy Constructor
		 */
		domBool( const domBool &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBool &operator=( const domBool &cpy ) { (void)cpy; return *this; }

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

	class domBool1;

	typedef daeSmartRef<domBool1> domBool1Ref;
	typedef daeTArray<domBool1Ref> domBool1_Array;

	class domBool1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BOOL1; }

	protected:  // Value
		/**
		 * The domCg_bool1 value of the text data of this element. 
		 */
		domCg_bool1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a domCg_bool1 of the value.
		 */
		domCg_bool1 getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( domCg_bool1 val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domBool1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domBool1() {}
		/**
		 * Copy Constructor
		 */
		domBool1( const domBool1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBool1 &operator=( const domBool1 &cpy ) { (void)cpy; return *this; }

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

	class domBool2;

	typedef daeSmartRef<domBool2> domBool2Ref;
	typedef daeTArray<domBool2Ref> domBool2_Array;

	class domBool2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BOOL2; }

	protected:  // Value
		/**
		 * The domCg_bool2 value of the text data of this element. 
		 */
		domCg_bool2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_bool2 reference of the _value array.
		 */
		domCg_bool2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_bool2 reference of the _value array.
		 */
		const domCg_bool2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_bool2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domBool2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domBool2() {}
		/**
		 * Copy Constructor
		 */
		domBool2( const domBool2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBool2 &operator=( const domBool2 &cpy ) { (void)cpy; return *this; }

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

	class domBool3;

	typedef daeSmartRef<domBool3> domBool3Ref;
	typedef daeTArray<domBool3Ref> domBool3_Array;

	class domBool3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BOOL3; }

	protected:  // Value
		/**
		 * The domCg_bool3 value of the text data of this element. 
		 */
		domCg_bool3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_bool3 reference of the _value array.
		 */
		domCg_bool3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_bool3 reference of the _value array.
		 */
		const domCg_bool3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_bool3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domBool3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domBool3() {}
		/**
		 * Copy Constructor
		 */
		domBool3( const domBool3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBool3 &operator=( const domBool3 &cpy ) { (void)cpy; return *this; }

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

	class domBool4;

	typedef daeSmartRef<domBool4> domBool4Ref;
	typedef daeTArray<domBool4Ref> domBool4_Array;

	class domBool4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BOOL4; }

	protected:  // Value
		/**
		 * The domCg_bool4 value of the text data of this element. 
		 */
		domCg_bool4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_bool4 reference of the _value array.
		 */
		domCg_bool4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_bool4 reference of the _value array.
		 */
		const domCg_bool4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_bool4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domBool4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domBool4() {}
		/**
		 * Copy Constructor
		 */
		domBool4( const domBool4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBool4 &operator=( const domBool4 &cpy ) { (void)cpy; return *this; }

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

	class domBool1x1;

	typedef daeSmartRef<domBool1x1> domBool1x1Ref;
	typedef daeTArray<domBool1x1Ref> domBool1x1_Array;

	class domBool1x1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BOOL1X1; }

	protected:  // Value
		/**
		 * The domCg_bool1x1 value of the text data of this element. 
		 */
		domCg_bool1x1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_bool1x1 reference of the _value array.
		 */
		domCg_bool1x1 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_bool1x1 reference of the _value array.
		 */
		const domCg_bool1x1 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_bool1x1 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domBool1x1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domBool1x1() {}
		/**
		 * Copy Constructor
		 */
		domBool1x1( const domBool1x1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBool1x1 &operator=( const domBool1x1 &cpy ) { (void)cpy; return *this; }

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

	class domBool1x2;

	typedef daeSmartRef<domBool1x2> domBool1x2Ref;
	typedef daeTArray<domBool1x2Ref> domBool1x2_Array;

	class domBool1x2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BOOL1X2; }

	protected:  // Value
		/**
		 * The domCg_bool1x2 value of the text data of this element. 
		 */
		domCg_bool1x2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_bool1x2 reference of the _value array.
		 */
		domCg_bool1x2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_bool1x2 reference of the _value array.
		 */
		const domCg_bool1x2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_bool1x2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domBool1x2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domBool1x2() {}
		/**
		 * Copy Constructor
		 */
		domBool1x2( const domBool1x2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBool1x2 &operator=( const domBool1x2 &cpy ) { (void)cpy; return *this; }

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

	class domBool1x3;

	typedef daeSmartRef<domBool1x3> domBool1x3Ref;
	typedef daeTArray<domBool1x3Ref> domBool1x3_Array;

	class domBool1x3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BOOL1X3; }

	protected:  // Value
		/**
		 * The domCg_bool1x3 value of the text data of this element. 
		 */
		domCg_bool1x3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_bool1x3 reference of the _value array.
		 */
		domCg_bool1x3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_bool1x3 reference of the _value array.
		 */
		const domCg_bool1x3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_bool1x3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domBool1x3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domBool1x3() {}
		/**
		 * Copy Constructor
		 */
		domBool1x3( const domBool1x3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBool1x3 &operator=( const domBool1x3 &cpy ) { (void)cpy; return *this; }

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

	class domBool1x4;

	typedef daeSmartRef<domBool1x4> domBool1x4Ref;
	typedef daeTArray<domBool1x4Ref> domBool1x4_Array;

	class domBool1x4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BOOL1X4; }

	protected:  // Value
		/**
		 * The domCg_bool1x4 value of the text data of this element. 
		 */
		domCg_bool1x4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_bool1x4 reference of the _value array.
		 */
		domCg_bool1x4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_bool1x4 reference of the _value array.
		 */
		const domCg_bool1x4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_bool1x4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domBool1x4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domBool1x4() {}
		/**
		 * Copy Constructor
		 */
		domBool1x4( const domBool1x4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBool1x4 &operator=( const domBool1x4 &cpy ) { (void)cpy; return *this; }

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

	class domBool2x1;

	typedef daeSmartRef<domBool2x1> domBool2x1Ref;
	typedef daeTArray<domBool2x1Ref> domBool2x1_Array;

	class domBool2x1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BOOL2X1; }

	protected:  // Value
		/**
		 * The domCg_bool2x1 value of the text data of this element. 
		 */
		domCg_bool2x1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_bool2x1 reference of the _value array.
		 */
		domCg_bool2x1 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_bool2x1 reference of the _value array.
		 */
		const domCg_bool2x1 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_bool2x1 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domBool2x1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domBool2x1() {}
		/**
		 * Copy Constructor
		 */
		domBool2x1( const domBool2x1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBool2x1 &operator=( const domBool2x1 &cpy ) { (void)cpy; return *this; }

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

	class domBool2x2;

	typedef daeSmartRef<domBool2x2> domBool2x2Ref;
	typedef daeTArray<domBool2x2Ref> domBool2x2_Array;

	class domBool2x2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BOOL2X2; }

	protected:  // Value
		/**
		 * The domCg_bool2x2 value of the text data of this element. 
		 */
		domCg_bool2x2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_bool2x2 reference of the _value array.
		 */
		domCg_bool2x2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_bool2x2 reference of the _value array.
		 */
		const domCg_bool2x2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_bool2x2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domBool2x2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domBool2x2() {}
		/**
		 * Copy Constructor
		 */
		domBool2x2( const domBool2x2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBool2x2 &operator=( const domBool2x2 &cpy ) { (void)cpy; return *this; }

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

	class domBool2x3;

	typedef daeSmartRef<domBool2x3> domBool2x3Ref;
	typedef daeTArray<domBool2x3Ref> domBool2x3_Array;

	class domBool2x3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BOOL2X3; }

	protected:  // Value
		/**
		 * The domCg_bool2x3 value of the text data of this element. 
		 */
		domCg_bool2x3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_bool2x3 reference of the _value array.
		 */
		domCg_bool2x3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_bool2x3 reference of the _value array.
		 */
		const domCg_bool2x3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_bool2x3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domBool2x3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domBool2x3() {}
		/**
		 * Copy Constructor
		 */
		domBool2x3( const domBool2x3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBool2x3 &operator=( const domBool2x3 &cpy ) { (void)cpy; return *this; }

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

	class domBool2x4;

	typedef daeSmartRef<domBool2x4> domBool2x4Ref;
	typedef daeTArray<domBool2x4Ref> domBool2x4_Array;

	class domBool2x4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BOOL2X4; }

	protected:  // Value
		/**
		 * The domCg_bool2x4 value of the text data of this element. 
		 */
		domCg_bool2x4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_bool2x4 reference of the _value array.
		 */
		domCg_bool2x4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_bool2x4 reference of the _value array.
		 */
		const domCg_bool2x4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_bool2x4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domBool2x4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domBool2x4() {}
		/**
		 * Copy Constructor
		 */
		domBool2x4( const domBool2x4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBool2x4 &operator=( const domBool2x4 &cpy ) { (void)cpy; return *this; }

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

	class domBool3x1;

	typedef daeSmartRef<domBool3x1> domBool3x1Ref;
	typedef daeTArray<domBool3x1Ref> domBool3x1_Array;

	class domBool3x1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BOOL3X1; }

	protected:  // Value
		/**
		 * The domCg_bool3x1 value of the text data of this element. 
		 */
		domCg_bool3x1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_bool3x1 reference of the _value array.
		 */
		domCg_bool3x1 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_bool3x1 reference of the _value array.
		 */
		const domCg_bool3x1 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_bool3x1 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domBool3x1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domBool3x1() {}
		/**
		 * Copy Constructor
		 */
		domBool3x1( const domBool3x1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBool3x1 &operator=( const domBool3x1 &cpy ) { (void)cpy; return *this; }

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

	class domBool3x2;

	typedef daeSmartRef<domBool3x2> domBool3x2Ref;
	typedef daeTArray<domBool3x2Ref> domBool3x2_Array;

	class domBool3x2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BOOL3X2; }

	protected:  // Value
		/**
		 * The domCg_bool3x2 value of the text data of this element. 
		 */
		domCg_bool3x2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_bool3x2 reference of the _value array.
		 */
		domCg_bool3x2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_bool3x2 reference of the _value array.
		 */
		const domCg_bool3x2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_bool3x2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domBool3x2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domBool3x2() {}
		/**
		 * Copy Constructor
		 */
		domBool3x2( const domBool3x2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBool3x2 &operator=( const domBool3x2 &cpy ) { (void)cpy; return *this; }

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

	class domBool3x3;

	typedef daeSmartRef<domBool3x3> domBool3x3Ref;
	typedef daeTArray<domBool3x3Ref> domBool3x3_Array;

	class domBool3x3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BOOL3X3; }

	protected:  // Value
		/**
		 * The domCg_bool3x3 value of the text data of this element. 
		 */
		domCg_bool3x3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_bool3x3 reference of the _value array.
		 */
		domCg_bool3x3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_bool3x3 reference of the _value array.
		 */
		const domCg_bool3x3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_bool3x3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domBool3x3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domBool3x3() {}
		/**
		 * Copy Constructor
		 */
		domBool3x3( const domBool3x3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBool3x3 &operator=( const domBool3x3 &cpy ) { (void)cpy; return *this; }

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

	class domBool3x4;

	typedef daeSmartRef<domBool3x4> domBool3x4Ref;
	typedef daeTArray<domBool3x4Ref> domBool3x4_Array;

	class domBool3x4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BOOL3X4; }

	protected:  // Value
		/**
		 * The domCg_bool3x4 value of the text data of this element. 
		 */
		domCg_bool3x4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_bool3x4 reference of the _value array.
		 */
		domCg_bool3x4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_bool3x4 reference of the _value array.
		 */
		const domCg_bool3x4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_bool3x4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domBool3x4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domBool3x4() {}
		/**
		 * Copy Constructor
		 */
		domBool3x4( const domBool3x4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBool3x4 &operator=( const domBool3x4 &cpy ) { (void)cpy; return *this; }

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

	class domBool4x1;

	typedef daeSmartRef<domBool4x1> domBool4x1Ref;
	typedef daeTArray<domBool4x1Ref> domBool4x1_Array;

	class domBool4x1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BOOL4X1; }

	protected:  // Value
		/**
		 * The domCg_bool4x1 value of the text data of this element. 
		 */
		domCg_bool4x1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_bool4x1 reference of the _value array.
		 */
		domCg_bool4x1 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_bool4x1 reference of the _value array.
		 */
		const domCg_bool4x1 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_bool4x1 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domBool4x1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domBool4x1() {}
		/**
		 * Copy Constructor
		 */
		domBool4x1( const domBool4x1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBool4x1 &operator=( const domBool4x1 &cpy ) { (void)cpy; return *this; }

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

	class domBool4x2;

	typedef daeSmartRef<domBool4x2> domBool4x2Ref;
	typedef daeTArray<domBool4x2Ref> domBool4x2_Array;

	class domBool4x2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BOOL4X2; }

	protected:  // Value
		/**
		 * The domCg_bool4x2 value of the text data of this element. 
		 */
		domCg_bool4x2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_bool4x2 reference of the _value array.
		 */
		domCg_bool4x2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_bool4x2 reference of the _value array.
		 */
		const domCg_bool4x2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_bool4x2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domBool4x2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domBool4x2() {}
		/**
		 * Copy Constructor
		 */
		domBool4x2( const domBool4x2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBool4x2 &operator=( const domBool4x2 &cpy ) { (void)cpy; return *this; }

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

	class domBool4x3;

	typedef daeSmartRef<domBool4x3> domBool4x3Ref;
	typedef daeTArray<domBool4x3Ref> domBool4x3_Array;

	class domBool4x3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BOOL4X3; }

	protected:  // Value
		/**
		 * The domCg_bool4x3 value of the text data of this element. 
		 */
		domCg_bool4x3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_bool4x3 reference of the _value array.
		 */
		domCg_bool4x3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_bool4x3 reference of the _value array.
		 */
		const domCg_bool4x3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_bool4x3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domBool4x3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domBool4x3() {}
		/**
		 * Copy Constructor
		 */
		domBool4x3( const domBool4x3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBool4x3 &operator=( const domBool4x3 &cpy ) { (void)cpy; return *this; }

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

	class domBool4x4;

	typedef daeSmartRef<domBool4x4> domBool4x4Ref;
	typedef daeTArray<domBool4x4Ref> domBool4x4_Array;

	class domBool4x4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BOOL4X4; }

	protected:  // Value
		/**
		 * The domCg_bool4x4 value of the text data of this element. 
		 */
		domCg_bool4x4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_bool4x4 reference of the _value array.
		 */
		domCg_bool4x4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_bool4x4 reference of the _value array.
		 */
		const domCg_bool4x4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_bool4x4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domBool4x4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domBool4x4() {}
		/**
		 * Copy Constructor
		 */
		domBool4x4( const domBool4x4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBool4x4 &operator=( const domBool4x4 &cpy ) { (void)cpy; return *this; }

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

	class domFloat;

	typedef daeSmartRef<domFloat> domFloatRef;
	typedef daeTArray<domFloatRef> domFloat_Array;

	class domFloat : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FLOAT; }

	protected:  // Value
		/**
		 * The domCg_float value of the text data of this element. 
		 */
		domCg_float _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a domCg_float of the value.
		 */
		domCg_float getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( domCg_float val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFloat() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFloat() {}
		/**
		 * Copy Constructor
		 */
		domFloat( const domFloat &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFloat &operator=( const domFloat &cpy ) { (void)cpy; return *this; }

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

	class domFloat1;

	typedef daeSmartRef<domFloat1> domFloat1Ref;
	typedef daeTArray<domFloat1Ref> domFloat1_Array;

	class domFloat1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FLOAT1; }

	protected:  // Value
		/**
		 * The domCg_float1 value of the text data of this element. 
		 */
		domCg_float1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a domCg_float1 of the value.
		 */
		domCg_float1 getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( domCg_float1 val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFloat1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFloat1() {}
		/**
		 * Copy Constructor
		 */
		domFloat1( const domFloat1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFloat1 &operator=( const domFloat1 &cpy ) { (void)cpy; return *this; }

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

	class domFloat2;

	typedef daeSmartRef<domFloat2> domFloat2Ref;
	typedef daeTArray<domFloat2Ref> domFloat2_Array;

	class domFloat2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FLOAT2; }

	protected:  // Value
		/**
		 * The domCg_float2 value of the text data of this element. 
		 */
		domCg_float2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_float2 reference of the _value array.
		 */
		domCg_float2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_float2 reference of the _value array.
		 */
		const domCg_float2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_float2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFloat2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFloat2() {}
		/**
		 * Copy Constructor
		 */
		domFloat2( const domFloat2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFloat2 &operator=( const domFloat2 &cpy ) { (void)cpy; return *this; }

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

	class domFloat3;

	typedef daeSmartRef<domFloat3> domFloat3Ref;
	typedef daeTArray<domFloat3Ref> domFloat3_Array;

	class domFloat3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FLOAT3; }

	protected:  // Value
		/**
		 * The domCg_float3 value of the text data of this element. 
		 */
		domCg_float3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_float3 reference of the _value array.
		 */
		domCg_float3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_float3 reference of the _value array.
		 */
		const domCg_float3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_float3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFloat3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFloat3() {}
		/**
		 * Copy Constructor
		 */
		domFloat3( const domFloat3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFloat3 &operator=( const domFloat3 &cpy ) { (void)cpy; return *this; }

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

	class domFloat4;

	typedef daeSmartRef<domFloat4> domFloat4Ref;
	typedef daeTArray<domFloat4Ref> domFloat4_Array;

	class domFloat4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FLOAT4; }

	protected:  // Value
		/**
		 * The domCg_float4 value of the text data of this element. 
		 */
		domCg_float4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_float4 reference of the _value array.
		 */
		domCg_float4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_float4 reference of the _value array.
		 */
		const domCg_float4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_float4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFloat4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFloat4() {}
		/**
		 * Copy Constructor
		 */
		domFloat4( const domFloat4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFloat4 &operator=( const domFloat4 &cpy ) { (void)cpy; return *this; }

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

	class domFloat1x1;

	typedef daeSmartRef<domFloat1x1> domFloat1x1Ref;
	typedef daeTArray<domFloat1x1Ref> domFloat1x1_Array;

	class domFloat1x1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FLOAT1X1; }

	protected:  // Value
		/**
		 * The domCg_float1x1 value of the text data of this element. 
		 */
		domCg_float1x1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_float1x1 reference of the _value array.
		 */
		domCg_float1x1 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_float1x1 reference of the _value array.
		 */
		const domCg_float1x1 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_float1x1 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFloat1x1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFloat1x1() {}
		/**
		 * Copy Constructor
		 */
		domFloat1x1( const domFloat1x1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFloat1x1 &operator=( const domFloat1x1 &cpy ) { (void)cpy; return *this; }

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

	class domFloat1x2;

	typedef daeSmartRef<domFloat1x2> domFloat1x2Ref;
	typedef daeTArray<domFloat1x2Ref> domFloat1x2_Array;

	class domFloat1x2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FLOAT1X2; }

	protected:  // Value
		/**
		 * The domCg_float1x2 value of the text data of this element. 
		 */
		domCg_float1x2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_float1x2 reference of the _value array.
		 */
		domCg_float1x2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_float1x2 reference of the _value array.
		 */
		const domCg_float1x2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_float1x2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFloat1x2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFloat1x2() {}
		/**
		 * Copy Constructor
		 */
		domFloat1x2( const domFloat1x2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFloat1x2 &operator=( const domFloat1x2 &cpy ) { (void)cpy; return *this; }

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

	class domFloat1x3;

	typedef daeSmartRef<domFloat1x3> domFloat1x3Ref;
	typedef daeTArray<domFloat1x3Ref> domFloat1x3_Array;

	class domFloat1x3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FLOAT1X3; }

	protected:  // Value
		/**
		 * The domCg_float1x3 value of the text data of this element. 
		 */
		domCg_float1x3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_float1x3 reference of the _value array.
		 */
		domCg_float1x3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_float1x3 reference of the _value array.
		 */
		const domCg_float1x3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_float1x3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFloat1x3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFloat1x3() {}
		/**
		 * Copy Constructor
		 */
		domFloat1x3( const domFloat1x3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFloat1x3 &operator=( const domFloat1x3 &cpy ) { (void)cpy; return *this; }

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

	class domFloat1x4;

	typedef daeSmartRef<domFloat1x4> domFloat1x4Ref;
	typedef daeTArray<domFloat1x4Ref> domFloat1x4_Array;

	class domFloat1x4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FLOAT1X4; }

	protected:  // Value
		/**
		 * The domCg_float1x4 value of the text data of this element. 
		 */
		domCg_float1x4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_float1x4 reference of the _value array.
		 */
		domCg_float1x4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_float1x4 reference of the _value array.
		 */
		const domCg_float1x4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_float1x4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFloat1x4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFloat1x4() {}
		/**
		 * Copy Constructor
		 */
		domFloat1x4( const domFloat1x4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFloat1x4 &operator=( const domFloat1x4 &cpy ) { (void)cpy; return *this; }

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

	class domFloat2x1;

	typedef daeSmartRef<domFloat2x1> domFloat2x1Ref;
	typedef daeTArray<domFloat2x1Ref> domFloat2x1_Array;

	class domFloat2x1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FLOAT2X1; }

	protected:  // Value
		/**
		 * The domCg_float2x1 value of the text data of this element. 
		 */
		domCg_float2x1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_float2x1 reference of the _value array.
		 */
		domCg_float2x1 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_float2x1 reference of the _value array.
		 */
		const domCg_float2x1 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_float2x1 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFloat2x1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFloat2x1() {}
		/**
		 * Copy Constructor
		 */
		domFloat2x1( const domFloat2x1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFloat2x1 &operator=( const domFloat2x1 &cpy ) { (void)cpy; return *this; }

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

	class domFloat2x2;

	typedef daeSmartRef<domFloat2x2> domFloat2x2Ref;
	typedef daeTArray<domFloat2x2Ref> domFloat2x2_Array;

	class domFloat2x2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FLOAT2X2; }

	protected:  // Value
		/**
		 * The domCg_float2x2 value of the text data of this element. 
		 */
		domCg_float2x2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_float2x2 reference of the _value array.
		 */
		domCg_float2x2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_float2x2 reference of the _value array.
		 */
		const domCg_float2x2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_float2x2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFloat2x2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFloat2x2() {}
		/**
		 * Copy Constructor
		 */
		domFloat2x2( const domFloat2x2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFloat2x2 &operator=( const domFloat2x2 &cpy ) { (void)cpy; return *this; }

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

	class domFloat2x3;

	typedef daeSmartRef<domFloat2x3> domFloat2x3Ref;
	typedef daeTArray<domFloat2x3Ref> domFloat2x3_Array;

	class domFloat2x3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FLOAT2X3; }

	protected:  // Value
		/**
		 * The domCg_float2x3 value of the text data of this element. 
		 */
		domCg_float2x3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_float2x3 reference of the _value array.
		 */
		domCg_float2x3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_float2x3 reference of the _value array.
		 */
		const domCg_float2x3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_float2x3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFloat2x3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFloat2x3() {}
		/**
		 * Copy Constructor
		 */
		domFloat2x3( const domFloat2x3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFloat2x3 &operator=( const domFloat2x3 &cpy ) { (void)cpy; return *this; }

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

	class domFloat2x4;

	typedef daeSmartRef<domFloat2x4> domFloat2x4Ref;
	typedef daeTArray<domFloat2x4Ref> domFloat2x4_Array;

	class domFloat2x4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FLOAT2X4; }

	protected:  // Value
		/**
		 * The domCg_float2x4 value of the text data of this element. 
		 */
		domCg_float2x4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_float2x4 reference of the _value array.
		 */
		domCg_float2x4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_float2x4 reference of the _value array.
		 */
		const domCg_float2x4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_float2x4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFloat2x4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFloat2x4() {}
		/**
		 * Copy Constructor
		 */
		domFloat2x4( const domFloat2x4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFloat2x4 &operator=( const domFloat2x4 &cpy ) { (void)cpy; return *this; }

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

	class domFloat3x1;

	typedef daeSmartRef<domFloat3x1> domFloat3x1Ref;
	typedef daeTArray<domFloat3x1Ref> domFloat3x1_Array;

	class domFloat3x1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FLOAT3X1; }

	protected:  // Value
		/**
		 * The domCg_float3x1 value of the text data of this element. 
		 */
		domCg_float3x1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_float3x1 reference of the _value array.
		 */
		domCg_float3x1 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_float3x1 reference of the _value array.
		 */
		const domCg_float3x1 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_float3x1 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFloat3x1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFloat3x1() {}
		/**
		 * Copy Constructor
		 */
		domFloat3x1( const domFloat3x1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFloat3x1 &operator=( const domFloat3x1 &cpy ) { (void)cpy; return *this; }

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

	class domFloat3x2;

	typedef daeSmartRef<domFloat3x2> domFloat3x2Ref;
	typedef daeTArray<domFloat3x2Ref> domFloat3x2_Array;

	class domFloat3x2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FLOAT3X2; }

	protected:  // Value
		/**
		 * The domCg_float3x2 value of the text data of this element. 
		 */
		domCg_float3x2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_float3x2 reference of the _value array.
		 */
		domCg_float3x2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_float3x2 reference of the _value array.
		 */
		const domCg_float3x2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_float3x2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFloat3x2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFloat3x2() {}
		/**
		 * Copy Constructor
		 */
		domFloat3x2( const domFloat3x2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFloat3x2 &operator=( const domFloat3x2 &cpy ) { (void)cpy; return *this; }

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

	class domFloat3x3;

	typedef daeSmartRef<domFloat3x3> domFloat3x3Ref;
	typedef daeTArray<domFloat3x3Ref> domFloat3x3_Array;

	class domFloat3x3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FLOAT3X3; }

	protected:  // Value
		/**
		 * The domCg_float3x3 value of the text data of this element. 
		 */
		domCg_float3x3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_float3x3 reference of the _value array.
		 */
		domCg_float3x3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_float3x3 reference of the _value array.
		 */
		const domCg_float3x3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_float3x3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFloat3x3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFloat3x3() {}
		/**
		 * Copy Constructor
		 */
		domFloat3x3( const domFloat3x3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFloat3x3 &operator=( const domFloat3x3 &cpy ) { (void)cpy; return *this; }

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

	class domFloat3x4;

	typedef daeSmartRef<domFloat3x4> domFloat3x4Ref;
	typedef daeTArray<domFloat3x4Ref> domFloat3x4_Array;

	class domFloat3x4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FLOAT3X4; }

	protected:  // Value
		/**
		 * The domCg_float3x4 value of the text data of this element. 
		 */
		domCg_float3x4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_float3x4 reference of the _value array.
		 */
		domCg_float3x4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_float3x4 reference of the _value array.
		 */
		const domCg_float3x4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_float3x4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFloat3x4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFloat3x4() {}
		/**
		 * Copy Constructor
		 */
		domFloat3x4( const domFloat3x4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFloat3x4 &operator=( const domFloat3x4 &cpy ) { (void)cpy; return *this; }

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

	class domFloat4x1;

	typedef daeSmartRef<domFloat4x1> domFloat4x1Ref;
	typedef daeTArray<domFloat4x1Ref> domFloat4x1_Array;

	class domFloat4x1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FLOAT4X1; }

	protected:  // Value
		/**
		 * The domCg_float4x1 value of the text data of this element. 
		 */
		domCg_float4x1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_float4x1 reference of the _value array.
		 */
		domCg_float4x1 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_float4x1 reference of the _value array.
		 */
		const domCg_float4x1 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_float4x1 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFloat4x1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFloat4x1() {}
		/**
		 * Copy Constructor
		 */
		domFloat4x1( const domFloat4x1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFloat4x1 &operator=( const domFloat4x1 &cpy ) { (void)cpy; return *this; }

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

	class domFloat4x2;

	typedef daeSmartRef<domFloat4x2> domFloat4x2Ref;
	typedef daeTArray<domFloat4x2Ref> domFloat4x2_Array;

	class domFloat4x2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FLOAT4X2; }

	protected:  // Value
		/**
		 * The domCg_float4x2 value of the text data of this element. 
		 */
		domCg_float4x2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_float4x2 reference of the _value array.
		 */
		domCg_float4x2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_float4x2 reference of the _value array.
		 */
		const domCg_float4x2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_float4x2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFloat4x2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFloat4x2() {}
		/**
		 * Copy Constructor
		 */
		domFloat4x2( const domFloat4x2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFloat4x2 &operator=( const domFloat4x2 &cpy ) { (void)cpy; return *this; }

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

	class domFloat4x3;

	typedef daeSmartRef<domFloat4x3> domFloat4x3Ref;
	typedef daeTArray<domFloat4x3Ref> domFloat4x3_Array;

	class domFloat4x3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FLOAT4X3; }

	protected:  // Value
		/**
		 * The domCg_float4x3 value of the text data of this element. 
		 */
		domCg_float4x3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_float4x3 reference of the _value array.
		 */
		domCg_float4x3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_float4x3 reference of the _value array.
		 */
		const domCg_float4x3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_float4x3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFloat4x3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFloat4x3() {}
		/**
		 * Copy Constructor
		 */
		domFloat4x3( const domFloat4x3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFloat4x3 &operator=( const domFloat4x3 &cpy ) { (void)cpy; return *this; }

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

	class domFloat4x4;

	typedef daeSmartRef<domFloat4x4> domFloat4x4Ref;
	typedef daeTArray<domFloat4x4Ref> domFloat4x4_Array;

	class domFloat4x4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FLOAT4X4; }

	protected:  // Value
		/**
		 * The domCg_float4x4 value of the text data of this element. 
		 */
		domCg_float4x4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_float4x4 reference of the _value array.
		 */
		domCg_float4x4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_float4x4 reference of the _value array.
		 */
		const domCg_float4x4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_float4x4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFloat4x4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFloat4x4() {}
		/**
		 * Copy Constructor
		 */
		domFloat4x4( const domFloat4x4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFloat4x4 &operator=( const domFloat4x4 &cpy ) { (void)cpy; return *this; }

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

	class domInt;

	typedef daeSmartRef<domInt> domIntRef;
	typedef daeTArray<domIntRef> domInt_Array;

	class domInt : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INT; }

	protected:  // Value
		/**
		 * The domCg_int value of the text data of this element. 
		 */
		domCg_int _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a domCg_int of the value.
		 */
		domCg_int getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( domCg_int val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domInt() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domInt() {}
		/**
		 * Copy Constructor
		 */
		domInt( const domInt &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInt &operator=( const domInt &cpy ) { (void)cpy; return *this; }

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

	class domInt1;

	typedef daeSmartRef<domInt1> domInt1Ref;
	typedef daeTArray<domInt1Ref> domInt1_Array;

	class domInt1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INT1; }

	protected:  // Value
		/**
		 * The domCg_int1 value of the text data of this element. 
		 */
		domCg_int1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a domCg_int1 of the value.
		 */
		domCg_int1 getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( domCg_int1 val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domInt1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domInt1() {}
		/**
		 * Copy Constructor
		 */
		domInt1( const domInt1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInt1 &operator=( const domInt1 &cpy ) { (void)cpy; return *this; }

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

	class domInt2;

	typedef daeSmartRef<domInt2> domInt2Ref;
	typedef daeTArray<domInt2Ref> domInt2_Array;

	class domInt2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INT2; }

	protected:  // Value
		/**
		 * The domCg_int2 value of the text data of this element. 
		 */
		domCg_int2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_int2 reference of the _value array.
		 */
		domCg_int2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_int2 reference of the _value array.
		 */
		const domCg_int2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_int2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domInt2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domInt2() {}
		/**
		 * Copy Constructor
		 */
		domInt2( const domInt2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInt2 &operator=( const domInt2 &cpy ) { (void)cpy; return *this; }

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

	class domInt3;

	typedef daeSmartRef<domInt3> domInt3Ref;
	typedef daeTArray<domInt3Ref> domInt3_Array;

	class domInt3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INT3; }

	protected:  // Value
		/**
		 * The domCg_int3 value of the text data of this element. 
		 */
		domCg_int3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_int3 reference of the _value array.
		 */
		domCg_int3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_int3 reference of the _value array.
		 */
		const domCg_int3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_int3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domInt3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domInt3() {}
		/**
		 * Copy Constructor
		 */
		domInt3( const domInt3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInt3 &operator=( const domInt3 &cpy ) { (void)cpy; return *this; }

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

	class domInt4;

	typedef daeSmartRef<domInt4> domInt4Ref;
	typedef daeTArray<domInt4Ref> domInt4_Array;

	class domInt4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INT4; }

	protected:  // Value
		/**
		 * The domCg_int4 value of the text data of this element. 
		 */
		domCg_int4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_int4 reference of the _value array.
		 */
		domCg_int4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_int4 reference of the _value array.
		 */
		const domCg_int4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_int4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domInt4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domInt4() {}
		/**
		 * Copy Constructor
		 */
		domInt4( const domInt4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInt4 &operator=( const domInt4 &cpy ) { (void)cpy; return *this; }

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

	class domInt1x1;

	typedef daeSmartRef<domInt1x1> domInt1x1Ref;
	typedef daeTArray<domInt1x1Ref> domInt1x1_Array;

	class domInt1x1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INT1X1; }

	protected:  // Value
		/**
		 * The domCg_int1x1 value of the text data of this element. 
		 */
		domCg_int1x1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_int1x1 reference of the _value array.
		 */
		domCg_int1x1 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_int1x1 reference of the _value array.
		 */
		const domCg_int1x1 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_int1x1 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domInt1x1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domInt1x1() {}
		/**
		 * Copy Constructor
		 */
		domInt1x1( const domInt1x1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInt1x1 &operator=( const domInt1x1 &cpy ) { (void)cpy; return *this; }

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

	class domInt1x2;

	typedef daeSmartRef<domInt1x2> domInt1x2Ref;
	typedef daeTArray<domInt1x2Ref> domInt1x2_Array;

	class domInt1x2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INT1X2; }

	protected:  // Value
		/**
		 * The domCg_int1x2 value of the text data of this element. 
		 */
		domCg_int1x2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_int1x2 reference of the _value array.
		 */
		domCg_int1x2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_int1x2 reference of the _value array.
		 */
		const domCg_int1x2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_int1x2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domInt1x2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domInt1x2() {}
		/**
		 * Copy Constructor
		 */
		domInt1x2( const domInt1x2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInt1x2 &operator=( const domInt1x2 &cpy ) { (void)cpy; return *this; }

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

	class domInt1x3;

	typedef daeSmartRef<domInt1x3> domInt1x3Ref;
	typedef daeTArray<domInt1x3Ref> domInt1x3_Array;

	class domInt1x3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INT1X3; }

	protected:  // Value
		/**
		 * The domCg_int1x3 value of the text data of this element. 
		 */
		domCg_int1x3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_int1x3 reference of the _value array.
		 */
		domCg_int1x3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_int1x3 reference of the _value array.
		 */
		const domCg_int1x3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_int1x3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domInt1x3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domInt1x3() {}
		/**
		 * Copy Constructor
		 */
		domInt1x3( const domInt1x3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInt1x3 &operator=( const domInt1x3 &cpy ) { (void)cpy; return *this; }

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

	class domInt1x4;

	typedef daeSmartRef<domInt1x4> domInt1x4Ref;
	typedef daeTArray<domInt1x4Ref> domInt1x4_Array;

	class domInt1x4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INT1X4; }

	protected:  // Value
		/**
		 * The domCg_int1x4 value of the text data of this element. 
		 */
		domCg_int1x4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_int1x4 reference of the _value array.
		 */
		domCg_int1x4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_int1x4 reference of the _value array.
		 */
		const domCg_int1x4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_int1x4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domInt1x4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domInt1x4() {}
		/**
		 * Copy Constructor
		 */
		domInt1x4( const domInt1x4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInt1x4 &operator=( const domInt1x4 &cpy ) { (void)cpy; return *this; }

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

	class domInt2x1;

	typedef daeSmartRef<domInt2x1> domInt2x1Ref;
	typedef daeTArray<domInt2x1Ref> domInt2x1_Array;

	class domInt2x1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INT2X1; }

	protected:  // Value
		/**
		 * The domCg_int2x1 value of the text data of this element. 
		 */
		domCg_int2x1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_int2x1 reference of the _value array.
		 */
		domCg_int2x1 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_int2x1 reference of the _value array.
		 */
		const domCg_int2x1 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_int2x1 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domInt2x1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domInt2x1() {}
		/**
		 * Copy Constructor
		 */
		domInt2x1( const domInt2x1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInt2x1 &operator=( const domInt2x1 &cpy ) { (void)cpy; return *this; }

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

	class domInt2x2;

	typedef daeSmartRef<domInt2x2> domInt2x2Ref;
	typedef daeTArray<domInt2x2Ref> domInt2x2_Array;

	class domInt2x2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INT2X2; }

	protected:  // Value
		/**
		 * The domCg_int2x2 value of the text data of this element. 
		 */
		domCg_int2x2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_int2x2 reference of the _value array.
		 */
		domCg_int2x2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_int2x2 reference of the _value array.
		 */
		const domCg_int2x2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_int2x2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domInt2x2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domInt2x2() {}
		/**
		 * Copy Constructor
		 */
		domInt2x2( const domInt2x2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInt2x2 &operator=( const domInt2x2 &cpy ) { (void)cpy; return *this; }

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

	class domInt2x3;

	typedef daeSmartRef<domInt2x3> domInt2x3Ref;
	typedef daeTArray<domInt2x3Ref> domInt2x3_Array;

	class domInt2x3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INT2X3; }

	protected:  // Value
		/**
		 * The domCg_int2x3 value of the text data of this element. 
		 */
		domCg_int2x3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_int2x3 reference of the _value array.
		 */
		domCg_int2x3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_int2x3 reference of the _value array.
		 */
		const domCg_int2x3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_int2x3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domInt2x3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domInt2x3() {}
		/**
		 * Copy Constructor
		 */
		domInt2x3( const domInt2x3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInt2x3 &operator=( const domInt2x3 &cpy ) { (void)cpy; return *this; }

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

	class domInt2x4;

	typedef daeSmartRef<domInt2x4> domInt2x4Ref;
	typedef daeTArray<domInt2x4Ref> domInt2x4_Array;

	class domInt2x4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INT2X4; }

	protected:  // Value
		/**
		 * The domCg_int2x4 value of the text data of this element. 
		 */
		domCg_int2x4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_int2x4 reference of the _value array.
		 */
		domCg_int2x4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_int2x4 reference of the _value array.
		 */
		const domCg_int2x4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_int2x4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domInt2x4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domInt2x4() {}
		/**
		 * Copy Constructor
		 */
		domInt2x4( const domInt2x4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInt2x4 &operator=( const domInt2x4 &cpy ) { (void)cpy; return *this; }

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

	class domInt3x1;

	typedef daeSmartRef<domInt3x1> domInt3x1Ref;
	typedef daeTArray<domInt3x1Ref> domInt3x1_Array;

	class domInt3x1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INT3X1; }

	protected:  // Value
		/**
		 * The domCg_int3x1 value of the text data of this element. 
		 */
		domCg_int3x1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_int3x1 reference of the _value array.
		 */
		domCg_int3x1 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_int3x1 reference of the _value array.
		 */
		const domCg_int3x1 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_int3x1 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domInt3x1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domInt3x1() {}
		/**
		 * Copy Constructor
		 */
		domInt3x1( const domInt3x1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInt3x1 &operator=( const domInt3x1 &cpy ) { (void)cpy; return *this; }

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

	class domInt3x2;

	typedef daeSmartRef<domInt3x2> domInt3x2Ref;
	typedef daeTArray<domInt3x2Ref> domInt3x2_Array;

	class domInt3x2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INT3X2; }

	protected:  // Value
		/**
		 * The domCg_int3x2 value of the text data of this element. 
		 */
		domCg_int3x2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_int3x2 reference of the _value array.
		 */
		domCg_int3x2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_int3x2 reference of the _value array.
		 */
		const domCg_int3x2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_int3x2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domInt3x2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domInt3x2() {}
		/**
		 * Copy Constructor
		 */
		domInt3x2( const domInt3x2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInt3x2 &operator=( const domInt3x2 &cpy ) { (void)cpy; return *this; }

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

	class domInt3x3;

	typedef daeSmartRef<domInt3x3> domInt3x3Ref;
	typedef daeTArray<domInt3x3Ref> domInt3x3_Array;

	class domInt3x3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INT3X3; }

	protected:  // Value
		/**
		 * The domCg_int3x3 value of the text data of this element. 
		 */
		domCg_int3x3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_int3x3 reference of the _value array.
		 */
		domCg_int3x3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_int3x3 reference of the _value array.
		 */
		const domCg_int3x3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_int3x3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domInt3x3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domInt3x3() {}
		/**
		 * Copy Constructor
		 */
		domInt3x3( const domInt3x3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInt3x3 &operator=( const domInt3x3 &cpy ) { (void)cpy; return *this; }

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

	class domInt3x4;

	typedef daeSmartRef<domInt3x4> domInt3x4Ref;
	typedef daeTArray<domInt3x4Ref> domInt3x4_Array;

	class domInt3x4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INT3X4; }

	protected:  // Value
		/**
		 * The domCg_int3x4 value of the text data of this element. 
		 */
		domCg_int3x4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_int3x4 reference of the _value array.
		 */
		domCg_int3x4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_int3x4 reference of the _value array.
		 */
		const domCg_int3x4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_int3x4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domInt3x4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domInt3x4() {}
		/**
		 * Copy Constructor
		 */
		domInt3x4( const domInt3x4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInt3x4 &operator=( const domInt3x4 &cpy ) { (void)cpy; return *this; }

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

	class domInt4x1;

	typedef daeSmartRef<domInt4x1> domInt4x1Ref;
	typedef daeTArray<domInt4x1Ref> domInt4x1_Array;

	class domInt4x1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INT4X1; }

	protected:  // Value
		/**
		 * The domCg_int4x1 value of the text data of this element. 
		 */
		domCg_int4x1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_int4x1 reference of the _value array.
		 */
		domCg_int4x1 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_int4x1 reference of the _value array.
		 */
		const domCg_int4x1 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_int4x1 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domInt4x1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domInt4x1() {}
		/**
		 * Copy Constructor
		 */
		domInt4x1( const domInt4x1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInt4x1 &operator=( const domInt4x1 &cpy ) { (void)cpy; return *this; }

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

	class domInt4x2;

	typedef daeSmartRef<domInt4x2> domInt4x2Ref;
	typedef daeTArray<domInt4x2Ref> domInt4x2_Array;

	class domInt4x2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INT4X2; }

	protected:  // Value
		/**
		 * The domCg_int4x2 value of the text data of this element. 
		 */
		domCg_int4x2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_int4x2 reference of the _value array.
		 */
		domCg_int4x2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_int4x2 reference of the _value array.
		 */
		const domCg_int4x2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_int4x2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domInt4x2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domInt4x2() {}
		/**
		 * Copy Constructor
		 */
		domInt4x2( const domInt4x2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInt4x2 &operator=( const domInt4x2 &cpy ) { (void)cpy; return *this; }

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

	class domInt4x3;

	typedef daeSmartRef<domInt4x3> domInt4x3Ref;
	typedef daeTArray<domInt4x3Ref> domInt4x3_Array;

	class domInt4x3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INT4X3; }

	protected:  // Value
		/**
		 * The domCg_int4x3 value of the text data of this element. 
		 */
		domCg_int4x3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_int4x3 reference of the _value array.
		 */
		domCg_int4x3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_int4x3 reference of the _value array.
		 */
		const domCg_int4x3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_int4x3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domInt4x3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domInt4x3() {}
		/**
		 * Copy Constructor
		 */
		domInt4x3( const domInt4x3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInt4x3 &operator=( const domInt4x3 &cpy ) { (void)cpy; return *this; }

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

	class domInt4x4;

	typedef daeSmartRef<domInt4x4> domInt4x4Ref;
	typedef daeTArray<domInt4x4Ref> domInt4x4_Array;

	class domInt4x4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INT4X4; }

	protected:  // Value
		/**
		 * The domCg_int4x4 value of the text data of this element. 
		 */
		domCg_int4x4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_int4x4 reference of the _value array.
		 */
		domCg_int4x4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_int4x4 reference of the _value array.
		 */
		const domCg_int4x4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_int4x4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domInt4x4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domInt4x4() {}
		/**
		 * Copy Constructor
		 */
		domInt4x4( const domInt4x4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInt4x4 &operator=( const domInt4x4 &cpy ) { (void)cpy; return *this; }

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

	class domHalf;

	typedef daeSmartRef<domHalf> domHalfRef;
	typedef daeTArray<domHalfRef> domHalf_Array;

	class domHalf : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HALF; }

	protected:  // Value
		/**
		 * The domCg_half value of the text data of this element. 
		 */
		domCg_half _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a domCg_half of the value.
		 */
		domCg_half getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( domCg_half val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domHalf() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHalf() {}
		/**
		 * Copy Constructor
		 */
		domHalf( const domHalf &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHalf &operator=( const domHalf &cpy ) { (void)cpy; return *this; }

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

	class domHalf1;

	typedef daeSmartRef<domHalf1> domHalf1Ref;
	typedef daeTArray<domHalf1Ref> domHalf1_Array;

	class domHalf1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HALF1; }

	protected:  // Value
		/**
		 * The domCg_half1 value of the text data of this element. 
		 */
		domCg_half1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a domCg_half1 of the value.
		 */
		domCg_half1 getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( domCg_half1 val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domHalf1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHalf1() {}
		/**
		 * Copy Constructor
		 */
		domHalf1( const domHalf1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHalf1 &operator=( const domHalf1 &cpy ) { (void)cpy; return *this; }

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

	class domHalf2;

	typedef daeSmartRef<domHalf2> domHalf2Ref;
	typedef daeTArray<domHalf2Ref> domHalf2_Array;

	class domHalf2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HALF2; }

	protected:  // Value
		/**
		 * The domCg_half2 value of the text data of this element. 
		 */
		domCg_half2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_half2 reference of the _value array.
		 */
		domCg_half2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_half2 reference of the _value array.
		 */
		const domCg_half2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_half2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domHalf2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHalf2() {}
		/**
		 * Copy Constructor
		 */
		domHalf2( const domHalf2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHalf2 &operator=( const domHalf2 &cpy ) { (void)cpy; return *this; }

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

	class domHalf3;

	typedef daeSmartRef<domHalf3> domHalf3Ref;
	typedef daeTArray<domHalf3Ref> domHalf3_Array;

	class domHalf3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HALF3; }

	protected:  // Value
		/**
		 * The domCg_half3 value of the text data of this element. 
		 */
		domCg_half3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_half3 reference of the _value array.
		 */
		domCg_half3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_half3 reference of the _value array.
		 */
		const domCg_half3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_half3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domHalf3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHalf3() {}
		/**
		 * Copy Constructor
		 */
		domHalf3( const domHalf3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHalf3 &operator=( const domHalf3 &cpy ) { (void)cpy; return *this; }

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

	class domHalf4;

	typedef daeSmartRef<domHalf4> domHalf4Ref;
	typedef daeTArray<domHalf4Ref> domHalf4_Array;

	class domHalf4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HALF4; }

	protected:  // Value
		/**
		 * The domCg_half4 value of the text data of this element. 
		 */
		domCg_half4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_half4 reference of the _value array.
		 */
		domCg_half4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_half4 reference of the _value array.
		 */
		const domCg_half4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_half4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domHalf4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHalf4() {}
		/**
		 * Copy Constructor
		 */
		domHalf4( const domHalf4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHalf4 &operator=( const domHalf4 &cpy ) { (void)cpy; return *this; }

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

	class domHalf1x1;

	typedef daeSmartRef<domHalf1x1> domHalf1x1Ref;
	typedef daeTArray<domHalf1x1Ref> domHalf1x1_Array;

	class domHalf1x1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HALF1X1; }

	protected:  // Value
		/**
		 * The domCg_half1x1 value of the text data of this element. 
		 */
		domCg_half1x1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_half1x1 reference of the _value array.
		 */
		domCg_half1x1 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_half1x1 reference of the _value array.
		 */
		const domCg_half1x1 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_half1x1 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domHalf1x1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHalf1x1() {}
		/**
		 * Copy Constructor
		 */
		domHalf1x1( const domHalf1x1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHalf1x1 &operator=( const domHalf1x1 &cpy ) { (void)cpy; return *this; }

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

	class domHalf1x2;

	typedef daeSmartRef<domHalf1x2> domHalf1x2Ref;
	typedef daeTArray<domHalf1x2Ref> domHalf1x2_Array;

	class domHalf1x2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HALF1X2; }

	protected:  // Value
		/**
		 * The domCg_half1x2 value of the text data of this element. 
		 */
		domCg_half1x2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_half1x2 reference of the _value array.
		 */
		domCg_half1x2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_half1x2 reference of the _value array.
		 */
		const domCg_half1x2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_half1x2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domHalf1x2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHalf1x2() {}
		/**
		 * Copy Constructor
		 */
		domHalf1x2( const domHalf1x2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHalf1x2 &operator=( const domHalf1x2 &cpy ) { (void)cpy; return *this; }

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

	class domHalf1x3;

	typedef daeSmartRef<domHalf1x3> domHalf1x3Ref;
	typedef daeTArray<domHalf1x3Ref> domHalf1x3_Array;

	class domHalf1x3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HALF1X3; }

	protected:  // Value
		/**
		 * The domCg_half1x3 value of the text data of this element. 
		 */
		domCg_half1x3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_half1x3 reference of the _value array.
		 */
		domCg_half1x3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_half1x3 reference of the _value array.
		 */
		const domCg_half1x3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_half1x3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domHalf1x3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHalf1x3() {}
		/**
		 * Copy Constructor
		 */
		domHalf1x3( const domHalf1x3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHalf1x3 &operator=( const domHalf1x3 &cpy ) { (void)cpy; return *this; }

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

	class domHalf1x4;

	typedef daeSmartRef<domHalf1x4> domHalf1x4Ref;
	typedef daeTArray<domHalf1x4Ref> domHalf1x4_Array;

	class domHalf1x4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HALF1X4; }

	protected:  // Value
		/**
		 * The domCg_half1x4 value of the text data of this element. 
		 */
		domCg_half1x4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_half1x4 reference of the _value array.
		 */
		domCg_half1x4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_half1x4 reference of the _value array.
		 */
		const domCg_half1x4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_half1x4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domHalf1x4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHalf1x4() {}
		/**
		 * Copy Constructor
		 */
		domHalf1x4( const domHalf1x4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHalf1x4 &operator=( const domHalf1x4 &cpy ) { (void)cpy; return *this; }

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

	class domHalf2x1;

	typedef daeSmartRef<domHalf2x1> domHalf2x1Ref;
	typedef daeTArray<domHalf2x1Ref> domHalf2x1_Array;

	class domHalf2x1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HALF2X1; }

	protected:  // Value
		/**
		 * The domCg_half2x1 value of the text data of this element. 
		 */
		domCg_half2x1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_half2x1 reference of the _value array.
		 */
		domCg_half2x1 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_half2x1 reference of the _value array.
		 */
		const domCg_half2x1 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_half2x1 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domHalf2x1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHalf2x1() {}
		/**
		 * Copy Constructor
		 */
		domHalf2x1( const domHalf2x1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHalf2x1 &operator=( const domHalf2x1 &cpy ) { (void)cpy; return *this; }

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

	class domHalf2x2;

	typedef daeSmartRef<domHalf2x2> domHalf2x2Ref;
	typedef daeTArray<domHalf2x2Ref> domHalf2x2_Array;

	class domHalf2x2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HALF2X2; }

	protected:  // Value
		/**
		 * The domCg_half2x2 value of the text data of this element. 
		 */
		domCg_half2x2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_half2x2 reference of the _value array.
		 */
		domCg_half2x2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_half2x2 reference of the _value array.
		 */
		const domCg_half2x2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_half2x2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domHalf2x2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHalf2x2() {}
		/**
		 * Copy Constructor
		 */
		domHalf2x2( const domHalf2x2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHalf2x2 &operator=( const domHalf2x2 &cpy ) { (void)cpy; return *this; }

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

	class domHalf2x3;

	typedef daeSmartRef<domHalf2x3> domHalf2x3Ref;
	typedef daeTArray<domHalf2x3Ref> domHalf2x3_Array;

	class domHalf2x3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HALF2X3; }

	protected:  // Value
		/**
		 * The domCg_half2x3 value of the text data of this element. 
		 */
		domCg_half2x3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_half2x3 reference of the _value array.
		 */
		domCg_half2x3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_half2x3 reference of the _value array.
		 */
		const domCg_half2x3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_half2x3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domHalf2x3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHalf2x3() {}
		/**
		 * Copy Constructor
		 */
		domHalf2x3( const domHalf2x3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHalf2x3 &operator=( const domHalf2x3 &cpy ) { (void)cpy; return *this; }

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

	class domHalf2x4;

	typedef daeSmartRef<domHalf2x4> domHalf2x4Ref;
	typedef daeTArray<domHalf2x4Ref> domHalf2x4_Array;

	class domHalf2x4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HALF2X4; }

	protected:  // Value
		/**
		 * The domCg_half2x4 value of the text data of this element. 
		 */
		domCg_half2x4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_half2x4 reference of the _value array.
		 */
		domCg_half2x4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_half2x4 reference of the _value array.
		 */
		const domCg_half2x4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_half2x4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domHalf2x4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHalf2x4() {}
		/**
		 * Copy Constructor
		 */
		domHalf2x4( const domHalf2x4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHalf2x4 &operator=( const domHalf2x4 &cpy ) { (void)cpy; return *this; }

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

	class domHalf3x1;

	typedef daeSmartRef<domHalf3x1> domHalf3x1Ref;
	typedef daeTArray<domHalf3x1Ref> domHalf3x1_Array;

	class domHalf3x1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HALF3X1; }

	protected:  // Value
		/**
		 * The domCg_half3x1 value of the text data of this element. 
		 */
		domCg_half3x1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_half3x1 reference of the _value array.
		 */
		domCg_half3x1 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_half3x1 reference of the _value array.
		 */
		const domCg_half3x1 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_half3x1 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domHalf3x1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHalf3x1() {}
		/**
		 * Copy Constructor
		 */
		domHalf3x1( const domHalf3x1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHalf3x1 &operator=( const domHalf3x1 &cpy ) { (void)cpy; return *this; }

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

	class domHalf3x2;

	typedef daeSmartRef<domHalf3x2> domHalf3x2Ref;
	typedef daeTArray<domHalf3x2Ref> domHalf3x2_Array;

	class domHalf3x2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HALF3X2; }

	protected:  // Value
		/**
		 * The domCg_half3x2 value of the text data of this element. 
		 */
		domCg_half3x2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_half3x2 reference of the _value array.
		 */
		domCg_half3x2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_half3x2 reference of the _value array.
		 */
		const domCg_half3x2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_half3x2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domHalf3x2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHalf3x2() {}
		/**
		 * Copy Constructor
		 */
		domHalf3x2( const domHalf3x2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHalf3x2 &operator=( const domHalf3x2 &cpy ) { (void)cpy; return *this; }

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

	class domHalf3x3;

	typedef daeSmartRef<domHalf3x3> domHalf3x3Ref;
	typedef daeTArray<domHalf3x3Ref> domHalf3x3_Array;

	class domHalf3x3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HALF3X3; }

	protected:  // Value
		/**
		 * The domCg_half3x3 value of the text data of this element. 
		 */
		domCg_half3x3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_half3x3 reference of the _value array.
		 */
		domCg_half3x3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_half3x3 reference of the _value array.
		 */
		const domCg_half3x3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_half3x3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domHalf3x3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHalf3x3() {}
		/**
		 * Copy Constructor
		 */
		domHalf3x3( const domHalf3x3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHalf3x3 &operator=( const domHalf3x3 &cpy ) { (void)cpy; return *this; }

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

	class domHalf3x4;

	typedef daeSmartRef<domHalf3x4> domHalf3x4Ref;
	typedef daeTArray<domHalf3x4Ref> domHalf3x4_Array;

	class domHalf3x4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HALF3X4; }

	protected:  // Value
		/**
		 * The domCg_half3x4 value of the text data of this element. 
		 */
		domCg_half3x4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_half3x4 reference of the _value array.
		 */
		domCg_half3x4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_half3x4 reference of the _value array.
		 */
		const domCg_half3x4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_half3x4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domHalf3x4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHalf3x4() {}
		/**
		 * Copy Constructor
		 */
		domHalf3x4( const domHalf3x4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHalf3x4 &operator=( const domHalf3x4 &cpy ) { (void)cpy; return *this; }

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

	class domHalf4x1;

	typedef daeSmartRef<domHalf4x1> domHalf4x1Ref;
	typedef daeTArray<domHalf4x1Ref> domHalf4x1_Array;

	class domHalf4x1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HALF4X1; }

	protected:  // Value
		/**
		 * The domCg_half4x1 value of the text data of this element. 
		 */
		domCg_half4x1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_half4x1 reference of the _value array.
		 */
		domCg_half4x1 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_half4x1 reference of the _value array.
		 */
		const domCg_half4x1 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_half4x1 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domHalf4x1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHalf4x1() {}
		/**
		 * Copy Constructor
		 */
		domHalf4x1( const domHalf4x1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHalf4x1 &operator=( const domHalf4x1 &cpy ) { (void)cpy; return *this; }

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

	class domHalf4x2;

	typedef daeSmartRef<domHalf4x2> domHalf4x2Ref;
	typedef daeTArray<domHalf4x2Ref> domHalf4x2_Array;

	class domHalf4x2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HALF4X2; }

	protected:  // Value
		/**
		 * The domCg_half4x2 value of the text data of this element. 
		 */
		domCg_half4x2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_half4x2 reference of the _value array.
		 */
		domCg_half4x2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_half4x2 reference of the _value array.
		 */
		const domCg_half4x2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_half4x2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domHalf4x2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHalf4x2() {}
		/**
		 * Copy Constructor
		 */
		domHalf4x2( const domHalf4x2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHalf4x2 &operator=( const domHalf4x2 &cpy ) { (void)cpy; return *this; }

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

	class domHalf4x3;

	typedef daeSmartRef<domHalf4x3> domHalf4x3Ref;
	typedef daeTArray<domHalf4x3Ref> domHalf4x3_Array;

	class domHalf4x3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HALF4X3; }

	protected:  // Value
		/**
		 * The domCg_half4x3 value of the text data of this element. 
		 */
		domCg_half4x3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_half4x3 reference of the _value array.
		 */
		domCg_half4x3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_half4x3 reference of the _value array.
		 */
		const domCg_half4x3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_half4x3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domHalf4x3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHalf4x3() {}
		/**
		 * Copy Constructor
		 */
		domHalf4x3( const domHalf4x3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHalf4x3 &operator=( const domHalf4x3 &cpy ) { (void)cpy; return *this; }

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

	class domHalf4x4;

	typedef daeSmartRef<domHalf4x4> domHalf4x4Ref;
	typedef daeTArray<domHalf4x4Ref> domHalf4x4_Array;

	class domHalf4x4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HALF4X4; }

	protected:  // Value
		/**
		 * The domCg_half4x4 value of the text data of this element. 
		 */
		domCg_half4x4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_half4x4 reference of the _value array.
		 */
		domCg_half4x4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_half4x4 reference of the _value array.
		 */
		const domCg_half4x4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_half4x4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domHalf4x4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domHalf4x4() {}
		/**
		 * Copy Constructor
		 */
		domHalf4x4( const domHalf4x4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domHalf4x4 &operator=( const domHalf4x4 &cpy ) { (void)cpy; return *this; }

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

	class domFixed;

	typedef daeSmartRef<domFixed> domFixedRef;
	typedef daeTArray<domFixedRef> domFixed_Array;

	class domFixed : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FIXED; }

	protected:  // Value
		/**
		 * The domCg_fixed value of the text data of this element. 
		 */
		domCg_fixed _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a domCg_fixed of the value.
		 */
		domCg_fixed getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( domCg_fixed val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFixed() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFixed() {}
		/**
		 * Copy Constructor
		 */
		domFixed( const domFixed &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFixed &operator=( const domFixed &cpy ) { (void)cpy; return *this; }

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

	class domFixed1;

	typedef daeSmartRef<domFixed1> domFixed1Ref;
	typedef daeTArray<domFixed1Ref> domFixed1_Array;

	class domFixed1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FIXED1; }

	protected:  // Value
		/**
		 * The domCg_fixed1 value of the text data of this element. 
		 */
		domCg_fixed1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a domCg_fixed1 of the value.
		 */
		domCg_fixed1 getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( domCg_fixed1 val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFixed1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFixed1() {}
		/**
		 * Copy Constructor
		 */
		domFixed1( const domFixed1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFixed1 &operator=( const domFixed1 &cpy ) { (void)cpy; return *this; }

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

	class domFixed2;

	typedef daeSmartRef<domFixed2> domFixed2Ref;
	typedef daeTArray<domFixed2Ref> domFixed2_Array;

	class domFixed2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FIXED2; }

	protected:  // Value
		/**
		 * The domCg_fixed2 value of the text data of this element. 
		 */
		domCg_fixed2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_fixed2 reference of the _value array.
		 */
		domCg_fixed2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_fixed2 reference of the _value array.
		 */
		const domCg_fixed2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_fixed2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFixed2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFixed2() {}
		/**
		 * Copy Constructor
		 */
		domFixed2( const domFixed2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFixed2 &operator=( const domFixed2 &cpy ) { (void)cpy; return *this; }

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

	class domFixed3;

	typedef daeSmartRef<domFixed3> domFixed3Ref;
	typedef daeTArray<domFixed3Ref> domFixed3_Array;

	class domFixed3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FIXED3; }

	protected:  // Value
		/**
		 * The domCg_fixed3 value of the text data of this element. 
		 */
		domCg_fixed3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_fixed3 reference of the _value array.
		 */
		domCg_fixed3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_fixed3 reference of the _value array.
		 */
		const domCg_fixed3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_fixed3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFixed3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFixed3() {}
		/**
		 * Copy Constructor
		 */
		domFixed3( const domFixed3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFixed3 &operator=( const domFixed3 &cpy ) { (void)cpy; return *this; }

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

	class domFixed4;

	typedef daeSmartRef<domFixed4> domFixed4Ref;
	typedef daeTArray<domFixed4Ref> domFixed4_Array;

	class domFixed4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FIXED4; }

	protected:  // Value
		/**
		 * The domCg_fixed4 value of the text data of this element. 
		 */
		domCg_fixed4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_fixed4 reference of the _value array.
		 */
		domCg_fixed4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_fixed4 reference of the _value array.
		 */
		const domCg_fixed4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_fixed4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFixed4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFixed4() {}
		/**
		 * Copy Constructor
		 */
		domFixed4( const domFixed4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFixed4 &operator=( const domFixed4 &cpy ) { (void)cpy; return *this; }

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

	class domFixed1x1;

	typedef daeSmartRef<domFixed1x1> domFixed1x1Ref;
	typedef daeTArray<domFixed1x1Ref> domFixed1x1_Array;

	class domFixed1x1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FIXED1X1; }

	protected:  // Value
		/**
		 * The domCg_fixed1x1 value of the text data of this element. 
		 */
		domCg_fixed1x1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_fixed1x1 reference of the _value array.
		 */
		domCg_fixed1x1 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_fixed1x1 reference of the _value array.
		 */
		const domCg_fixed1x1 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_fixed1x1 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFixed1x1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFixed1x1() {}
		/**
		 * Copy Constructor
		 */
		domFixed1x1( const domFixed1x1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFixed1x1 &operator=( const domFixed1x1 &cpy ) { (void)cpy; return *this; }

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

	class domFixed1x2;

	typedef daeSmartRef<domFixed1x2> domFixed1x2Ref;
	typedef daeTArray<domFixed1x2Ref> domFixed1x2_Array;

	class domFixed1x2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FIXED1X2; }

	protected:  // Value
		/**
		 * The domCg_fixed1x2 value of the text data of this element. 
		 */
		domCg_fixed1x2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_fixed1x2 reference of the _value array.
		 */
		domCg_fixed1x2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_fixed1x2 reference of the _value array.
		 */
		const domCg_fixed1x2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_fixed1x2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFixed1x2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFixed1x2() {}
		/**
		 * Copy Constructor
		 */
		domFixed1x2( const domFixed1x2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFixed1x2 &operator=( const domFixed1x2 &cpy ) { (void)cpy; return *this; }

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

	class domFixed1x3;

	typedef daeSmartRef<domFixed1x3> domFixed1x3Ref;
	typedef daeTArray<domFixed1x3Ref> domFixed1x3_Array;

	class domFixed1x3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FIXED1X3; }

	protected:  // Value
		/**
		 * The domCg_fixed1x3 value of the text data of this element. 
		 */
		domCg_fixed1x3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_fixed1x3 reference of the _value array.
		 */
		domCg_fixed1x3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_fixed1x3 reference of the _value array.
		 */
		const domCg_fixed1x3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_fixed1x3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFixed1x3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFixed1x3() {}
		/**
		 * Copy Constructor
		 */
		domFixed1x3( const domFixed1x3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFixed1x3 &operator=( const domFixed1x3 &cpy ) { (void)cpy; return *this; }

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

	class domFixed1x4;

	typedef daeSmartRef<domFixed1x4> domFixed1x4Ref;
	typedef daeTArray<domFixed1x4Ref> domFixed1x4_Array;

	class domFixed1x4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FIXED1X4; }

	protected:  // Value
		/**
		 * The domCg_fixed1x4 value of the text data of this element. 
		 */
		domCg_fixed1x4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_fixed1x4 reference of the _value array.
		 */
		domCg_fixed1x4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_fixed1x4 reference of the _value array.
		 */
		const domCg_fixed1x4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_fixed1x4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFixed1x4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFixed1x4() {}
		/**
		 * Copy Constructor
		 */
		domFixed1x4( const domFixed1x4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFixed1x4 &operator=( const domFixed1x4 &cpy ) { (void)cpy; return *this; }

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

	class domFixed2x1;

	typedef daeSmartRef<domFixed2x1> domFixed2x1Ref;
	typedef daeTArray<domFixed2x1Ref> domFixed2x1_Array;

	class domFixed2x1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FIXED2X1; }

	protected:  // Value
		/**
		 * The domCg_fixed2x1 value of the text data of this element. 
		 */
		domCg_fixed2x1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_fixed2x1 reference of the _value array.
		 */
		domCg_fixed2x1 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_fixed2x1 reference of the _value array.
		 */
		const domCg_fixed2x1 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_fixed2x1 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFixed2x1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFixed2x1() {}
		/**
		 * Copy Constructor
		 */
		domFixed2x1( const domFixed2x1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFixed2x1 &operator=( const domFixed2x1 &cpy ) { (void)cpy; return *this; }

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

	class domFixed2x2;

	typedef daeSmartRef<domFixed2x2> domFixed2x2Ref;
	typedef daeTArray<domFixed2x2Ref> domFixed2x2_Array;

	class domFixed2x2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FIXED2X2; }

	protected:  // Value
		/**
		 * The domCg_fixed2x2 value of the text data of this element. 
		 */
		domCg_fixed2x2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_fixed2x2 reference of the _value array.
		 */
		domCg_fixed2x2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_fixed2x2 reference of the _value array.
		 */
		const domCg_fixed2x2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_fixed2x2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFixed2x2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFixed2x2() {}
		/**
		 * Copy Constructor
		 */
		domFixed2x2( const domFixed2x2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFixed2x2 &operator=( const domFixed2x2 &cpy ) { (void)cpy; return *this; }

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

	class domFixed2x3;

	typedef daeSmartRef<domFixed2x3> domFixed2x3Ref;
	typedef daeTArray<domFixed2x3Ref> domFixed2x3_Array;

	class domFixed2x3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FIXED2X3; }

	protected:  // Value
		/**
		 * The domCg_fixed2x3 value of the text data of this element. 
		 */
		domCg_fixed2x3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_fixed2x3 reference of the _value array.
		 */
		domCg_fixed2x3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_fixed2x3 reference of the _value array.
		 */
		const domCg_fixed2x3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_fixed2x3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFixed2x3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFixed2x3() {}
		/**
		 * Copy Constructor
		 */
		domFixed2x3( const domFixed2x3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFixed2x3 &operator=( const domFixed2x3 &cpy ) { (void)cpy; return *this; }

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

	class domFixed2x4;

	typedef daeSmartRef<domFixed2x4> domFixed2x4Ref;
	typedef daeTArray<domFixed2x4Ref> domFixed2x4_Array;

	class domFixed2x4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FIXED2X4; }

	protected:  // Value
		/**
		 * The domCg_fixed2x4 value of the text data of this element. 
		 */
		domCg_fixed2x4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_fixed2x4 reference of the _value array.
		 */
		domCg_fixed2x4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_fixed2x4 reference of the _value array.
		 */
		const domCg_fixed2x4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_fixed2x4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFixed2x4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFixed2x4() {}
		/**
		 * Copy Constructor
		 */
		domFixed2x4( const domFixed2x4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFixed2x4 &operator=( const domFixed2x4 &cpy ) { (void)cpy; return *this; }

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

	class domFixed3x1;

	typedef daeSmartRef<domFixed3x1> domFixed3x1Ref;
	typedef daeTArray<domFixed3x1Ref> domFixed3x1_Array;

	class domFixed3x1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FIXED3X1; }

	protected:  // Value
		/**
		 * The domCg_fixed3x1 value of the text data of this element. 
		 */
		domCg_fixed3x1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_fixed3x1 reference of the _value array.
		 */
		domCg_fixed3x1 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_fixed3x1 reference of the _value array.
		 */
		const domCg_fixed3x1 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_fixed3x1 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFixed3x1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFixed3x1() {}
		/**
		 * Copy Constructor
		 */
		domFixed3x1( const domFixed3x1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFixed3x1 &operator=( const domFixed3x1 &cpy ) { (void)cpy; return *this; }

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

	class domFixed3x2;

	typedef daeSmartRef<domFixed3x2> domFixed3x2Ref;
	typedef daeTArray<domFixed3x2Ref> domFixed3x2_Array;

	class domFixed3x2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FIXED3X2; }

	protected:  // Value
		/**
		 * The domCg_fixed3x2 value of the text data of this element. 
		 */
		domCg_fixed3x2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_fixed3x2 reference of the _value array.
		 */
		domCg_fixed3x2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_fixed3x2 reference of the _value array.
		 */
		const domCg_fixed3x2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_fixed3x2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFixed3x2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFixed3x2() {}
		/**
		 * Copy Constructor
		 */
		domFixed3x2( const domFixed3x2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFixed3x2 &operator=( const domFixed3x2 &cpy ) { (void)cpy; return *this; }

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

	class domFixed3x3;

	typedef daeSmartRef<domFixed3x3> domFixed3x3Ref;
	typedef daeTArray<domFixed3x3Ref> domFixed3x3_Array;

	class domFixed3x3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FIXED3X3; }

	protected:  // Value
		/**
		 * The domCg_fixed3x3 value of the text data of this element. 
		 */
		domCg_fixed3x3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_fixed3x3 reference of the _value array.
		 */
		domCg_fixed3x3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_fixed3x3 reference of the _value array.
		 */
		const domCg_fixed3x3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_fixed3x3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFixed3x3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFixed3x3() {}
		/**
		 * Copy Constructor
		 */
		domFixed3x3( const domFixed3x3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFixed3x3 &operator=( const domFixed3x3 &cpy ) { (void)cpy; return *this; }

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

	class domFixed3x4;

	typedef daeSmartRef<domFixed3x4> domFixed3x4Ref;
	typedef daeTArray<domFixed3x4Ref> domFixed3x4_Array;

	class domFixed3x4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FIXED3X4; }

	protected:  // Value
		/**
		 * The domCg_fixed3x4 value of the text data of this element. 
		 */
		domCg_fixed3x4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_fixed3x4 reference of the _value array.
		 */
		domCg_fixed3x4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_fixed3x4 reference of the _value array.
		 */
		const domCg_fixed3x4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_fixed3x4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFixed3x4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFixed3x4() {}
		/**
		 * Copy Constructor
		 */
		domFixed3x4( const domFixed3x4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFixed3x4 &operator=( const domFixed3x4 &cpy ) { (void)cpy; return *this; }

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

	class domFixed4x1;

	typedef daeSmartRef<domFixed4x1> domFixed4x1Ref;
	typedef daeTArray<domFixed4x1Ref> domFixed4x1_Array;

	class domFixed4x1 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FIXED4X1; }

	protected:  // Value
		/**
		 * The domCg_fixed4x1 value of the text data of this element. 
		 */
		domCg_fixed4x1 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_fixed4x1 reference of the _value array.
		 */
		domCg_fixed4x1 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_fixed4x1 reference of the _value array.
		 */
		const domCg_fixed4x1 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_fixed4x1 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFixed4x1() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFixed4x1() {}
		/**
		 * Copy Constructor
		 */
		domFixed4x1( const domFixed4x1 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFixed4x1 &operator=( const domFixed4x1 &cpy ) { (void)cpy; return *this; }

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

	class domFixed4x2;

	typedef daeSmartRef<domFixed4x2> domFixed4x2Ref;
	typedef daeTArray<domFixed4x2Ref> domFixed4x2_Array;

	class domFixed4x2 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FIXED4X2; }

	protected:  // Value
		/**
		 * The domCg_fixed4x2 value of the text data of this element. 
		 */
		domCg_fixed4x2 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_fixed4x2 reference of the _value array.
		 */
		domCg_fixed4x2 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_fixed4x2 reference of the _value array.
		 */
		const domCg_fixed4x2 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_fixed4x2 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFixed4x2() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFixed4x2() {}
		/**
		 * Copy Constructor
		 */
		domFixed4x2( const domFixed4x2 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFixed4x2 &operator=( const domFixed4x2 &cpy ) { (void)cpy; return *this; }

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

	class domFixed4x3;

	typedef daeSmartRef<domFixed4x3> domFixed4x3Ref;
	typedef daeTArray<domFixed4x3Ref> domFixed4x3_Array;

	class domFixed4x3 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FIXED4X3; }

	protected:  // Value
		/**
		 * The domCg_fixed4x3 value of the text data of this element. 
		 */
		domCg_fixed4x3 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_fixed4x3 reference of the _value array.
		 */
		domCg_fixed4x3 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_fixed4x3 reference of the _value array.
		 */
		const domCg_fixed4x3 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_fixed4x3 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFixed4x3() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFixed4x3() {}
		/**
		 * Copy Constructor
		 */
		domFixed4x3( const domFixed4x3 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFixed4x3 &operator=( const domFixed4x3 &cpy ) { (void)cpy; return *this; }

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

	class domFixed4x4;

	typedef daeSmartRef<domFixed4x4> domFixed4x4Ref;
	typedef daeTArray<domFixed4x4Ref> domFixed4x4_Array;

	class domFixed4x4 : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FIXED4X4; }

	protected:  // Value
		/**
		 * The domCg_fixed4x4 value of the text data of this element. 
		 */
		domCg_fixed4x4 _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the _value array.
		 * @return Returns a domCg_fixed4x4 reference of the _value array.
		 */
		domCg_fixed4x4 &getValue() { return _value; }
		/**
		 * Gets the _value array.
		 * @return Returns a constant domCg_fixed4x4 reference of the _value array.
		 */
		const domCg_fixed4x4 &getValue() const { return _value; }
		/**
		 * Sets the _value array.
		 * @param val The new value for the _value array.
		 */
		void setValue( const domCg_fixed4x4 &val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domFixed4x4() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domFixed4x4() {}
		/**
		 * Copy Constructor
		 */
		domFixed4x4( const domFixed4x4 &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFixed4x4 &operator=( const domFixed4x4 &cpy ) { (void)cpy; return *this; }

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

	class domString;

	typedef daeSmartRef<domString> domStringRef;
	typedef daeTArray<domStringRef> domString_Array;

	class domString : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::STRING; }

	protected:  // Value
		/**
		 * The ::xsString value of the text data of this element. 
		 */
		::xsString _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return Returns a ::xsString of the value.
		 */
		::xsString getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( ::xsString val ) { *(daeStringRef*)&_value = val; }

	protected:
		/**
		 * Constructor
		 */
		domString() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domString() {}
		/**
		 * Copy Constructor
		 */
		domString( const domString &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domString &operator=( const domString &cpy ) { (void)cpy; return *this; }

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

	class domEnum;

	typedef daeSmartRef<domEnum> domEnumRef;
	typedef daeTArray<domEnumRef> domEnum_Array;

	class domEnum : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::ENUM; }

	protected:  // Value
		/**
		 * The domGl_enumeration value of the text data of this element. 
		 */
		domGl_enumeration _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return a domGl_enumeration of the value.
		 */
		domGl_enumeration getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( domGl_enumeration val ) { _value = val; }

	protected:
		/**
		 * Constructor
		 */
		domEnum() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domEnum() {}
		/**
		 * Copy Constructor
		 */
		domEnum( const domEnum &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domEnum &operator=( const domEnum &cpy ) { (void)cpy; return *this; }

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
	domBoolRef elemBool;
	domBool1Ref elemBool1;
	domBool2Ref elemBool2;
	domBool3Ref elemBool3;
	domBool4Ref elemBool4;
	domBool1x1Ref elemBool1x1;
	domBool1x2Ref elemBool1x2;
	domBool1x3Ref elemBool1x3;
	domBool1x4Ref elemBool1x4;
	domBool2x1Ref elemBool2x1;
	domBool2x2Ref elemBool2x2;
	domBool2x3Ref elemBool2x3;
	domBool2x4Ref elemBool2x4;
	domBool3x1Ref elemBool3x1;
	domBool3x2Ref elemBool3x2;
	domBool3x3Ref elemBool3x3;
	domBool3x4Ref elemBool3x4;
	domBool4x1Ref elemBool4x1;
	domBool4x2Ref elemBool4x2;
	domBool4x3Ref elemBool4x3;
	domBool4x4Ref elemBool4x4;
	domFloatRef elemFloat;
	domFloat1Ref elemFloat1;
	domFloat2Ref elemFloat2;
	domFloat3Ref elemFloat3;
	domFloat4Ref elemFloat4;
	domFloat1x1Ref elemFloat1x1;
	domFloat1x2Ref elemFloat1x2;
	domFloat1x3Ref elemFloat1x3;
	domFloat1x4Ref elemFloat1x4;
	domFloat2x1Ref elemFloat2x1;
	domFloat2x2Ref elemFloat2x2;
	domFloat2x3Ref elemFloat2x3;
	domFloat2x4Ref elemFloat2x4;
	domFloat3x1Ref elemFloat3x1;
	domFloat3x2Ref elemFloat3x2;
	domFloat3x3Ref elemFloat3x3;
	domFloat3x4Ref elemFloat3x4;
	domFloat4x1Ref elemFloat4x1;
	domFloat4x2Ref elemFloat4x2;
	domFloat4x3Ref elemFloat4x3;
	domFloat4x4Ref elemFloat4x4;
	domIntRef elemInt;
	domInt1Ref elemInt1;
	domInt2Ref elemInt2;
	domInt3Ref elemInt3;
	domInt4Ref elemInt4;
	domInt1x1Ref elemInt1x1;
	domInt1x2Ref elemInt1x2;
	domInt1x3Ref elemInt1x3;
	domInt1x4Ref elemInt1x4;
	domInt2x1Ref elemInt2x1;
	domInt2x2Ref elemInt2x2;
	domInt2x3Ref elemInt2x3;
	domInt2x4Ref elemInt2x4;
	domInt3x1Ref elemInt3x1;
	domInt3x2Ref elemInt3x2;
	domInt3x3Ref elemInt3x3;
	domInt3x4Ref elemInt3x4;
	domInt4x1Ref elemInt4x1;
	domInt4x2Ref elemInt4x2;
	domInt4x3Ref elemInt4x3;
	domInt4x4Ref elemInt4x4;
	domHalfRef elemHalf;
	domHalf1Ref elemHalf1;
	domHalf2Ref elemHalf2;
	domHalf3Ref elemHalf3;
	domHalf4Ref elemHalf4;
	domHalf1x1Ref elemHalf1x1;
	domHalf1x2Ref elemHalf1x2;
	domHalf1x3Ref elemHalf1x3;
	domHalf1x4Ref elemHalf1x4;
	domHalf2x1Ref elemHalf2x1;
	domHalf2x2Ref elemHalf2x2;
	domHalf2x3Ref elemHalf2x3;
	domHalf2x4Ref elemHalf2x4;
	domHalf3x1Ref elemHalf3x1;
	domHalf3x2Ref elemHalf3x2;
	domHalf3x3Ref elemHalf3x3;
	domHalf3x4Ref elemHalf3x4;
	domHalf4x1Ref elemHalf4x1;
	domHalf4x2Ref elemHalf4x2;
	domHalf4x3Ref elemHalf4x3;
	domHalf4x4Ref elemHalf4x4;
	domFixedRef elemFixed;
	domFixed1Ref elemFixed1;
	domFixed2Ref elemFixed2;
	domFixed3Ref elemFixed3;
	domFixed4Ref elemFixed4;
	domFixed1x1Ref elemFixed1x1;
	domFixed1x2Ref elemFixed1x2;
	domFixed1x3Ref elemFixed1x3;
	domFixed1x4Ref elemFixed1x4;
	domFixed2x1Ref elemFixed2x1;
	domFixed2x2Ref elemFixed2x2;
	domFixed2x3Ref elemFixed2x3;
	domFixed2x4Ref elemFixed2x4;
	domFixed3x1Ref elemFixed3x1;
	domFixed3x2Ref elemFixed3x2;
	domFixed3x3Ref elemFixed3x3;
	domFixed3x4Ref elemFixed3x4;
	domFixed4x1Ref elemFixed4x1;
	domFixed4x2Ref elemFixed4x2;
	domFixed4x3Ref elemFixed4x3;
	domFixed4x4Ref elemFixed4x4;
	domCg_surface_typeRef elemSurface;
	domCg_sampler1DRef elemSampler1D;
	domCg_sampler2DRef elemSampler2D;
	domCg_sampler3DRef elemSampler3D;
	domCg_samplerRECTRef elemSamplerRECT;
	domCg_samplerCUBERef elemSamplerCUBE;
	domCg_samplerDEPTHRef elemSamplerDEPTH;
	domStringRef elemString;
	domEnumRef elemEnum;
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
	 * Gets the bool element.
	 * @return a daeSmartRef to the bool element.
	 */
	const domBoolRef getBool() const { return elemBool; }
	/**
	 * Gets the bool1 element.
	 * @return a daeSmartRef to the bool1 element.
	 */
	const domBool1Ref getBool1() const { return elemBool1; }
	/**
	 * Gets the bool2 element.
	 * @return a daeSmartRef to the bool2 element.
	 */
	const domBool2Ref getBool2() const { return elemBool2; }
	/**
	 * Gets the bool3 element.
	 * @return a daeSmartRef to the bool3 element.
	 */
	const domBool3Ref getBool3() const { return elemBool3; }
	/**
	 * Gets the bool4 element.
	 * @return a daeSmartRef to the bool4 element.
	 */
	const domBool4Ref getBool4() const { return elemBool4; }
	/**
	 * Gets the bool1x1 element.
	 * @return a daeSmartRef to the bool1x1 element.
	 */
	const domBool1x1Ref getBool1x1() const { return elemBool1x1; }
	/**
	 * Gets the bool1x2 element.
	 * @return a daeSmartRef to the bool1x2 element.
	 */
	const domBool1x2Ref getBool1x2() const { return elemBool1x2; }
	/**
	 * Gets the bool1x3 element.
	 * @return a daeSmartRef to the bool1x3 element.
	 */
	const domBool1x3Ref getBool1x3() const { return elemBool1x3; }
	/**
	 * Gets the bool1x4 element.
	 * @return a daeSmartRef to the bool1x4 element.
	 */
	const domBool1x4Ref getBool1x4() const { return elemBool1x4; }
	/**
	 * Gets the bool2x1 element.
	 * @return a daeSmartRef to the bool2x1 element.
	 */
	const domBool2x1Ref getBool2x1() const { return elemBool2x1; }
	/**
	 * Gets the bool2x2 element.
	 * @return a daeSmartRef to the bool2x2 element.
	 */
	const domBool2x2Ref getBool2x2() const { return elemBool2x2; }
	/**
	 * Gets the bool2x3 element.
	 * @return a daeSmartRef to the bool2x3 element.
	 */
	const domBool2x3Ref getBool2x3() const { return elemBool2x3; }
	/**
	 * Gets the bool2x4 element.
	 * @return a daeSmartRef to the bool2x4 element.
	 */
	const domBool2x4Ref getBool2x4() const { return elemBool2x4; }
	/**
	 * Gets the bool3x1 element.
	 * @return a daeSmartRef to the bool3x1 element.
	 */
	const domBool3x1Ref getBool3x1() const { return elemBool3x1; }
	/**
	 * Gets the bool3x2 element.
	 * @return a daeSmartRef to the bool3x2 element.
	 */
	const domBool3x2Ref getBool3x2() const { return elemBool3x2; }
	/**
	 * Gets the bool3x3 element.
	 * @return a daeSmartRef to the bool3x3 element.
	 */
	const domBool3x3Ref getBool3x3() const { return elemBool3x3; }
	/**
	 * Gets the bool3x4 element.
	 * @return a daeSmartRef to the bool3x4 element.
	 */
	const domBool3x4Ref getBool3x4() const { return elemBool3x4; }
	/**
	 * Gets the bool4x1 element.
	 * @return a daeSmartRef to the bool4x1 element.
	 */
	const domBool4x1Ref getBool4x1() const { return elemBool4x1; }
	/**
	 * Gets the bool4x2 element.
	 * @return a daeSmartRef to the bool4x2 element.
	 */
	const domBool4x2Ref getBool4x2() const { return elemBool4x2; }
	/**
	 * Gets the bool4x3 element.
	 * @return a daeSmartRef to the bool4x3 element.
	 */
	const domBool4x3Ref getBool4x3() const { return elemBool4x3; }
	/**
	 * Gets the bool4x4 element.
	 * @return a daeSmartRef to the bool4x4 element.
	 */
	const domBool4x4Ref getBool4x4() const { return elemBool4x4; }
	/**
	 * Gets the float element.
	 * @return a daeSmartRef to the float element.
	 */
	const domFloatRef getFloat() const { return elemFloat; }
	/**
	 * Gets the float1 element.
	 * @return a daeSmartRef to the float1 element.
	 */
	const domFloat1Ref getFloat1() const { return elemFloat1; }
	/**
	 * Gets the float2 element.
	 * @return a daeSmartRef to the float2 element.
	 */
	const domFloat2Ref getFloat2() const { return elemFloat2; }
	/**
	 * Gets the float3 element.
	 * @return a daeSmartRef to the float3 element.
	 */
	const domFloat3Ref getFloat3() const { return elemFloat3; }
	/**
	 * Gets the float4 element.
	 * @return a daeSmartRef to the float4 element.
	 */
	const domFloat4Ref getFloat4() const { return elemFloat4; }
	/**
	 * Gets the float1x1 element.
	 * @return a daeSmartRef to the float1x1 element.
	 */
	const domFloat1x1Ref getFloat1x1() const { return elemFloat1x1; }
	/**
	 * Gets the float1x2 element.
	 * @return a daeSmartRef to the float1x2 element.
	 */
	const domFloat1x2Ref getFloat1x2() const { return elemFloat1x2; }
	/**
	 * Gets the float1x3 element.
	 * @return a daeSmartRef to the float1x3 element.
	 */
	const domFloat1x3Ref getFloat1x3() const { return elemFloat1x3; }
	/**
	 * Gets the float1x4 element.
	 * @return a daeSmartRef to the float1x4 element.
	 */
	const domFloat1x4Ref getFloat1x4() const { return elemFloat1x4; }
	/**
	 * Gets the float2x1 element.
	 * @return a daeSmartRef to the float2x1 element.
	 */
	const domFloat2x1Ref getFloat2x1() const { return elemFloat2x1; }
	/**
	 * Gets the float2x2 element.
	 * @return a daeSmartRef to the float2x2 element.
	 */
	const domFloat2x2Ref getFloat2x2() const { return elemFloat2x2; }
	/**
	 * Gets the float2x3 element.
	 * @return a daeSmartRef to the float2x3 element.
	 */
	const domFloat2x3Ref getFloat2x3() const { return elemFloat2x3; }
	/**
	 * Gets the float2x4 element.
	 * @return a daeSmartRef to the float2x4 element.
	 */
	const domFloat2x4Ref getFloat2x4() const { return elemFloat2x4; }
	/**
	 * Gets the float3x1 element.
	 * @return a daeSmartRef to the float3x1 element.
	 */
	const domFloat3x1Ref getFloat3x1() const { return elemFloat3x1; }
	/**
	 * Gets the float3x2 element.
	 * @return a daeSmartRef to the float3x2 element.
	 */
	const domFloat3x2Ref getFloat3x2() const { return elemFloat3x2; }
	/**
	 * Gets the float3x3 element.
	 * @return a daeSmartRef to the float3x3 element.
	 */
	const domFloat3x3Ref getFloat3x3() const { return elemFloat3x3; }
	/**
	 * Gets the float3x4 element.
	 * @return a daeSmartRef to the float3x4 element.
	 */
	const domFloat3x4Ref getFloat3x4() const { return elemFloat3x4; }
	/**
	 * Gets the float4x1 element.
	 * @return a daeSmartRef to the float4x1 element.
	 */
	const domFloat4x1Ref getFloat4x1() const { return elemFloat4x1; }
	/**
	 * Gets the float4x2 element.
	 * @return a daeSmartRef to the float4x2 element.
	 */
	const domFloat4x2Ref getFloat4x2() const { return elemFloat4x2; }
	/**
	 * Gets the float4x3 element.
	 * @return a daeSmartRef to the float4x3 element.
	 */
	const domFloat4x3Ref getFloat4x3() const { return elemFloat4x3; }
	/**
	 * Gets the float4x4 element.
	 * @return a daeSmartRef to the float4x4 element.
	 */
	const domFloat4x4Ref getFloat4x4() const { return elemFloat4x4; }
	/**
	 * Gets the int element.
	 * @return a daeSmartRef to the int element.
	 */
	const domIntRef getInt() const { return elemInt; }
	/**
	 * Gets the int1 element.
	 * @return a daeSmartRef to the int1 element.
	 */
	const domInt1Ref getInt1() const { return elemInt1; }
	/**
	 * Gets the int2 element.
	 * @return a daeSmartRef to the int2 element.
	 */
	const domInt2Ref getInt2() const { return elemInt2; }
	/**
	 * Gets the int3 element.
	 * @return a daeSmartRef to the int3 element.
	 */
	const domInt3Ref getInt3() const { return elemInt3; }
	/**
	 * Gets the int4 element.
	 * @return a daeSmartRef to the int4 element.
	 */
	const domInt4Ref getInt4() const { return elemInt4; }
	/**
	 * Gets the int1x1 element.
	 * @return a daeSmartRef to the int1x1 element.
	 */
	const domInt1x1Ref getInt1x1() const { return elemInt1x1; }
	/**
	 * Gets the int1x2 element.
	 * @return a daeSmartRef to the int1x2 element.
	 */
	const domInt1x2Ref getInt1x2() const { return elemInt1x2; }
	/**
	 * Gets the int1x3 element.
	 * @return a daeSmartRef to the int1x3 element.
	 */
	const domInt1x3Ref getInt1x3() const { return elemInt1x3; }
	/**
	 * Gets the int1x4 element.
	 * @return a daeSmartRef to the int1x4 element.
	 */
	const domInt1x4Ref getInt1x4() const { return elemInt1x4; }
	/**
	 * Gets the int2x1 element.
	 * @return a daeSmartRef to the int2x1 element.
	 */
	const domInt2x1Ref getInt2x1() const { return elemInt2x1; }
	/**
	 * Gets the int2x2 element.
	 * @return a daeSmartRef to the int2x2 element.
	 */
	const domInt2x2Ref getInt2x2() const { return elemInt2x2; }
	/**
	 * Gets the int2x3 element.
	 * @return a daeSmartRef to the int2x3 element.
	 */
	const domInt2x3Ref getInt2x3() const { return elemInt2x3; }
	/**
	 * Gets the int2x4 element.
	 * @return a daeSmartRef to the int2x4 element.
	 */
	const domInt2x4Ref getInt2x4() const { return elemInt2x4; }
	/**
	 * Gets the int3x1 element.
	 * @return a daeSmartRef to the int3x1 element.
	 */
	const domInt3x1Ref getInt3x1() const { return elemInt3x1; }
	/**
	 * Gets the int3x2 element.
	 * @return a daeSmartRef to the int3x2 element.
	 */
	const domInt3x2Ref getInt3x2() const { return elemInt3x2; }
	/**
	 * Gets the int3x3 element.
	 * @return a daeSmartRef to the int3x3 element.
	 */
	const domInt3x3Ref getInt3x3() const { return elemInt3x3; }
	/**
	 * Gets the int3x4 element.
	 * @return a daeSmartRef to the int3x4 element.
	 */
	const domInt3x4Ref getInt3x4() const { return elemInt3x4; }
	/**
	 * Gets the int4x1 element.
	 * @return a daeSmartRef to the int4x1 element.
	 */
	const domInt4x1Ref getInt4x1() const { return elemInt4x1; }
	/**
	 * Gets the int4x2 element.
	 * @return a daeSmartRef to the int4x2 element.
	 */
	const domInt4x2Ref getInt4x2() const { return elemInt4x2; }
	/**
	 * Gets the int4x3 element.
	 * @return a daeSmartRef to the int4x3 element.
	 */
	const domInt4x3Ref getInt4x3() const { return elemInt4x3; }
	/**
	 * Gets the int4x4 element.
	 * @return a daeSmartRef to the int4x4 element.
	 */
	const domInt4x4Ref getInt4x4() const { return elemInt4x4; }
	/**
	 * Gets the half element.
	 * @return a daeSmartRef to the half element.
	 */
	const domHalfRef getHalf() const { return elemHalf; }
	/**
	 * Gets the half1 element.
	 * @return a daeSmartRef to the half1 element.
	 */
	const domHalf1Ref getHalf1() const { return elemHalf1; }
	/**
	 * Gets the half2 element.
	 * @return a daeSmartRef to the half2 element.
	 */
	const domHalf2Ref getHalf2() const { return elemHalf2; }
	/**
	 * Gets the half3 element.
	 * @return a daeSmartRef to the half3 element.
	 */
	const domHalf3Ref getHalf3() const { return elemHalf3; }
	/**
	 * Gets the half4 element.
	 * @return a daeSmartRef to the half4 element.
	 */
	const domHalf4Ref getHalf4() const { return elemHalf4; }
	/**
	 * Gets the half1x1 element.
	 * @return a daeSmartRef to the half1x1 element.
	 */
	const domHalf1x1Ref getHalf1x1() const { return elemHalf1x1; }
	/**
	 * Gets the half1x2 element.
	 * @return a daeSmartRef to the half1x2 element.
	 */
	const domHalf1x2Ref getHalf1x2() const { return elemHalf1x2; }
	/**
	 * Gets the half1x3 element.
	 * @return a daeSmartRef to the half1x3 element.
	 */
	const domHalf1x3Ref getHalf1x3() const { return elemHalf1x3; }
	/**
	 * Gets the half1x4 element.
	 * @return a daeSmartRef to the half1x4 element.
	 */
	const domHalf1x4Ref getHalf1x4() const { return elemHalf1x4; }
	/**
	 * Gets the half2x1 element.
	 * @return a daeSmartRef to the half2x1 element.
	 */
	const domHalf2x1Ref getHalf2x1() const { return elemHalf2x1; }
	/**
	 * Gets the half2x2 element.
	 * @return a daeSmartRef to the half2x2 element.
	 */
	const domHalf2x2Ref getHalf2x2() const { return elemHalf2x2; }
	/**
	 * Gets the half2x3 element.
	 * @return a daeSmartRef to the half2x3 element.
	 */
	const domHalf2x3Ref getHalf2x3() const { return elemHalf2x3; }
	/**
	 * Gets the half2x4 element.
	 * @return a daeSmartRef to the half2x4 element.
	 */
	const domHalf2x4Ref getHalf2x4() const { return elemHalf2x4; }
	/**
	 * Gets the half3x1 element.
	 * @return a daeSmartRef to the half3x1 element.
	 */
	const domHalf3x1Ref getHalf3x1() const { return elemHalf3x1; }
	/**
	 * Gets the half3x2 element.
	 * @return a daeSmartRef to the half3x2 element.
	 */
	const domHalf3x2Ref getHalf3x2() const { return elemHalf3x2; }
	/**
	 * Gets the half3x3 element.
	 * @return a daeSmartRef to the half3x3 element.
	 */
	const domHalf3x3Ref getHalf3x3() const { return elemHalf3x3; }
	/**
	 * Gets the half3x4 element.
	 * @return a daeSmartRef to the half3x4 element.
	 */
	const domHalf3x4Ref getHalf3x4() const { return elemHalf3x4; }
	/**
	 * Gets the half4x1 element.
	 * @return a daeSmartRef to the half4x1 element.
	 */
	const domHalf4x1Ref getHalf4x1() const { return elemHalf4x1; }
	/**
	 * Gets the half4x2 element.
	 * @return a daeSmartRef to the half4x2 element.
	 */
	const domHalf4x2Ref getHalf4x2() const { return elemHalf4x2; }
	/**
	 * Gets the half4x3 element.
	 * @return a daeSmartRef to the half4x3 element.
	 */
	const domHalf4x3Ref getHalf4x3() const { return elemHalf4x3; }
	/**
	 * Gets the half4x4 element.
	 * @return a daeSmartRef to the half4x4 element.
	 */
	const domHalf4x4Ref getHalf4x4() const { return elemHalf4x4; }
	/**
	 * Gets the fixed element.
	 * @return a daeSmartRef to the fixed element.
	 */
	const domFixedRef getFixed() const { return elemFixed; }
	/**
	 * Gets the fixed1 element.
	 * @return a daeSmartRef to the fixed1 element.
	 */
	const domFixed1Ref getFixed1() const { return elemFixed1; }
	/**
	 * Gets the fixed2 element.
	 * @return a daeSmartRef to the fixed2 element.
	 */
	const domFixed2Ref getFixed2() const { return elemFixed2; }
	/**
	 * Gets the fixed3 element.
	 * @return a daeSmartRef to the fixed3 element.
	 */
	const domFixed3Ref getFixed3() const { return elemFixed3; }
	/**
	 * Gets the fixed4 element.
	 * @return a daeSmartRef to the fixed4 element.
	 */
	const domFixed4Ref getFixed4() const { return elemFixed4; }
	/**
	 * Gets the fixed1x1 element.
	 * @return a daeSmartRef to the fixed1x1 element.
	 */
	const domFixed1x1Ref getFixed1x1() const { return elemFixed1x1; }
	/**
	 * Gets the fixed1x2 element.
	 * @return a daeSmartRef to the fixed1x2 element.
	 */
	const domFixed1x2Ref getFixed1x2() const { return elemFixed1x2; }
	/**
	 * Gets the fixed1x3 element.
	 * @return a daeSmartRef to the fixed1x3 element.
	 */
	const domFixed1x3Ref getFixed1x3() const { return elemFixed1x3; }
	/**
	 * Gets the fixed1x4 element.
	 * @return a daeSmartRef to the fixed1x4 element.
	 */
	const domFixed1x4Ref getFixed1x4() const { return elemFixed1x4; }
	/**
	 * Gets the fixed2x1 element.
	 * @return a daeSmartRef to the fixed2x1 element.
	 */
	const domFixed2x1Ref getFixed2x1() const { return elemFixed2x1; }
	/**
	 * Gets the fixed2x2 element.
	 * @return a daeSmartRef to the fixed2x2 element.
	 */
	const domFixed2x2Ref getFixed2x2() const { return elemFixed2x2; }
	/**
	 * Gets the fixed2x3 element.
	 * @return a daeSmartRef to the fixed2x3 element.
	 */
	const domFixed2x3Ref getFixed2x3() const { return elemFixed2x3; }
	/**
	 * Gets the fixed2x4 element.
	 * @return a daeSmartRef to the fixed2x4 element.
	 */
	const domFixed2x4Ref getFixed2x4() const { return elemFixed2x4; }
	/**
	 * Gets the fixed3x1 element.
	 * @return a daeSmartRef to the fixed3x1 element.
	 */
	const domFixed3x1Ref getFixed3x1() const { return elemFixed3x1; }
	/**
	 * Gets the fixed3x2 element.
	 * @return a daeSmartRef to the fixed3x2 element.
	 */
	const domFixed3x2Ref getFixed3x2() const { return elemFixed3x2; }
	/**
	 * Gets the fixed3x3 element.
	 * @return a daeSmartRef to the fixed3x3 element.
	 */
	const domFixed3x3Ref getFixed3x3() const { return elemFixed3x3; }
	/**
	 * Gets the fixed3x4 element.
	 * @return a daeSmartRef to the fixed3x4 element.
	 */
	const domFixed3x4Ref getFixed3x4() const { return elemFixed3x4; }
	/**
	 * Gets the fixed4x1 element.
	 * @return a daeSmartRef to the fixed4x1 element.
	 */
	const domFixed4x1Ref getFixed4x1() const { return elemFixed4x1; }
	/**
	 * Gets the fixed4x2 element.
	 * @return a daeSmartRef to the fixed4x2 element.
	 */
	const domFixed4x2Ref getFixed4x2() const { return elemFixed4x2; }
	/**
	 * Gets the fixed4x3 element.
	 * @return a daeSmartRef to the fixed4x3 element.
	 */
	const domFixed4x3Ref getFixed4x3() const { return elemFixed4x3; }
	/**
	 * Gets the fixed4x4 element.
	 * @return a daeSmartRef to the fixed4x4 element.
	 */
	const domFixed4x4Ref getFixed4x4() const { return elemFixed4x4; }
	/**
	 * Gets the surface element.
	 * @return a daeSmartRef to the surface element.
	 */
	const domCg_surface_typeRef getSurface() const { return elemSurface; }
	/**
	 * Gets the sampler1D element.
	 * @return a daeSmartRef to the sampler1D element.
	 */
	const domCg_sampler1DRef getSampler1D() const { return elemSampler1D; }
	/**
	 * Gets the sampler2D element.
	 * @return a daeSmartRef to the sampler2D element.
	 */
	const domCg_sampler2DRef getSampler2D() const { return elemSampler2D; }
	/**
	 * Gets the sampler3D element.
	 * @return a daeSmartRef to the sampler3D element.
	 */
	const domCg_sampler3DRef getSampler3D() const { return elemSampler3D; }
	/**
	 * Gets the samplerRECT element.
	 * @return a daeSmartRef to the samplerRECT element.
	 */
	const domCg_samplerRECTRef getSamplerRECT() const { return elemSamplerRECT; }
	/**
	 * Gets the samplerCUBE element.
	 * @return a daeSmartRef to the samplerCUBE element.
	 */
	const domCg_samplerCUBERef getSamplerCUBE() const { return elemSamplerCUBE; }
	/**
	 * Gets the samplerDEPTH element.
	 * @return a daeSmartRef to the samplerDEPTH element.
	 */
	const domCg_samplerDEPTHRef getSamplerDEPTH() const { return elemSamplerDEPTH; }
	/**
	 * Gets the string element.
	 * @return a daeSmartRef to the string element.
	 */
	const domStringRef getString() const { return elemString; }
	/**
	 * Gets the enum element.
	 * @return a daeSmartRef to the enum element.
	 */
	const domEnumRef getEnum() const { return elemEnum; }
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
	domCg_param_type() : elemBool(), elemBool1(), elemBool2(), elemBool3(), elemBool4(), elemBool1x1(), elemBool1x2(), elemBool1x3(), elemBool1x4(), elemBool2x1(), elemBool2x2(), elemBool2x3(), elemBool2x4(), elemBool3x1(), elemBool3x2(), elemBool3x3(), elemBool3x4(), elemBool4x1(), elemBool4x2(), elemBool4x3(), elemBool4x4(), elemFloat(), elemFloat1(), elemFloat2(), elemFloat3(), elemFloat4(), elemFloat1x1(), elemFloat1x2(), elemFloat1x3(), elemFloat1x4(), elemFloat2x1(), elemFloat2x2(), elemFloat2x3(), elemFloat2x4(), elemFloat3x1(), elemFloat3x2(), elemFloat3x3(), elemFloat3x4(), elemFloat4x1(), elemFloat4x2(), elemFloat4x3(), elemFloat4x4(), elemInt(), elemInt1(), elemInt2(), elemInt3(), elemInt4(), elemInt1x1(), elemInt1x2(), elemInt1x3(), elemInt1x4(), elemInt2x1(), elemInt2x2(), elemInt2x3(), elemInt2x4(), elemInt3x1(), elemInt3x2(), elemInt3x3(), elemInt3x4(), elemInt4x1(), elemInt4x2(), elemInt4x3(), elemInt4x4(), elemHalf(), elemHalf1(), elemHalf2(), elemHalf3(), elemHalf4(), elemHalf1x1(), elemHalf1x2(), elemHalf1x3(), elemHalf1x4(), elemHalf2x1(), elemHalf2x2(), elemHalf2x3(), elemHalf2x4(), elemHalf3x1(), elemHalf3x2(), elemHalf3x3(), elemHalf3x4(), elemHalf4x1(), elemHalf4x2(), elemHalf4x3(), elemHalf4x4(), elemFixed(), elemFixed1(), elemFixed2(), elemFixed3(), elemFixed4(), elemFixed1x1(), elemFixed1x2(), elemFixed1x3(), elemFixed1x4(), elemFixed2x1(), elemFixed2x2(), elemFixed2x3(), elemFixed2x4(), elemFixed3x1(), elemFixed3x2(), elemFixed3x3(), elemFixed3x4(), elemFixed4x1(), elemFixed4x2(), elemFixed4x3(), elemFixed4x4(), elemSurface(), elemSampler1D(), elemSampler2D(), elemSampler3D(), elemSamplerRECT(), elemSamplerCUBE(), elemSamplerDEPTH(), elemString(), elemEnum() {}
	/**
	 * Destructor
	 */
	virtual ~domCg_param_type() {}
	/**
	 * Copy Constructor
	 */
	domCg_param_type( const domCg_param_type &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domCg_param_type &operator=( const domCg_param_type &cpy ) { (void)cpy; return *this; }

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
