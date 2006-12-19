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
#ifndef __domFx_surface_init_planar_common_h__
#define __domFx_surface_init_planar_common_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>


/**
 * For 1D, 2D, RECT surface types
 */
class domFx_surface_init_planar_common_complexType 
{
public:
	class domAll;

	typedef daeSmartRef<domAll> domAllRef;
	typedef daeTArray<domAllRef> domAll_Array;

/**
 * Init the entire surface with one compound image such as DDS
 */
	class domAll : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::ALL; }
	protected:  // Attribute
		xsIDREF attrRef;


	public:	//Accessors and Mutators
		/**
		 * Gets the ref attribute.
		 * @return Returns a xsIDREF reference of the ref attribute.
		 */
		xsIDREF &getRef() { return attrRef; }
		/**
		 * Gets the ref attribute.
		 * @return Returns a constant xsIDREF reference of the ref attribute.
		 */
		const xsIDREF &getRef() const{ return attrRef; }
		/**
		 * Sets the ref attribute.
		 * @param atRef The new value for the ref attribute.
		 */
		void setRef( const xsIDREF &atRef ) { attrRef = atRef;	
	 _validAttributeArray[0] = true; }

	protected:
		/**
		 * Constructor
		 */
		domAll() : attrRef() {}
		/**
		 * Destructor
		 */
		virtual ~domAll() {}
		/**
		 * Copy Constructor
		 */
		domAll( const domAll &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domAll &operator=( const domAll &cpy ) { (void)cpy; return *this; }

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



protected:  // Element
/**
 * Init the entire surface with one compound image such as DDS @see domAll
 */
	domAllRef elemAll;

public:	//Accessors and Mutators
	/**
	 * Gets the all element.
	 * @return a daeSmartRef to the all element.
	 */
	const domAllRef getAll() const { return elemAll; }
protected:
	/**
	 * Constructor
	 */
	domFx_surface_init_planar_common_complexType() : elemAll() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_surface_init_planar_common_complexType() {}
	/**
	 * Copy Constructor
	 */
	domFx_surface_init_planar_common_complexType( const domFx_surface_init_planar_common_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_surface_init_planar_common_complexType &operator=( const domFx_surface_init_planar_common_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domFx_surface_init_planar_common_complexType.
 */
class domFx_surface_init_planar_common : public daeElement, public domFx_surface_init_planar_common_complexType
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FX_SURFACE_INIT_PLANAR_COMMON; }
protected:
	/**
	 * Constructor
	 */
	domFx_surface_init_planar_common() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_surface_init_planar_common() {}
	/**
	 * Copy Constructor
	 */
	domFx_surface_init_planar_common( const domFx_surface_init_planar_common &cpy ) : daeElement(), domFx_surface_init_planar_common_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_surface_init_planar_common &operator=( const domFx_surface_init_planar_common &cpy ) { (void)cpy; return *this; }

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
