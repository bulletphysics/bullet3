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
#ifndef __domFx_surface_init_volume_common_h__
#define __domFx_surface_init_volume_common_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>


class domFx_surface_init_volume_common_complexType 
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
		void setRef( const xsIDREF &atRef ) { attrRef.setID( atRef.getID() );	
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

	class domPrimary;

	typedef daeSmartRef<domPrimary> domPrimaryRef;
	typedef daeTArray<domPrimaryRef> domPrimary_Array;

/**
 * Init mip level 0 of the surface with one compound image such as DDS.  Use
 * of this element expects that the surface has element mip_levels=0 or mipmap_generate.
 */
	class domPrimary : public daeElement
	{
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
		void setRef( const xsIDREF &atRef ) { attrRef.setID( atRef.getID() );	
	 _validAttributeArray[0] = true; }

	protected:
		/**
		 * Constructor
		 */
		domPrimary() : attrRef() {}
		/**
		 * Destructor
		 */
		virtual ~domPrimary() {}
		/**
		 * Copy Constructor
		 */
		domPrimary( const domPrimary &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domPrimary &operator=( const domPrimary &cpy ) { (void)cpy; return *this; }

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
 * Init the entire surface with one compound image such as DDS @see domAll
 */
	domAllRef elemAll;
/**
 * Init mip level 0 of the surface with one compound image such as DDS.  Use
 * of this element expects that the surface has element mip_levels=0 or mipmap_generate.
 * @see domPrimary
 */
	domPrimaryRef elemPrimary;
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
	 * Gets the all element.
	 * @return a daeSmartRef to the all element.
	 */
	const domAllRef getAll() const { return elemAll; }
	/**
	 * Gets the primary element.
	 * @return a daeSmartRef to the primary element.
	 */
	const domPrimaryRef getPrimary() const { return elemPrimary; }
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
	domFx_surface_init_volume_common_complexType() : elemAll(), elemPrimary() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_surface_init_volume_common_complexType() {}
	/**
	 * Copy Constructor
	 */
	domFx_surface_init_volume_common_complexType( const domFx_surface_init_volume_common_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_surface_init_volume_common_complexType &operator=( const domFx_surface_init_volume_common_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domFx_surface_init_volume_common_complexType.
 */
class domFx_surface_init_volume_common : public daeElement, public domFx_surface_init_volume_common_complexType
{
protected:
	/**
	 * Constructor
	 */
	domFx_surface_init_volume_common() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_surface_init_volume_common() {}
	/**
	 * Copy Constructor
	 */
	domFx_surface_init_volume_common( const domFx_surface_init_volume_common &cpy ) : daeElement(), domFx_surface_init_volume_common_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_surface_init_volume_common &operator=( const domFx_surface_init_volume_common &cpy ) { (void)cpy; return *this; }

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
