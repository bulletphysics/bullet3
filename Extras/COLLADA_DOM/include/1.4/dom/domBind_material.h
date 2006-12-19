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
#ifndef __domBind_material_h__
#define __domBind_material_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domParam.h>
#include <dom/domTechnique.h>
#include <dom/domExtra.h>
#include <dom/domInstance_material.h>

/**
 * Bind a specific material to a piece of geometry, binding varying and uniform
 * parameters at the  same time.
 */
class domBind_material : public daeElement
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BIND_MATERIAL; }
public:
	class domTechnique_common;

	typedef daeSmartRef<domTechnique_common> domTechnique_commonRef;
	typedef daeTArray<domTechnique_commonRef> domTechnique_common_Array;

/**
 * The technique_common element specifies the bind_material information for
 * the common  profile which all COLLADA implementations need to support.
 */
	class domTechnique_common : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::TECHNIQUE_COMMON; }

	protected:  // Element
/**
 *  The instance_material element specifies the information needed to bind
 * a geometry to a material. This element must appear at least once.  @see
 * domInstance_material
 */
		domInstance_material_Array elemInstance_material_array;

	public:	//Accessors and Mutators
		/**
		 * Gets the instance_material element array.
		 * @return Returns a reference to the array of instance_material elements.
		 */
		domInstance_material_Array &getInstance_material_array() { return elemInstance_material_array; }
		/**
		 * Gets the instance_material element array.
		 * @return Returns a constant reference to the array of instance_material elements.
		 */
		const domInstance_material_Array &getInstance_material_array() const { return elemInstance_material_array; }
	protected:
		/**
		 * Constructor
		 */
		domTechnique_common() : elemInstance_material_array() {}
		/**
		 * Destructor
		 */
		virtual ~domTechnique_common() {}
		/**
		 * Copy Constructor
		 */
		domTechnique_common( const domTechnique_common &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domTechnique_common &operator=( const domTechnique_common &cpy ) { (void)cpy; return *this; }

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
 *  The bind_material element may contain any number of param elements.  @see
 * domParam
 */
	domParam_Array elemParam_array;
/**
 * The technique_common element specifies the bind_material information for
 * the common  profile which all COLLADA implementations need to support.
 * @see domTechnique_common
 */
	domTechnique_commonRef elemTechnique_common;
/**
 *  This element may contain any number of non-common profile techniques.
 * @see domTechnique
 */
	domTechnique_Array elemTechnique_array;
/**
 *  The extra element may appear any number of times.  @see domExtra
 */
	domExtra_Array elemExtra_array;

public:	//Accessors and Mutators
	/**
	 * Gets the param element array.
	 * @return Returns a reference to the array of param elements.
	 */
	domParam_Array &getParam_array() { return elemParam_array; }
	/**
	 * Gets the param element array.
	 * @return Returns a constant reference to the array of param elements.
	 */
	const domParam_Array &getParam_array() const { return elemParam_array; }
	/**
	 * Gets the technique_common element.
	 * @return a daeSmartRef to the technique_common element.
	 */
	const domTechnique_commonRef getTechnique_common() const { return elemTechnique_common; }
	/**
	 * Gets the technique element array.
	 * @return Returns a reference to the array of technique elements.
	 */
	domTechnique_Array &getTechnique_array() { return elemTechnique_array; }
	/**
	 * Gets the technique element array.
	 * @return Returns a constant reference to the array of technique elements.
	 */
	const domTechnique_Array &getTechnique_array() const { return elemTechnique_array; }
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
	domBind_material() : elemParam_array(), elemTechnique_common(), elemTechnique_array(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domBind_material() {}
	/**
	 * Copy Constructor
	 */
	domBind_material( const domBind_material &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domBind_material &operator=( const domBind_material &cpy ) { (void)cpy; return *this; }

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
