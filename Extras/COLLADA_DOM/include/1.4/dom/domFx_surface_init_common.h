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
#ifndef __domFx_surface_init_common_h__
#define __domFx_surface_init_common_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domFx_surface_init_cube_common.h>
#include <dom/domFx_surface_init_volume_common.h>
#include <dom/domFx_surface_init_planar_common.h>
#include <dom/domFx_surface_init_from_common.h>

/**
 * The common set of initalization options for surfaces.  Choose which is
 * appropriate for your surface based on type and other characteristics. described
 * by the annotation docs on the child elements.
 */
class domFx_surface_init_common : public daeElement
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FX_SURFACE_INIT_COMMON; }
public:
	class domInit_as_null;

	typedef daeSmartRef<domInit_as_null> domInit_as_nullRef;
	typedef daeTArray<domInit_as_nullRef> domInit_as_null_Array;

/**
 * This surface is intended to be initialized later externally by a "setparam"
 * element.  If it is used before being initialized there is profile and platform
 * specific behavior.  Most elements on the surface element containing this
 * will be ignored including mip_levels, mipmap_generate, size, viewport_ratio,
 * and format.
 */
	class domInit_as_null : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INIT_AS_NULL; }

	protected:
		/**
		 * Constructor
		 */
		domInit_as_null() {}
		/**
		 * Destructor
		 */
		virtual ~domInit_as_null() {}
		/**
		 * Copy Constructor
		 */
		domInit_as_null( const domInit_as_null &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInit_as_null &operator=( const domInit_as_null &cpy ) { (void)cpy; return *this; }

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

	class domInit_as_target;

	typedef daeSmartRef<domInit_as_target> domInit_as_targetRef;
	typedef daeTArray<domInit_as_targetRef> domInit_as_target_Array;

/**
 * Init as a target for depth, stencil, or color.  It does not need image
 * data. Surface should not have mipmap_generate when using this.
 */
	class domInit_as_target : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INIT_AS_TARGET; }

	protected:
		/**
		 * Constructor
		 */
		domInit_as_target() {}
		/**
		 * Destructor
		 */
		virtual ~domInit_as_target() {}
		/**
		 * Copy Constructor
		 */
		domInit_as_target( const domInit_as_target &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domInit_as_target &operator=( const domInit_as_target &cpy ) { (void)cpy; return *this; }

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
 * This surface is intended to be initialized later externally by a "setparam"
 * element.  If it is used before being initialized there is profile and platform
 * specific behavior.  Most elements on the surface element containing this
 * will be ignored including mip_levels, mipmap_generate, size, viewport_ratio,
 * and format. @see domInit_as_null
 */
	domInit_as_nullRef elemInit_as_null;
/**
 * Init as a target for depth, stencil, or color.  It does not need image
 * data. Surface should not have mipmap_generate when using this. @see domInit_as_target
 */
	domInit_as_targetRef elemInit_as_target;
/**
 * Init a CUBE from a compound image such as DDS @see domInit_cube
 */
	domFx_surface_init_cube_commonRef elemInit_cube;
/**
 * Init a 3D from a compound image such as DDS @see domInit_volume
 */
	domFx_surface_init_volume_commonRef elemInit_volume;
/**
 * Init a 1D,2D,RECT,DEPTH from a compound image such as DDS @see domInit_planar
 */
	domFx_surface_init_planar_commonRef elemInit_planar;
/**
 * Initialize the surface one sub-surface at a time by specifying combinations
 * of mip, face, and slice which make sense for a particular surface type.
 * Each sub-surface is initialized by a common 2D image, not a complex compound
 * image such as DDS. If not all subsurfaces are initialized, it is invalid
 * and will result in profile and platform specific behavior unless mipmap_generate
 * is responsible for initializing the remainder of the sub-surfaces @see
 * domInit_from
 */
	domFx_surface_init_from_common_Array elemInit_from_array;
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
	 * Gets the init_as_null element.
	 * @return a daeSmartRef to the init_as_null element.
	 */
	const domInit_as_nullRef getInit_as_null() const { return elemInit_as_null; }
	/**
	 * Gets the init_as_target element.
	 * @return a daeSmartRef to the init_as_target element.
	 */
	const domInit_as_targetRef getInit_as_target() const { return elemInit_as_target; }
	/**
	 * Gets the init_cube element.
	 * @return a daeSmartRef to the init_cube element.
	 */
	const domFx_surface_init_cube_commonRef getInit_cube() const { return elemInit_cube; }
	/**
	 * Gets the init_volume element.
	 * @return a daeSmartRef to the init_volume element.
	 */
	const domFx_surface_init_volume_commonRef getInit_volume() const { return elemInit_volume; }
	/**
	 * Gets the init_planar element.
	 * @return a daeSmartRef to the init_planar element.
	 */
	const domFx_surface_init_planar_commonRef getInit_planar() const { return elemInit_planar; }
	/**
	 * Gets the init_from element array.
	 * @return Returns a reference to the array of init_from elements.
	 */
	domFx_surface_init_from_common_Array &getInit_from_array() { return elemInit_from_array; }
	/**
	 * Gets the init_from element array.
	 * @return Returns a constant reference to the array of init_from elements.
	 */
	const domFx_surface_init_from_common_Array &getInit_from_array() const { return elemInit_from_array; }
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
	domFx_surface_init_common() : elemInit_as_null(), elemInit_as_target(), elemInit_cube(), elemInit_volume(), elemInit_planar(), elemInit_from_array() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_surface_init_common() {}
	/**
	 * Copy Constructor
	 */
	domFx_surface_init_common( const domFx_surface_init_common &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_surface_init_common &operator=( const domFx_surface_init_common &cpy ) { (void)cpy; return *this; }

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
