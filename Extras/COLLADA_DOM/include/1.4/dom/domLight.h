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
#ifndef __domLight_h__
#define __domLight_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domAsset.h>
#include <dom/domTechnique.h>
#include <dom/domExtra.h>
#include <dom/domTargetableFloat3.h>
#include <dom/domTargetableFloat.h>

/**
 * The light element declares a light source that illuminates the scene. Light
 * sources have many different properties and radiate light in many different
 * patterns and  frequencies.
 */
class domLight : public daeElement
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::LIGHT; }
public:
	class domTechnique_common;

	typedef daeSmartRef<domTechnique_common> domTechnique_commonRef;
	typedef daeTArray<domTechnique_commonRef> domTechnique_common_Array;

/**
 * The technique_common element specifies the light information for the common
 * profile which all  COLLADA implementations need to support.
 */
	class domTechnique_common : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::TECHNIQUE_COMMON; }
	public:
		class domAmbient;

		typedef daeSmartRef<domAmbient> domAmbientRef;
		typedef daeTArray<domAmbientRef> domAmbient_Array;

/**
 * The ambient element declares the parameters required to describe an ambient
 * light source.   An ambient light is one that lights everything evenly,
 * regardless of location or orientation.
 */
		class domAmbient : public daeElement
		{
		public:
			COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::AMBIENT; }

		protected:  // Element
/**
 * The color element contains three floating point numbers specifying the
 * color of the light. The color element must occur exactly once. @see domColor
 */
			domTargetableFloat3Ref elemColor;

		public:	//Accessors and Mutators
			/**
			 * Gets the color element.
			 * @return a daeSmartRef to the color element.
			 */
			const domTargetableFloat3Ref getColor() const { return elemColor; }
		protected:
			/**
			 * Constructor
			 */
			domAmbient() : elemColor() {}
			/**
			 * Destructor
			 */
			virtual ~domAmbient() {}
			/**
			 * Copy Constructor
			 */
			domAmbient( const domAmbient &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domAmbient &operator=( const domAmbient &cpy ) { (void)cpy; return *this; }

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

		class domDirectional;

		typedef daeSmartRef<domDirectional> domDirectionalRef;
		typedef daeTArray<domDirectionalRef> domDirectional_Array;

/**
 * The directional element declares the parameters required to describe a
 * directional light source.   A directional light is one that lights everything
 * from the same direction, regardless of location.   The light’s default
 * direction vector in local coordinates is [0,0,-1], pointing down the -Z
 * axis.  The actual direction of the light is defined by the transform of
 * the node where the light is  instantiated.
 */
		class domDirectional : public daeElement
		{
		public:
			COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::DIRECTIONAL; }

		protected:  // Element
/**
 * The color element contains three floating point numbers specifying the
 * color of the light. The color element must occur exactly once. @see domColor
 */
			domTargetableFloat3Ref elemColor;

		public:	//Accessors and Mutators
			/**
			 * Gets the color element.
			 * @return a daeSmartRef to the color element.
			 */
			const domTargetableFloat3Ref getColor() const { return elemColor; }
		protected:
			/**
			 * Constructor
			 */
			domDirectional() : elemColor() {}
			/**
			 * Destructor
			 */
			virtual ~domDirectional() {}
			/**
			 * Copy Constructor
			 */
			domDirectional( const domDirectional &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domDirectional &operator=( const domDirectional &cpy ) { (void)cpy; return *this; }

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

		class domPoint;

		typedef daeSmartRef<domPoint> domPointRef;
		typedef daeTArray<domPointRef> domPoint_Array;

/**
 * The point element declares the parameters required to describe a point
 * light source.  A point light  source radiates light in all directions from
 * a known location in space. The intensity of a point  light source is attenuated
 * as the distance to the light source increases. The position of the light
 * is defined by the transform of the node in which it is instantiated.
 */
		class domPoint : public daeElement
		{
		public:
			COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::POINT; }

		protected:  // Elements
/**
 * The color element contains three floating point numbers specifying the
 * color of the light. The color element must occur exactly once. @see domColor
 */
			domTargetableFloat3Ref elemColor;
/**
 * The constant_attenuation is used to calculate the total attenuation of
 * this light given a distance.  The equation used is A = constant_attenuation
 * + Dist*linear_attenuation + Dist^2*quadratic_attenuation. @see domConstant_attenuation
 */
			domTargetableFloatRef elemConstant_attenuation;
/**
 * The linear_attenuation is used to calculate the total attenuation of this
 * light given a distance.  The equation used is A = constant_attenuation
 * + Dist*linear_attenuation + Dist^2*quadratic_attenuation. @see domLinear_attenuation
 */
			domTargetableFloatRef elemLinear_attenuation;
/**
 * The quadratic_attenuation is used to calculate the total attenuation of
 * this light given a distance.  The equation used is A = constant_attenuation
 * + Dist*linear_attenuation + Dist^2*quadratic_attenuation. @see domQuadratic_attenuation
 */
			domTargetableFloatRef elemQuadratic_attenuation;

		public:	//Accessors and Mutators
			/**
			 * Gets the color element.
			 * @return a daeSmartRef to the color element.
			 */
			const domTargetableFloat3Ref getColor() const { return elemColor; }
			/**
			 * Gets the constant_attenuation element.
			 * @return a daeSmartRef to the constant_attenuation element.
			 */
			const domTargetableFloatRef getConstant_attenuation() const { return elemConstant_attenuation; }
			/**
			 * Gets the linear_attenuation element.
			 * @return a daeSmartRef to the linear_attenuation element.
			 */
			const domTargetableFloatRef getLinear_attenuation() const { return elemLinear_attenuation; }
			/**
			 * Gets the quadratic_attenuation element.
			 * @return a daeSmartRef to the quadratic_attenuation element.
			 */
			const domTargetableFloatRef getQuadratic_attenuation() const { return elemQuadratic_attenuation; }
		protected:
			/**
			 * Constructor
			 */
			domPoint() : elemColor(), elemConstant_attenuation(), elemLinear_attenuation(), elemQuadratic_attenuation() {}
			/**
			 * Destructor
			 */
			virtual ~domPoint() {}
			/**
			 * Copy Constructor
			 */
			domPoint( const domPoint &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domPoint &operator=( const domPoint &cpy ) { (void)cpy; return *this; }

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

		class domSpot;

		typedef daeSmartRef<domSpot> domSpotRef;
		typedef daeTArray<domSpotRef> domSpot_Array;

/**
 * The spot element declares the parameters required to describe a spot light
 * source.  A spot light  source radiates light in one direction from a known
 * location in space. The light radiates from  the spot light source in a
 * cone shape. The intensity of the light is attenuated as the radiation 
 * angle increases away from the direction of the light source. The intensity
 * of a spot light source  is also attenuated as the distance to the light
 * source increases. The position of the light is  defined by the transform
 * of the node in which it is instantiated. The light’s default direction
 * vector in local coordinates is [0,0,-1], pointing down the -Z axis. The
 * actual direction of the  light is defined by the transform of the node
 * where the light is instantiated.
 */
		class domSpot : public daeElement
		{
		public:
			COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::SPOT; }

		protected:  // Elements
/**
 * The color element contains three floating point numbers specifying the
 * color of the light. The color element must occur exactly once. @see domColor
 */
			domTargetableFloat3Ref elemColor;
/**
 * The constant_attenuation is used to calculate the total attenuation of
 * this light given a distance.  The equation used is A = constant_attenuation
 * + Dist*linear_attenuation + Dist^2*quadratic_attenuation. @see domConstant_attenuation
 */
			domTargetableFloatRef elemConstant_attenuation;
/**
 * The linear_attenuation is used to calculate the total attenuation of this
 * light given a distance.  The equation used is A = constant_attenuation
 * + Dist*linear_attenuation + Dist^2*quadratic_attenuation. @see domLinear_attenuation
 */
			domTargetableFloatRef elemLinear_attenuation;
/**
 * The quadratic_attenuation is used to calculate the total attenuation of
 * this light given a distance.  The equation used is A = constant_attenuation
 * + Dist*linear_attenuation + Dist^2*quadratic_attenuation. @see domQuadratic_attenuation
 */
			domTargetableFloatRef elemQuadratic_attenuation;
/**
 * The falloff_angle is used to specify the amount of attenuation based on
 * the direction of the light. @see domFalloff_angle
 */
			domTargetableFloatRef elemFalloff_angle;
/**
 * The falloff_exponent is used to specify the amount of attenuation based
 * on the direction of the light. @see domFalloff_exponent
 */
			domTargetableFloatRef elemFalloff_exponent;

		public:	//Accessors and Mutators
			/**
			 * Gets the color element.
			 * @return a daeSmartRef to the color element.
			 */
			const domTargetableFloat3Ref getColor() const { return elemColor; }
			/**
			 * Gets the constant_attenuation element.
			 * @return a daeSmartRef to the constant_attenuation element.
			 */
			const domTargetableFloatRef getConstant_attenuation() const { return elemConstant_attenuation; }
			/**
			 * Gets the linear_attenuation element.
			 * @return a daeSmartRef to the linear_attenuation element.
			 */
			const domTargetableFloatRef getLinear_attenuation() const { return elemLinear_attenuation; }
			/**
			 * Gets the quadratic_attenuation element.
			 * @return a daeSmartRef to the quadratic_attenuation element.
			 */
			const domTargetableFloatRef getQuadratic_attenuation() const { return elemQuadratic_attenuation; }
			/**
			 * Gets the falloff_angle element.
			 * @return a daeSmartRef to the falloff_angle element.
			 */
			const domTargetableFloatRef getFalloff_angle() const { return elemFalloff_angle; }
			/**
			 * Gets the falloff_exponent element.
			 * @return a daeSmartRef to the falloff_exponent element.
			 */
			const domTargetableFloatRef getFalloff_exponent() const { return elemFalloff_exponent; }
		protected:
			/**
			 * Constructor
			 */
			domSpot() : elemColor(), elemConstant_attenuation(), elemLinear_attenuation(), elemQuadratic_attenuation(), elemFalloff_angle(), elemFalloff_exponent() {}
			/**
			 * Destructor
			 */
			virtual ~domSpot() {}
			/**
			 * Copy Constructor
			 */
			domSpot( const domSpot &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domSpot &operator=( const domSpot &cpy ) { (void)cpy; return *this; }

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
 * The ambient element declares the parameters required to describe an ambient
 * light source.   An ambient light is one that lights everything evenly,
 * regardless of location or orientation. @see domAmbient
 */
		domAmbientRef elemAmbient;
/**
 * The directional element declares the parameters required to describe a
 * directional light source.   A directional light is one that lights everything
 * from the same direction, regardless of location.   The light’s default
 * direction vector in local coordinates is [0,0,-1], pointing down the -Z
 * axis.  The actual direction of the light is defined by the transform of
 * the node where the light is  instantiated. @see domDirectional
 */
		domDirectionalRef elemDirectional;
/**
 * The point element declares the parameters required to describe a point
 * light source.  A point light  source radiates light in all directions from
 * a known location in space. The intensity of a point  light source is attenuated
 * as the distance to the light source increases. The position of the light
 * is defined by the transform of the node in which it is instantiated. @see
 * domPoint
 */
		domPointRef elemPoint;
/**
 * The spot element declares the parameters required to describe a spot light
 * source.  A spot light  source radiates light in one direction from a known
 * location in space. The light radiates from  the spot light source in a
 * cone shape. The intensity of the light is attenuated as the radiation 
 * angle increases away from the direction of the light source. The intensity
 * of a spot light source  is also attenuated as the distance to the light
 * source increases. The position of the light is  defined by the transform
 * of the node in which it is instantiated. The light’s default direction
 * vector in local coordinates is [0,0,-1], pointing down the -Z axis. The
 * actual direction of the  light is defined by the transform of the node
 * where the light is instantiated. @see domSpot
 */
		domSpotRef elemSpot;
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
		 * Gets the ambient element.
		 * @return a daeSmartRef to the ambient element.
		 */
		const domAmbientRef getAmbient() const { return elemAmbient; }
		/**
		 * Gets the directional element.
		 * @return a daeSmartRef to the directional element.
		 */
		const domDirectionalRef getDirectional() const { return elemDirectional; }
		/**
		 * Gets the point element.
		 * @return a daeSmartRef to the point element.
		 */
		const domPointRef getPoint() const { return elemPoint; }
		/**
		 * Gets the spot element.
		 * @return a daeSmartRef to the spot element.
		 */
		const domSpotRef getSpot() const { return elemSpot; }
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
		domTechnique_common() : elemAmbient(), elemDirectional(), elemPoint(), elemSpot() {}
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


protected:  // Attributes
/**
 *  The id attribute is a text string containing the unique identifier of
 * this element.  This value must be unique within the instance document.
 * Optional attribute. 
 */
	xsID attrId;
/**
 *  The name attribute is the text string name of this element. Optional attribute.
 */
	xsNCName attrName;

protected:  // Elements
/**
 *  The light element may contain an asset element.  @see domAsset
 */
	domAssetRef elemAsset;
/**
 * The technique_common element specifies the light information for the common
 * profile which all  COLLADA implementations need to support. @see domTechnique_common
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
	 * Gets the id attribute.
	 * @return Returns a xsID of the id attribute.
	 */
	xsID getId() const { return attrId; }
	/**
	 * Sets the id attribute.
	 * @param atId The new value for the id attribute.
	 */
	void setId( xsID atId ) { *(daeStringRef*)&attrId = atId;
	 _validAttributeArray[0] = true; }

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
	 _validAttributeArray[1] = true; }

	/**
	 * Gets the asset element.
	 * @return a daeSmartRef to the asset element.
	 */
	const domAssetRef getAsset() const { return elemAsset; }
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
	domLight() : attrId(), attrName(), elemAsset(), elemTechnique_common(), elemTechnique_array(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domLight() {}
	/**
	 * Copy Constructor
	 */
	domLight( const domLight &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domLight &operator=( const domLight &cpy ) { (void)cpy; return *this; }

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
