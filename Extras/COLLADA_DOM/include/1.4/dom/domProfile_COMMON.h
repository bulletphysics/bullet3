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
#ifndef __domProfile_COMMON_h__
#define __domProfile_COMMON_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domFx_profile_abstract.h>
#include <dom/domAsset.h>
#include <dom/domImage.h>
#include <dom/domExtra.h>
#include <dom/domCommon_newparam_type.h>
#include <dom/domCommon_color_or_texture_type.h>
#include <dom/domCommon_float_or_param_type.h>
#include <dom/domCommon_transparent_type.h>

/**
 * Opens a block of COMMON platform-specific data types and technique declarations.
 */
class domProfile_COMMON : public domFx_profile_abstract
{
public:
	class domTechnique;

	typedef daeSmartRef<domTechnique> domTechniqueRef;
	typedef daeTArray<domTechniqueRef> domTechnique_Array;

/**
 * Holds a description of the textures, samplers, shaders, parameters, and
 * passes necessary for rendering this effect using one method.
 */
	class domTechnique : public daeElement
	{
	public:
		class domConstant;

		typedef daeSmartRef<domConstant> domConstantRef;
		typedef daeTArray<domConstantRef> domConstant_Array;

		class domConstant : public daeElement
		{

		protected:  // Elements
			domCommon_color_or_texture_typeRef elemEmission;
			domCommon_color_or_texture_typeRef elemReflective;
			domCommon_float_or_param_typeRef elemReflectivity;
			domCommon_transparent_typeRef elemTransparent;
			domCommon_float_or_param_typeRef elemTransparency;
			domCommon_float_or_param_typeRef elemIndex_of_refraction;

		public:	//Accessors and Mutators
			/**
			 * Gets the emission element.
			 * @return a daeSmartRef to the emission element.
			 */
			const domCommon_color_or_texture_typeRef getEmission() const { return elemEmission; }
			/**
			 * Gets the reflective element.
			 * @return a daeSmartRef to the reflective element.
			 */
			const domCommon_color_or_texture_typeRef getReflective() const { return elemReflective; }
			/**
			 * Gets the reflectivity element.
			 * @return a daeSmartRef to the reflectivity element.
			 */
			const domCommon_float_or_param_typeRef getReflectivity() const { return elemReflectivity; }
			/**
			 * Gets the transparent element.
			 * @return a daeSmartRef to the transparent element.
			 */
			const domCommon_transparent_typeRef getTransparent() const { return elemTransparent; }

			/**
			 * Gets the transparency element.
			 * @return a daeSmartRef to the transparency element.
			 */
			const domCommon_float_or_param_typeRef getTransparency() const { return elemTransparency; }
			/**
			 * Gets the index_of_refraction element.
			 * @return a daeSmartRef to the index_of_refraction element.
			 */
			const domCommon_float_or_param_typeRef getIndex_of_refraction() const { return elemIndex_of_refraction; }
		protected:
			/**
			 * Constructor
			 */
			domConstant() : elemEmission(), elemReflective(), elemReflectivity(), elemTransparent(), elemTransparency(), elemIndex_of_refraction() {}
			/**
			 * Destructor
			 */
			virtual ~domConstant() {}
			/**
			 * Copy Constructor
			 */
			domConstant( const domConstant &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domConstant &operator=( const domConstant &cpy ) { (void)cpy; return *this; }

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

		class domLambert;

		typedef daeSmartRef<domLambert> domLambertRef;
		typedef daeTArray<domLambertRef> domLambert_Array;

		class domLambert : public daeElement
		{

		protected:  // Elements
			domCommon_color_or_texture_typeRef elemEmission;
			domCommon_color_or_texture_typeRef elemAmbient;
			domCommon_color_or_texture_typeRef elemDiffuse;
			domCommon_color_or_texture_typeRef elemReflective;
			domCommon_float_or_param_typeRef elemReflectivity;
			domCommon_transparent_typeRef elemTransparent;
			domCommon_float_or_param_typeRef elemTransparency;
			domCommon_float_or_param_typeRef elemIndex_of_refraction;

		public:	//Accessors and Mutators
			/**
			 * Gets the emission element.
			 * @return a daeSmartRef to the emission element.
			 */
			const domCommon_color_or_texture_typeRef getEmission() const { return elemEmission; }
			/**
			 * Gets the ambient element.
			 * @return a daeSmartRef to the ambient element.
			 */
			const domCommon_color_or_texture_typeRef getAmbient() const { return elemAmbient; }
			/**
			 * Gets the diffuse element.
			 * @return a daeSmartRef to the diffuse element.
			 */
			const domCommon_color_or_texture_typeRef getDiffuse() const { return elemDiffuse; }
			/**
			 * Gets the reflective element.
			 * @return a daeSmartRef to the reflective element.
			 */
			const domCommon_color_or_texture_typeRef getReflective() const { return elemReflective; }
			/**
			 * Gets the reflectivity element.
			 * @return a daeSmartRef to the reflectivity element.
			 */
			const domCommon_float_or_param_typeRef getReflectivity() const { return elemReflectivity; }
			/**
			 * Gets the transparent element.
			 * @return a daeSmartRef to the transparent element.
			 */
			const domCommon_transparent_typeRef getTransparent() const { return elemTransparent; }

			/**
			 * Gets the transparency element.
			 * @return a daeSmartRef to the transparency element.
			 */
			const domCommon_float_or_param_typeRef getTransparency() const { return elemTransparency; }
			/**
			 * Gets the index_of_refraction element.
			 * @return a daeSmartRef to the index_of_refraction element.
			 */
			const domCommon_float_or_param_typeRef getIndex_of_refraction() const { return elemIndex_of_refraction; }
		protected:
			/**
			 * Constructor
			 */
			domLambert() : elemEmission(), elemAmbient(), elemDiffuse(), elemReflective(), elemReflectivity(), elemTransparent(), elemTransparency(), elemIndex_of_refraction() {}
			/**
			 * Destructor
			 */
			virtual ~domLambert() {}
			/**
			 * Copy Constructor
			 */
			domLambert( const domLambert &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domLambert &operator=( const domLambert &cpy ) { (void)cpy; return *this; }

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

		class domPhong;

		typedef daeSmartRef<domPhong> domPhongRef;
		typedef daeTArray<domPhongRef> domPhong_Array;

		class domPhong : public daeElement
		{

		protected:  // Elements
			domCommon_color_or_texture_typeRef elemEmission;
			domCommon_color_or_texture_typeRef elemAmbient;
			domCommon_color_or_texture_typeRef elemDiffuse;
			domCommon_color_or_texture_typeRef elemSpecular;
			domCommon_float_or_param_typeRef elemShininess;
			domCommon_color_or_texture_typeRef elemReflective;
			domCommon_float_or_param_typeRef elemReflectivity;
			domCommon_transparent_typeRef elemTransparent;
			domCommon_float_or_param_typeRef elemTransparency;
			domCommon_float_or_param_typeRef elemIndex_of_refraction;

		public:	//Accessors and Mutators
			/**
			 * Gets the emission element.
			 * @return a daeSmartRef to the emission element.
			 */
			const domCommon_color_or_texture_typeRef getEmission() const { return elemEmission; }
			/**
			 * Gets the ambient element.
			 * @return a daeSmartRef to the ambient element.
			 */
			const domCommon_color_or_texture_typeRef getAmbient() const { return elemAmbient; }
			/**
			 * Gets the diffuse element.
			 * @return a daeSmartRef to the diffuse element.
			 */
			const domCommon_color_or_texture_typeRef getDiffuse() const { return elemDiffuse; }
			/**
			 * Gets the specular element.
			 * @return a daeSmartRef to the specular element.
			 */
			const domCommon_color_or_texture_typeRef getSpecular() const { return elemSpecular; }
			/**
			 * Gets the shininess element.
			 * @return a daeSmartRef to the shininess element.
			 */
			const domCommon_float_or_param_typeRef getShininess() const { return elemShininess; }
			/**
			 * Gets the reflective element.
			 * @return a daeSmartRef to the reflective element.
			 */
			const domCommon_color_or_texture_typeRef getReflective() const { return elemReflective; }
			/**
			 * Gets the reflectivity element.
			 * @return a daeSmartRef to the reflectivity element.
			 */
			const domCommon_float_or_param_typeRef getReflectivity() const { return elemReflectivity; }
			/**
			 * Gets the transparent element.
			 * @return a daeSmartRef to the transparent element.
			 */
			const domCommon_transparent_typeRef getTransparent() const { return elemTransparent; }

			/**
			 * Gets the transparency element.
			 * @return a daeSmartRef to the transparency element.
			 */
			const domCommon_float_or_param_typeRef getTransparency() const { return elemTransparency; }
			/**
			 * Gets the index_of_refraction element.
			 * @return a daeSmartRef to the index_of_refraction element.
			 */
			const domCommon_float_or_param_typeRef getIndex_of_refraction() const { return elemIndex_of_refraction; }
		protected:
			/**
			 * Constructor
			 */
			domPhong() : elemEmission(), elemAmbient(), elemDiffuse(), elemSpecular(), elemShininess(), elemReflective(), elemReflectivity(), elemTransparent(), elemTransparency(), elemIndex_of_refraction() {}
			/**
			 * Destructor
			 */
			virtual ~domPhong() {}
			/**
			 * Copy Constructor
			 */
			domPhong( const domPhong &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domPhong &operator=( const domPhong &cpy ) { (void)cpy; return *this; }

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

		class domBlinn;

		typedef daeSmartRef<domBlinn> domBlinnRef;
		typedef daeTArray<domBlinnRef> domBlinn_Array;

		class domBlinn : public daeElement
		{

		protected:  // Elements
			domCommon_color_or_texture_typeRef elemEmission;
			domCommon_color_or_texture_typeRef elemAmbient;
			domCommon_color_or_texture_typeRef elemDiffuse;
			domCommon_color_or_texture_typeRef elemSpecular;
			domCommon_float_or_param_typeRef elemShininess;
			domCommon_color_or_texture_typeRef elemReflective;
			domCommon_float_or_param_typeRef elemReflectivity;
			domCommon_transparent_typeRef elemTransparent;
			domCommon_float_or_param_typeRef elemTransparency;
			domCommon_float_or_param_typeRef elemIndex_of_refraction;

		public:	//Accessors and Mutators
			/**
			 * Gets the emission element.
			 * @return a daeSmartRef to the emission element.
			 */
			const domCommon_color_or_texture_typeRef getEmission() const { return elemEmission; }
			/**
			 * Gets the ambient element.
			 * @return a daeSmartRef to the ambient element.
			 */
			const domCommon_color_or_texture_typeRef getAmbient() const { return elemAmbient; }
			/**
			 * Gets the diffuse element.
			 * @return a daeSmartRef to the diffuse element.
			 */
			const domCommon_color_or_texture_typeRef getDiffuse() const { return elemDiffuse; }
			/**
			 * Gets the specular element.
			 * @return a daeSmartRef to the specular element.
			 */
			const domCommon_color_or_texture_typeRef getSpecular() const { return elemSpecular; }
			/**
			 * Gets the shininess element.
			 * @return a daeSmartRef to the shininess element.
			 */
			const domCommon_float_or_param_typeRef getShininess() const { return elemShininess; }
			/**
			 * Gets the reflective element.
			 * @return a daeSmartRef to the reflective element.
			 */
			const domCommon_color_or_texture_typeRef getReflective() const { return elemReflective; }
			/**
			 * Gets the reflectivity element.
			 * @return a daeSmartRef to the reflectivity element.
			 */
			const domCommon_float_or_param_typeRef getReflectivity() const { return elemReflectivity; }
			/**
			 * Gets the transparent element.
			 * @return a daeSmartRef to the transparent element.
			 */
			const domCommon_transparent_typeRef getTransparent() const { return elemTransparent; }
			/**
			 * Gets the transparency element.
			 * @return a daeSmartRef to the transparency element.
			 */
			const domCommon_float_or_param_typeRef getTransparency() const { return elemTransparency; }
			/**
			 * Gets the index_of_refraction element.
			 * @return a daeSmartRef to the index_of_refraction element.
			 */
			const domCommon_float_or_param_typeRef getIndex_of_refraction() const { return elemIndex_of_refraction; }
		protected:
			/**
			 * Constructor
			 */
			domBlinn() : elemEmission(), elemAmbient(), elemDiffuse(), elemSpecular(), elemShininess(), elemReflective(), elemReflectivity(), elemTransparent(), elemTransparency(), elemIndex_of_refraction() {}
			/**
			 * Destructor
			 */
			virtual ~domBlinn() {}
			/**
			 * Copy Constructor
			 */
			domBlinn( const domBlinn &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domBlinn &operator=( const domBlinn &cpy ) { (void)cpy; return *this; }

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


	protected:  // Attributes
/**
 *  The id attribute is a text string containing the unique identifier of
 * this element.  This value must be unique within the instance document.
 * Optional attribute. 
 */
		xsID attrId;
/**
 *  The sid attribute is a text string value containing the sub-identifier
 * of this element.  This value must be unique within the scope of the parent
 * element. Optional attribute. 
 */
		xsNCName attrSid;

	protected:  // Elements
/**
 *  The technique element may contain an asset element.  @see domAsset
 */
		domAssetRef elemAsset;
		domImage_Array elemImage_array;
		domCommon_newparam_type_Array elemNewparam_array;
		domConstantRef elemConstant;
		domLambertRef elemLambert;
		domPhongRef elemPhong;
		domBlinnRef elemBlinn;
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
		 * Gets the sid attribute.
		 * @return Returns a xsNCName of the sid attribute.
		 */
		xsNCName getSid() const { return attrSid; }
		/**
		 * Sets the sid attribute.
		 * @param atSid The new value for the sid attribute.
		 */
		void setSid( xsNCName atSid ) { *(daeStringRef*)&attrSid = atSid;	
	 _validAttributeArray[1] = true; }

		/**
		 * Gets the asset element.
		 * @return a daeSmartRef to the asset element.
		 */
		const domAssetRef getAsset() const { return elemAsset; }
		/**
		 * Gets the image element array.
		 * @return Returns a reference to the array of image elements.
		 */
		domImage_Array &getImage_array() { return elemImage_array; }
		/**
		 * Gets the image element array.
		 * @return Returns a constant reference to the array of image elements.
		 */
		const domImage_Array &getImage_array() const { return elemImage_array; }
		/**
		 * Gets the newparam element array.
		 * @return Returns a reference to the array of newparam elements.
		 */
		domCommon_newparam_type_Array &getNewparam_array() { return elemNewparam_array; }
		/**
		 * Gets the newparam element array.
		 * @return Returns a constant reference to the array of newparam elements.
		 */
		const domCommon_newparam_type_Array &getNewparam_array() const { return elemNewparam_array; }
		/**
		 * Gets the constant element.
		 * @return a daeSmartRef to the constant element.
		 */
		const domConstantRef getConstant() const { return elemConstant; }
		/**
		 * Gets the lambert element.
		 * @return a daeSmartRef to the lambert element.
		 */
		const domLambertRef getLambert() const { return elemLambert; }
		/**
		 * Gets the phong element.
		 * @return a daeSmartRef to the phong element.
		 */
		const domPhongRef getPhong() const { return elemPhong; }
		/**
		 * Gets the blinn element.
		 * @return a daeSmartRef to the blinn element.
		 */
		const domBlinnRef getBlinn() const { return elemBlinn; }
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
		domTechnique() : attrId(), attrSid(), elemAsset(), elemImage_array(), elemNewparam_array(), elemConstant(), elemLambert(), elemPhong(), elemBlinn(), elemExtra_array() {}
		/**
		 * Destructor
		 */
		virtual ~domTechnique() {}
		/**
		 * Copy Constructor
		 */
		domTechnique( const domTechnique &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domTechnique &operator=( const domTechnique &cpy ) { (void)cpy; return *this; }

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


protected:  // Attribute
/**
 *  The id attribute is a text string containing the unique identifier of
 * this element.  This value must be unique within the instance document.
 * Optional attribute. 
 */
	xsID attrId;

protected:  // Elements
	domAssetRef elemAsset;
	domImage_Array elemImage_array;
	domCommon_newparam_type_Array elemNewparam_array;
/**
 * Holds a description of the textures, samplers, shaders, parameters, and
 * passes necessary for rendering this effect using one method. @see domTechnique
 */
	domTechniqueRef elemTechnique;
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
	 * Gets the asset element.
	 * @return a daeSmartRef to the asset element.
	 */
	const domAssetRef getAsset() const { return elemAsset; }
	/**
	 * Gets the image element array.
	 * @return Returns a reference to the array of image elements.
	 */
	domImage_Array &getImage_array() { return elemImage_array; }
	/**
	 * Gets the image element array.
	 * @return Returns a constant reference to the array of image elements.
	 */
	const domImage_Array &getImage_array() const { return elemImage_array; }
	/**
	 * Gets the newparam element array.
	 * @return Returns a reference to the array of newparam elements.
	 */
	domCommon_newparam_type_Array &getNewparam_array() { return elemNewparam_array; }
	/**
	 * Gets the newparam element array.
	 * @return Returns a constant reference to the array of newparam elements.
	 */
	const domCommon_newparam_type_Array &getNewparam_array() const { return elemNewparam_array; }
	/**
	 * Gets the technique element.
	 * @return a daeSmartRef to the technique element.
	 */
	const domTechniqueRef getTechnique() const { return elemTechnique; }
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
	domProfile_COMMON() : attrId(), elemAsset(), elemImage_array(), elemNewparam_array(), elemTechnique(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domProfile_COMMON() {}
	/**
	 * Copy Constructor
	 */
	domProfile_COMMON( const domProfile_COMMON &cpy ) : domFx_profile_abstract() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domProfile_COMMON &operator=( const domProfile_COMMON &cpy ) { (void)cpy; return *this; }

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
