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
#ifndef __domProfile_GLES_h__
#define __domProfile_GLES_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domFx_profile_abstract.h>
#include <dom/domAsset.h>
#include <dom/domImage.h>
#include <dom/domExtra.h>
#include <dom/domGles_newparam.h>
#include <dom/domFx_annotate_common.h>
#include <dom/domGles_basic_type_common.h>
#include <dom/domGles_pipeline_settings.h>

/**
 * Opens a block of GLES platform-specific data types and technique declarations.
 */
class domProfile_GLES : public domFx_profile_abstract
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
		class domSetparam;

		typedef daeSmartRef<domSetparam> domSetparamRef;
		typedef daeTArray<domSetparamRef> domSetparam_Array;

		class domSetparam : public daeElement
		{
		protected:  // Attribute
			xsNCName attrRef;

		protected:  // Elements
			domFx_annotate_common_Array elemAnnotate_array;
			domGles_basic_type_commonRef elemGles_basic_type_common;

		public:	//Accessors and Mutators
			/**
			 * Gets the ref attribute.
			 * @return Returns a xsNCName of the ref attribute.
			 */
			xsNCName getRef() const { return attrRef; }
			/**
			 * Sets the ref attribute.
			 * @param atRef The new value for the ref attribute.
			 */
			void setRef( xsNCName atRef ) { *(daeStringRef*)&attrRef = atRef;		
	 _validAttributeArray[0] = true; }

			/**
			 * Gets the annotate element array.
			 * @return Returns a reference to the array of annotate elements.
			 */
			domFx_annotate_common_Array &getAnnotate_array() { return elemAnnotate_array; }
			/**
			 * Gets the annotate element array.
			 * @return Returns a constant reference to the array of annotate elements.
			 */
			const domFx_annotate_common_Array &getAnnotate_array() const { return elemAnnotate_array; }
			/**
			 * Gets the gles_basic_type_common element.
			 * @return a daeSmartRef to the gles_basic_type_common element.
			 */
			const domGles_basic_type_commonRef getGles_basic_type_common() const { return elemGles_basic_type_common; }
		protected:
			/**
			 * Constructor
			 */
			domSetparam() : attrRef(), elemAnnotate_array(), elemGles_basic_type_common() {}
			/**
			 * Destructor
			 */
			virtual ~domSetparam() {}
			/**
			 * Copy Constructor
			 */
			domSetparam( const domSetparam &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domSetparam &operator=( const domSetparam &cpy ) { (void)cpy; return *this; }

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

		class domPass;

		typedef daeSmartRef<domPass> domPassRef;
		typedef daeTArray<domPassRef> domPass_Array;

/**
 * A static declaration of all the render states, shaders, and settings for
 * one rendering pipeline.
 */
		class domPass : public daeElement
		{
		public:
			class domColor_target;

			typedef daeSmartRef<domColor_target> domColor_targetRef;
			typedef daeTArray<domColor_targetRef> domColor_target_Array;

			class domColor_target : public daeElement
			{

			protected:  // Value
				/**
				 * The domGles_rendertarget_common value of the text data of this element. 
				 */
				domGles_rendertarget_common _value;

			public:	//Accessors and Mutators
				/**
				 * Gets the value of this element.
				 * @return a domGles_rendertarget_common of the value.
				 */
				domGles_rendertarget_common getValue() const { return _value; }
				/**
				 * Sets the _value of this element.
				 * @param val The new value for this element.
				 */
				void setValue( domGles_rendertarget_common val ) { _value = val; }

			protected:
				/**
				 * Constructor
				 */
				domColor_target() : _value() {}
				/**
				 * Destructor
				 */
				virtual ~domColor_target() {}
				/**
				 * Copy Constructor
				 */
				domColor_target( const domColor_target &cpy ) : daeElement() { (void)cpy; }
				/**
				 * Overloaded assignment operator
				 */
				virtual domColor_target &operator=( const domColor_target &cpy ) { (void)cpy; return *this; }

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

			class domDepth_target;

			typedef daeSmartRef<domDepth_target> domDepth_targetRef;
			typedef daeTArray<domDepth_targetRef> domDepth_target_Array;

			class domDepth_target : public daeElement
			{

			protected:  // Value
				/**
				 * The domGles_rendertarget_common value of the text data of this element. 
				 */
				domGles_rendertarget_common _value;

			public:	//Accessors and Mutators
				/**
				 * Gets the value of this element.
				 * @return a domGles_rendertarget_common of the value.
				 */
				domGles_rendertarget_common getValue() const { return _value; }
				/**
				 * Sets the _value of this element.
				 * @param val The new value for this element.
				 */
				void setValue( domGles_rendertarget_common val ) { _value = val; }

			protected:
				/**
				 * Constructor
				 */
				domDepth_target() : _value() {}
				/**
				 * Destructor
				 */
				virtual ~domDepth_target() {}
				/**
				 * Copy Constructor
				 */
				domDepth_target( const domDepth_target &cpy ) : daeElement() { (void)cpy; }
				/**
				 * Overloaded assignment operator
				 */
				virtual domDepth_target &operator=( const domDepth_target &cpy ) { (void)cpy; return *this; }

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

			class domStencil_target;

			typedef daeSmartRef<domStencil_target> domStencil_targetRef;
			typedef daeTArray<domStencil_targetRef> domStencil_target_Array;

			class domStencil_target : public daeElement
			{

			protected:  // Value
				/**
				 * The domGles_rendertarget_common value of the text data of this element. 
				 */
				domGles_rendertarget_common _value;

			public:	//Accessors and Mutators
				/**
				 * Gets the value of this element.
				 * @return a domGles_rendertarget_common of the value.
				 */
				domGles_rendertarget_common getValue() const { return _value; }
				/**
				 * Sets the _value of this element.
				 * @param val The new value for this element.
				 */
				void setValue( domGles_rendertarget_common val ) { _value = val; }

			protected:
				/**
				 * Constructor
				 */
				domStencil_target() : _value() {}
				/**
				 * Destructor
				 */
				virtual ~domStencil_target() {}
				/**
				 * Copy Constructor
				 */
				domStencil_target( const domStencil_target &cpy ) : daeElement() { (void)cpy; }
				/**
				 * Overloaded assignment operator
				 */
				virtual domStencil_target &operator=( const domStencil_target &cpy ) { (void)cpy; return *this; }

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

			class domColor_clear;

			typedef daeSmartRef<domColor_clear> domColor_clearRef;
			typedef daeTArray<domColor_clearRef> domColor_clear_Array;

			class domColor_clear : public daeElement
			{

			protected:  // Value
				/**
				 * The domFx_color_common value of the text data of this element. 
				 */
				domFx_color_common _value;

			public:	//Accessors and Mutators
				/**
				 * Gets the _value array.
				 * @return Returns a domFx_color_common reference of the _value array.
				 */
				domFx_color_common &getValue() { return _value; }
				/**
				 * Gets the _value array.
				 * @return Returns a constant domFx_color_common reference of the _value array.
				 */
				const domFx_color_common &getValue() const { return _value; }
				/**
				 * Sets the _value array.
				 * @param val The new value for the _value array.
				 */
				void setValue( const domFx_color_common &val ) { _value = val; }

			protected:
				/**
				 * Constructor
				 */
				domColor_clear() : _value() {}
				/**
				 * Destructor
				 */
				virtual ~domColor_clear() {}
				/**
				 * Copy Constructor
				 */
				domColor_clear( const domColor_clear &cpy ) : daeElement() { (void)cpy; }
				/**
				 * Overloaded assignment operator
				 */
				virtual domColor_clear &operator=( const domColor_clear &cpy ) { (void)cpy; return *this; }

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

			class domDepth_clear;

			typedef daeSmartRef<domDepth_clear> domDepth_clearRef;
			typedef daeTArray<domDepth_clearRef> domDepth_clear_Array;

			class domDepth_clear : public daeElement
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
				domDepth_clear() : _value() {}
				/**
				 * Destructor
				 */
				virtual ~domDepth_clear() {}
				/**
				 * Copy Constructor
				 */
				domDepth_clear( const domDepth_clear &cpy ) : daeElement() { (void)cpy; }
				/**
				 * Overloaded assignment operator
				 */
				virtual domDepth_clear &operator=( const domDepth_clear &cpy ) { (void)cpy; return *this; }

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

			class domStencil_clear;

			typedef daeSmartRef<domStencil_clear> domStencil_clearRef;
			typedef daeTArray<domStencil_clearRef> domStencil_clear_Array;

			class domStencil_clear : public daeElement
			{

			protected:  // Value
				/**
				 * The xsByte value of the text data of this element. 
				 */
				xsByte _value;

			public:	//Accessors and Mutators
				/**
				 * Gets the value of this element.
				 * @return a xsByte of the value.
				 */
				xsByte getValue() const { return _value; }
				/**
				 * Sets the _value of this element.
				 * @param val The new value for this element.
				 */
				void setValue( xsByte val ) { _value = val; }

			protected:
				/**
				 * Constructor
				 */
				domStencil_clear() : _value() {}
				/**
				 * Destructor
				 */
				virtual ~domStencil_clear() {}
				/**
				 * Copy Constructor
				 */
				domStencil_clear( const domStencil_clear &cpy ) : daeElement() { (void)cpy; }
				/**
				 * Overloaded assignment operator
				 */
				virtual domStencil_clear &operator=( const domStencil_clear &cpy ) { (void)cpy; return *this; }

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

			class domDraw;

			typedef daeSmartRef<domDraw> domDrawRef;
			typedef daeTArray<domDrawRef> domDraw_Array;

			class domDraw : public daeElement
			{

			protected:  // Value
				/**
				 * The domFx_draw_common value of the text data of this element. 
				 */
				domFx_draw_common _value;

			public:	//Accessors and Mutators
				/**
				 * Gets the value of this element.
				 * @return a domFx_draw_common of the value.
				 */
				domFx_draw_common getValue() const { return _value; }
				/**
				 * Sets the _value of this element.
				 * @param val The new value for this element.
				 */
				void setValue( domFx_draw_common val ) { _value = val; }

			protected:
				/**
				 * Constructor
				 */
				domDraw() : _value() {}
				/**
				 * Destructor
				 */
				virtual ~domDraw() {}
				/**
				 * Copy Constructor
				 */
				domDraw( const domDraw &cpy ) : daeElement() { (void)cpy; }
				/**
				 * Overloaded assignment operator
				 */
				virtual domDraw &operator=( const domDraw &cpy ) { (void)cpy; return *this; }

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
 *  The sid attribute is a text string value containing the sub-identifier
 * of this element.  This value must be unique within the scope of the parent
 * element. Optional attribute. 
 */
			xsNCName attrSid;

		protected:  // Elements
			domFx_annotate_common_Array elemAnnotate_array;
			domColor_targetRef elemColor_target;
			domDepth_targetRef elemDepth_target;
			domStencil_targetRef elemStencil_target;
			domColor_clearRef elemColor_clear;
			domDepth_clearRef elemDepth_clear;
			domStencil_clearRef elemStencil_clear;
			domDrawRef elemDraw;
			domGles_pipeline_settings_Array elemGles_pipeline_settings_array;
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
			 * Gets the sid attribute.
			 * @return Returns a xsNCName of the sid attribute.
			 */
			xsNCName getSid() const { return attrSid; }
			/**
			 * Sets the sid attribute.
			 * @param atSid The new value for the sid attribute.
			 */
			void setSid( xsNCName atSid ) { *(daeStringRef*)&attrSid = atSid;		
	 _validAttributeArray[0] = true; }

			/**
			 * Gets the annotate element array.
			 * @return Returns a reference to the array of annotate elements.
			 */
			domFx_annotate_common_Array &getAnnotate_array() { return elemAnnotate_array; }
			/**
			 * Gets the annotate element array.
			 * @return Returns a constant reference to the array of annotate elements.
			 */
			const domFx_annotate_common_Array &getAnnotate_array() const { return elemAnnotate_array; }
			/**
			 * Gets the color_target element.
			 * @return a daeSmartRef to the color_target element.
			 */
			const domColor_targetRef getColor_target() const { return elemColor_target; }
			/**
			 * Gets the depth_target element.
			 * @return a daeSmartRef to the depth_target element.
			 */
			const domDepth_targetRef getDepth_target() const { return elemDepth_target; }
			/**
			 * Gets the stencil_target element.
			 * @return a daeSmartRef to the stencil_target element.
			 */
			const domStencil_targetRef getStencil_target() const { return elemStencil_target; }
			/**
			 * Gets the color_clear element.
			 * @return a daeSmartRef to the color_clear element.
			 */
			const domColor_clearRef getColor_clear() const { return elemColor_clear; }
			/**
			 * Gets the depth_clear element.
			 * @return a daeSmartRef to the depth_clear element.
			 */
			const domDepth_clearRef getDepth_clear() const { return elemDepth_clear; }
			/**
			 * Gets the stencil_clear element.
			 * @return a daeSmartRef to the stencil_clear element.
			 */
			const domStencil_clearRef getStencil_clear() const { return elemStencil_clear; }
			/**
			 * Gets the draw element.
			 * @return a daeSmartRef to the draw element.
			 */
			const domDrawRef getDraw() const { return elemDraw; }
			/**
			 * Gets the gles_pipeline_settings element array.
			 * @return Returns a reference to the array of gles_pipeline_settings elements.
			 */
			domGles_pipeline_settings_Array &getGles_pipeline_settings_array() { return elemGles_pipeline_settings_array; }
			/**
			 * Gets the gles_pipeline_settings element array.
			 * @return Returns a constant reference to the array of gles_pipeline_settings elements.
			 */
			const domGles_pipeline_settings_Array &getGles_pipeline_settings_array() const { return elemGles_pipeline_settings_array; }
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
			domPass() : attrSid(), elemAnnotate_array(), elemColor_target(), elemDepth_target(), elemStencil_target(), elemColor_clear(), elemDepth_clear(), elemStencil_clear(), elemDraw(), elemGles_pipeline_settings_array(), elemExtra_array() {}
			/**
			 * Destructor
			 */
			virtual ~domPass() {}
			/**
			 * Copy Constructor
			 */
			domPass( const domPass &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domPass &operator=( const domPass &cpy ) { (void)cpy; return *this; }

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
		xsID attrId;
/**
 *  The sid attribute is a text string value containing the sub-identifier
 * of this element.  This value must be unique within the scope of the parent
 * element. 
 */
		xsNCName attrSid;

	protected:  // Elements
		domAssetRef elemAsset;
		domFx_annotate_common_Array elemAnnotate_array;
		domImage_Array elemImage_array;
		domGles_newparam_Array elemNewparam_array;
		domSetparam_Array elemSetparam_array;
/**
 * A static declaration of all the render states, shaders, and settings for
 * one rendering pipeline. @see domPass
 */
		domPass_Array elemPass_array;
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
		 * Gets the annotate element array.
		 * @return Returns a reference to the array of annotate elements.
		 */
		domFx_annotate_common_Array &getAnnotate_array() { return elemAnnotate_array; }
		/**
		 * Gets the annotate element array.
		 * @return Returns a constant reference to the array of annotate elements.
		 */
		const domFx_annotate_common_Array &getAnnotate_array() const { return elemAnnotate_array; }
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
		domGles_newparam_Array &getNewparam_array() { return elemNewparam_array; }
		/**
		 * Gets the newparam element array.
		 * @return Returns a constant reference to the array of newparam elements.
		 */
		const domGles_newparam_Array &getNewparam_array() const { return elemNewparam_array; }
		/**
		 * Gets the setparam element array.
		 * @return Returns a reference to the array of setparam elements.
		 */
		domSetparam_Array &getSetparam_array() { return elemSetparam_array; }
		/**
		 * Gets the setparam element array.
		 * @return Returns a constant reference to the array of setparam elements.
		 */
		const domSetparam_Array &getSetparam_array() const { return elemSetparam_array; }
		/**
		 * Gets the pass element array.
		 * @return Returns a reference to the array of pass elements.
		 */
		domPass_Array &getPass_array() { return elemPass_array; }
		/**
		 * Gets the pass element array.
		 * @return Returns a constant reference to the array of pass elements.
		 */
		const domPass_Array &getPass_array() const { return elemPass_array; }
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
		domTechnique() : attrId(), attrSid(), elemAsset(), elemAnnotate_array(), elemImage_array(), elemNewparam_array(), elemSetparam_array(), elemPass_array(), elemExtra_array() {}
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


protected:  // Attributes
/**
 *  The id attribute is a text string containing the unique identifier of
 * this element.  This value must be unique within the instance document.
 * Optional attribute. 
 */
	xsID attrId;
/**
 *  The type of platform. This is a vendor-defined character string that indicates
 * the platform or capability target for the technique. Optional 
 */
	xsNCName attrPlatform;

protected:  // Elements
	domAssetRef elemAsset;
	domImage_Array elemImage_array;
	domGles_newparam_Array elemNewparam_array;
/**
 * Holds a description of the textures, samplers, shaders, parameters, and
 * passes necessary for rendering this effect using one method. @see domTechnique
 */
	domTechnique_Array elemTechnique_array;
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
	 * Gets the platform attribute.
	 * @return Returns a xsNCName of the platform attribute.
	 */
	xsNCName getPlatform() const { return attrPlatform; }
	/**
	 * Sets the platform attribute.
	 * @param atPlatform The new value for the platform attribute.
	 */
	void setPlatform( xsNCName atPlatform ) { *(daeStringRef*)&attrPlatform = atPlatform;
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
	domGles_newparam_Array &getNewparam_array() { return elemNewparam_array; }
	/**
	 * Gets the newparam element array.
	 * @return Returns a constant reference to the array of newparam elements.
	 */
	const domGles_newparam_Array &getNewparam_array() const { return elemNewparam_array; }
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
	domProfile_GLES() : attrId(), attrPlatform(), elemAsset(), elemImage_array(), elemNewparam_array(), elemTechnique_array(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domProfile_GLES() {}
	/**
	 * Copy Constructor
	 */
	domProfile_GLES( const domProfile_GLES &cpy ) : domFx_profile_abstract() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domProfile_GLES &operator=( const domProfile_GLES &cpy ) { (void)cpy; return *this; }

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
