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
#ifndef __domGles_pipeline_settings_h__
#define __domGles_pipeline_settings_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domGles_texture_pipeline.h>

/**
 * A group that contains the renderstates available for the GLES profile.
 */
class domGles_pipeline_settings : public daeElement
{
public:
	class domAlpha_func;

	typedef daeSmartRef<domAlpha_func> domAlpha_funcRef;
	typedef daeTArray<domAlpha_funcRef> domAlpha_func_Array;

	class domAlpha_func : public daeElement
	{
	public:
		class domFunc;

		typedef daeSmartRef<domFunc> domFuncRef;
		typedef daeTArray<domFuncRef> domFunc_Array;

		class domFunc : public daeElement
		{
		protected:  // Attributes
			domGl_func_type attrValue;
			xsNCName attrParam;


		public:	//Accessors and Mutators
			/**
			 * Gets the value attribute.
			 * @return Returns a domGl_func_type of the value attribute.
			 */
			domGl_func_type getValue() const { return attrValue; }
			/**
			 * Sets the value attribute.
			 * @param atValue The new value for the value attribute.
			 */
			void setValue( domGl_func_type atValue ) { attrValue = atValue; }

			/**
			 * Gets the param attribute.
			 * @return Returns a xsNCName of the param attribute.
			 */
			xsNCName getParam() const { return attrParam; }
			/**
			 * Sets the param attribute.
			 * @param atParam The new value for the param attribute.
			 */
			void setParam( xsNCName atParam ) { attrParam = atParam; }

		protected:
			/**
			 * Constructor
			 */
			domFunc() : attrValue(), attrParam() {}
			/**
			 * Destructor
			 */
			virtual ~domFunc() {}
			/**
			 * Copy Constructor
			 */
			domFunc( const domFunc &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domFunc &operator=( const domFunc &cpy ) { (void)cpy; return *this; }

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

		class domValue;

		typedef daeSmartRef<domValue> domValueRef;
		typedef daeTArray<domValueRef> domValue_Array;

		class domValue : public daeElement
		{
		protected:  // Attributes
			domGl_alpha_value_type attrValue;
			xsNCName attrParam;


		public:	//Accessors and Mutators
			/**
			 * Gets the value attribute.
			 * @return Returns a domGl_alpha_value_type of the value attribute.
			 */
			domGl_alpha_value_type getValue() const { return attrValue; }
			/**
			 * Sets the value attribute.
			 * @param atValue The new value for the value attribute.
			 */
			void setValue( domGl_alpha_value_type atValue ) { attrValue = atValue; }

			/**
			 * Gets the param attribute.
			 * @return Returns a xsNCName of the param attribute.
			 */
			xsNCName getParam() const { return attrParam; }
			/**
			 * Sets the param attribute.
			 * @param atParam The new value for the param attribute.
			 */
			void setParam( xsNCName atParam ) { attrParam = atParam; }

		protected:
			/**
			 * Constructor
			 */
			domValue() : attrValue(), attrParam() {}
			/**
			 * Destructor
			 */
			virtual ~domValue() {}
			/**
			 * Copy Constructor
			 */
			domValue( const domValue &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domValue &operator=( const domValue &cpy ) { (void)cpy; return *this; }

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
		domFuncRef elemFunc;
		domValueRef elemValue;

	public:	//Accessors and Mutators
		/**
		 * Gets the func element.
		 * @return a daeSmartRef to the func element.
		 */
		const domFuncRef getFunc() const { return elemFunc; }
		/**
		 * Gets the value element.
		 * @return a daeSmartRef to the value element.
		 */
		const domValueRef getValue() const { return elemValue; }
	protected:
		/**
		 * Constructor
		 */
		domAlpha_func() : elemFunc(), elemValue() {}
		/**
		 * Destructor
		 */
		virtual ~domAlpha_func() {}
		/**
		 * Copy Constructor
		 */
		domAlpha_func( const domAlpha_func &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domAlpha_func &operator=( const domAlpha_func &cpy ) { (void)cpy; return *this; }

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

	class domBlend_func;

	typedef daeSmartRef<domBlend_func> domBlend_funcRef;
	typedef daeTArray<domBlend_funcRef> domBlend_func_Array;

	class domBlend_func : public daeElement
	{
	public:
		class domSrc;

		typedef daeSmartRef<domSrc> domSrcRef;
		typedef daeTArray<domSrcRef> domSrc_Array;

		class domSrc : public daeElement
		{
		protected:  // Attributes
			domGl_blend_type attrValue;
			xsNCName attrParam;


		public:	//Accessors and Mutators
			/**
			 * Gets the value attribute.
			 * @return Returns a domGl_blend_type of the value attribute.
			 */
			domGl_blend_type getValue() const { return attrValue; }
			/**
			 * Sets the value attribute.
			 * @param atValue The new value for the value attribute.
			 */
			void setValue( domGl_blend_type atValue ) { attrValue = atValue; }

			/**
			 * Gets the param attribute.
			 * @return Returns a xsNCName of the param attribute.
			 */
			xsNCName getParam() const { return attrParam; }
			/**
			 * Sets the param attribute.
			 * @param atParam The new value for the param attribute.
			 */
			void setParam( xsNCName atParam ) { attrParam = atParam; }

		protected:
			/**
			 * Constructor
			 */
			domSrc() : attrValue(), attrParam() {}
			/**
			 * Destructor
			 */
			virtual ~domSrc() {}
			/**
			 * Copy Constructor
			 */
			domSrc( const domSrc &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domSrc &operator=( const domSrc &cpy ) { (void)cpy; return *this; }

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

		class domDest;

		typedef daeSmartRef<domDest> domDestRef;
		typedef daeTArray<domDestRef> domDest_Array;

		class domDest : public daeElement
		{
		protected:  // Attributes
			domGl_blend_type attrValue;
			xsNCName attrParam;


		public:	//Accessors and Mutators
			/**
			 * Gets the value attribute.
			 * @return Returns a domGl_blend_type of the value attribute.
			 */
			domGl_blend_type getValue() const { return attrValue; }
			/**
			 * Sets the value attribute.
			 * @param atValue The new value for the value attribute.
			 */
			void setValue( domGl_blend_type atValue ) { attrValue = atValue; }

			/**
			 * Gets the param attribute.
			 * @return Returns a xsNCName of the param attribute.
			 */
			xsNCName getParam() const { return attrParam; }
			/**
			 * Sets the param attribute.
			 * @param atParam The new value for the param attribute.
			 */
			void setParam( xsNCName atParam ) { attrParam = atParam; }

		protected:
			/**
			 * Constructor
			 */
			domDest() : attrValue(), attrParam() {}
			/**
			 * Destructor
			 */
			virtual ~domDest() {}
			/**
			 * Copy Constructor
			 */
			domDest( const domDest &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domDest &operator=( const domDest &cpy ) { (void)cpy; return *this; }

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
		domSrcRef elemSrc;
		domDestRef elemDest;

	public:	//Accessors and Mutators
		/**
		 * Gets the src element.
		 * @return a daeSmartRef to the src element.
		 */
		const domSrcRef getSrc() const { return elemSrc; }
		/**
		 * Gets the dest element.
		 * @return a daeSmartRef to the dest element.
		 */
		const domDestRef getDest() const { return elemDest; }
	protected:
		/**
		 * Constructor
		 */
		domBlend_func() : elemSrc(), elemDest() {}
		/**
		 * Destructor
		 */
		virtual ~domBlend_func() {}
		/**
		 * Copy Constructor
		 */
		domBlend_func( const domBlend_func &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBlend_func &operator=( const domBlend_func &cpy ) { (void)cpy; return *this; }

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

	class domClear_color;

	typedef daeSmartRef<domClear_color> domClear_colorRef;
	typedef daeTArray<domClear_colorRef> domClear_color_Array;

	class domClear_color : public daeElement
	{
	protected:  // Attributes
		domFloat4 attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value array attribute.
		 * @return Returns a domFloat4 reference of the value array attribute.
		 */
		domFloat4 &getValue() { return attrValue; }
		/**
		 * Gets the value array attribute.
		 * @return Returns a constant domFloat4 reference of the value array attribute.
		 */
		const domFloat4 &getValue() const { return attrValue; }
		/**
		 * Sets the value array attribute.
		 * @param atValue The new value for the value array attribute.
		 */
		void setValue( const domFloat4 &atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domClear_color() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domClear_color() {}
		/**
		 * Copy Constructor
		 */
		domClear_color( const domClear_color &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domClear_color &operator=( const domClear_color &cpy ) { (void)cpy; return *this; }

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

	class domClear_stencil;

	typedef daeSmartRef<domClear_stencil> domClear_stencilRef;
	typedef daeTArray<domClear_stencilRef> domClear_stencil_Array;

	class domClear_stencil : public daeElement
	{
	protected:  // Attributes
		domInt attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domInt of the value attribute.
		 */
		domInt getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domInt atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domClear_stencil() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domClear_stencil() {}
		/**
		 * Copy Constructor
		 */
		domClear_stencil( const domClear_stencil &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domClear_stencil &operator=( const domClear_stencil &cpy ) { (void)cpy; return *this; }

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

	class domClear_depth;

	typedef daeSmartRef<domClear_depth> domClear_depthRef;
	typedef daeTArray<domClear_depthRef> domClear_depth_Array;

	class domClear_depth : public daeElement
	{
	protected:  // Attributes
		domFloat attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domFloat of the value attribute.
		 */
		domFloat getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domFloat atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domClear_depth() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domClear_depth() {}
		/**
		 * Copy Constructor
		 */
		domClear_depth( const domClear_depth &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domClear_depth &operator=( const domClear_depth &cpy ) { (void)cpy; return *this; }

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

	class domClip_plane;

	typedef daeSmartRef<domClip_plane> domClip_planeRef;
	typedef daeTArray<domClip_planeRef> domClip_plane_Array;

	class domClip_plane : public daeElement
	{
	protected:  // Attributes
		domBool4 attrValue;
		xsNCName attrParam;
		domGLES_MAX_CLIP_PLANES_index attrIndex;


	public:	//Accessors and Mutators
		/**
		 * Gets the value array attribute.
		 * @return Returns a domBool4 reference of the value array attribute.
		 */
		domBool4 &getValue() { return attrValue; }
		/**
		 * Gets the value array attribute.
		 * @return Returns a constant domBool4 reference of the value array attribute.
		 */
		const domBool4 &getValue() const { return attrValue; }
		/**
		 * Sets the value array attribute.
		 * @param atValue The new value for the value array attribute.
		 */
		void setValue( const domBool4 &atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

		/**
		 * Gets the index attribute.
		 * @return Returns a domGLES_MAX_CLIP_PLANES_index of the index attribute.
		 */
		domGLES_MAX_CLIP_PLANES_index getIndex() const { return attrIndex; }
		/**
		 * Sets the index attribute.
		 * @param atIndex The new value for the index attribute.
		 */
		void setIndex( domGLES_MAX_CLIP_PLANES_index atIndex ) { attrIndex = atIndex; }

	protected:
		/**
		 * Constructor
		 */
		domClip_plane() : attrValue(), attrParam(), attrIndex() {}
		/**
		 * Destructor
		 */
		virtual ~domClip_plane() {}
		/**
		 * Copy Constructor
		 */
		domClip_plane( const domClip_plane &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domClip_plane &operator=( const domClip_plane &cpy ) { (void)cpy; return *this; }

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

	class domColor_mask;

	typedef daeSmartRef<domColor_mask> domColor_maskRef;
	typedef daeTArray<domColor_maskRef> domColor_mask_Array;

	class domColor_mask : public daeElement
	{
	protected:  // Attributes
		domBool4 attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value array attribute.
		 * @return Returns a domBool4 reference of the value array attribute.
		 */
		domBool4 &getValue() { return attrValue; }
		/**
		 * Gets the value array attribute.
		 * @return Returns a constant domBool4 reference of the value array attribute.
		 */
		const domBool4 &getValue() const { return attrValue; }
		/**
		 * Sets the value array attribute.
		 * @param atValue The new value for the value array attribute.
		 */
		void setValue( const domBool4 &atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domColor_mask() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domColor_mask() {}
		/**
		 * Copy Constructor
		 */
		domColor_mask( const domColor_mask &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domColor_mask &operator=( const domColor_mask &cpy ) { (void)cpy; return *this; }

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

	class domCull_face;

	typedef daeSmartRef<domCull_face> domCull_faceRef;
	typedef daeTArray<domCull_faceRef> domCull_face_Array;

	class domCull_face : public daeElement
	{
	protected:  // Attributes
		domGl_face_type attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domGl_face_type of the value attribute.
		 */
		domGl_face_type getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domGl_face_type atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domCull_face() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domCull_face() {}
		/**
		 * Copy Constructor
		 */
		domCull_face( const domCull_face &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domCull_face &operator=( const domCull_face &cpy ) { (void)cpy; return *this; }

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

	class domDepth_func;

	typedef daeSmartRef<domDepth_func> domDepth_funcRef;
	typedef daeTArray<domDepth_funcRef> domDepth_func_Array;

	class domDepth_func : public daeElement
	{
	protected:  // Attributes
		domGl_func_type attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domGl_func_type of the value attribute.
		 */
		domGl_func_type getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domGl_func_type atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domDepth_func() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domDepth_func() {}
		/**
		 * Copy Constructor
		 */
		domDepth_func( const domDepth_func &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domDepth_func &operator=( const domDepth_func &cpy ) { (void)cpy; return *this; }

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

	class domDepth_mask;

	typedef daeSmartRef<domDepth_mask> domDepth_maskRef;
	typedef daeTArray<domDepth_maskRef> domDepth_mask_Array;

	class domDepth_mask : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domDepth_mask() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domDepth_mask() {}
		/**
		 * Copy Constructor
		 */
		domDepth_mask( const domDepth_mask &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domDepth_mask &operator=( const domDepth_mask &cpy ) { (void)cpy; return *this; }

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

	class domDepth_range;

	typedef daeSmartRef<domDepth_range> domDepth_rangeRef;
	typedef daeTArray<domDepth_rangeRef> domDepth_range_Array;

	class domDepth_range : public daeElement
	{
	protected:  // Attributes
		domFloat2 attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value array attribute.
		 * @return Returns a domFloat2 reference of the value array attribute.
		 */
		domFloat2 &getValue() { return attrValue; }
		/**
		 * Gets the value array attribute.
		 * @return Returns a constant domFloat2 reference of the value array attribute.
		 */
		const domFloat2 &getValue() const { return attrValue; }
		/**
		 * Sets the value array attribute.
		 * @param atValue The new value for the value array attribute.
		 */
		void setValue( const domFloat2 &atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domDepth_range() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domDepth_range() {}
		/**
		 * Copy Constructor
		 */
		domDepth_range( const domDepth_range &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domDepth_range &operator=( const domDepth_range &cpy ) { (void)cpy; return *this; }

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

	class domFog_color;

	typedef daeSmartRef<domFog_color> domFog_colorRef;
	typedef daeTArray<domFog_colorRef> domFog_color_Array;

	class domFog_color : public daeElement
	{
	protected:  // Attributes
		domFloat4 attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value array attribute.
		 * @return Returns a domFloat4 reference of the value array attribute.
		 */
		domFloat4 &getValue() { return attrValue; }
		/**
		 * Gets the value array attribute.
		 * @return Returns a constant domFloat4 reference of the value array attribute.
		 */
		const domFloat4 &getValue() const { return attrValue; }
		/**
		 * Sets the value array attribute.
		 * @param atValue The new value for the value array attribute.
		 */
		void setValue( const domFloat4 &atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domFog_color() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domFog_color() {}
		/**
		 * Copy Constructor
		 */
		domFog_color( const domFog_color &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFog_color &operator=( const domFog_color &cpy ) { (void)cpy; return *this; }

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

	class domFog_density;

	typedef daeSmartRef<domFog_density> domFog_densityRef;
	typedef daeTArray<domFog_densityRef> domFog_density_Array;

	class domFog_density : public daeElement
	{
	protected:  // Attributes
		domFloat attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domFloat of the value attribute.
		 */
		domFloat getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domFloat atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domFog_density() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domFog_density() {}
		/**
		 * Copy Constructor
		 */
		domFog_density( const domFog_density &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFog_density &operator=( const domFog_density &cpy ) { (void)cpy; return *this; }

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

	class domFog_mode;

	typedef daeSmartRef<domFog_mode> domFog_modeRef;
	typedef daeTArray<domFog_modeRef> domFog_mode_Array;

	class domFog_mode : public daeElement
	{
	protected:  // Attributes
		domGl_fog_type attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domGl_fog_type of the value attribute.
		 */
		domGl_fog_type getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domGl_fog_type atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domFog_mode() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domFog_mode() {}
		/**
		 * Copy Constructor
		 */
		domFog_mode( const domFog_mode &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFog_mode &operator=( const domFog_mode &cpy ) { (void)cpy; return *this; }

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

	class domFog_start;

	typedef daeSmartRef<domFog_start> domFog_startRef;
	typedef daeTArray<domFog_startRef> domFog_start_Array;

	class domFog_start : public daeElement
	{
	protected:  // Attributes
		domFloat attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domFloat of the value attribute.
		 */
		domFloat getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domFloat atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domFog_start() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domFog_start() {}
		/**
		 * Copy Constructor
		 */
		domFog_start( const domFog_start &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFog_start &operator=( const domFog_start &cpy ) { (void)cpy; return *this; }

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

	class domFog_end;

	typedef daeSmartRef<domFog_end> domFog_endRef;
	typedef daeTArray<domFog_endRef> domFog_end_Array;

	class domFog_end : public daeElement
	{
	protected:  // Attributes
		domFloat attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domFloat of the value attribute.
		 */
		domFloat getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domFloat atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domFog_end() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domFog_end() {}
		/**
		 * Copy Constructor
		 */
		domFog_end( const domFog_end &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFog_end &operator=( const domFog_end &cpy ) { (void)cpy; return *this; }

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

	class domFront_face;

	typedef daeSmartRef<domFront_face> domFront_faceRef;
	typedef daeTArray<domFront_faceRef> domFront_face_Array;

	class domFront_face : public daeElement
	{
	protected:  // Attributes
		domGl_front_face_type attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domGl_front_face_type of the value attribute.
		 */
		domGl_front_face_type getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domGl_front_face_type atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domFront_face() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domFront_face() {}
		/**
		 * Copy Constructor
		 */
		domFront_face( const domFront_face &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFront_face &operator=( const domFront_face &cpy ) { (void)cpy; return *this; }

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

	class domTexture_pipeline;

	typedef daeSmartRef<domTexture_pipeline> domTexture_pipelineRef;
	typedef daeTArray<domTexture_pipelineRef> domTexture_pipeline_Array;

	class domTexture_pipeline : public daeElement
	{
	protected:  // Attribute
		xsNCName attrParam;

	protected:  // Element
		domGles_texture_pipelineRef elemValue;

	public:	//Accessors and Mutators
		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

		/**
		 * Gets the value element.
		 * @return a daeSmartRef to the value element.
		 */
		const domGles_texture_pipelineRef getValue() const { return elemValue; }
	protected:
		/**
		 * Constructor
		 */
		domTexture_pipeline() : attrParam(), elemValue() {}
		/**
		 * Destructor
		 */
		virtual ~domTexture_pipeline() {}
		/**
		 * Copy Constructor
		 */
		domTexture_pipeline( const domTexture_pipeline &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domTexture_pipeline &operator=( const domTexture_pipeline &cpy ) { (void)cpy; return *this; }

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

	class domLogic_op;

	typedef daeSmartRef<domLogic_op> domLogic_opRef;
	typedef daeTArray<domLogic_opRef> domLogic_op_Array;

	class domLogic_op : public daeElement
	{
	protected:  // Attributes
		domGl_logic_op_type attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domGl_logic_op_type of the value attribute.
		 */
		domGl_logic_op_type getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domGl_logic_op_type atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domLogic_op() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domLogic_op() {}
		/**
		 * Copy Constructor
		 */
		domLogic_op( const domLogic_op &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domLogic_op &operator=( const domLogic_op &cpy ) { (void)cpy; return *this; }

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

	class domLight_ambient;

	typedef daeSmartRef<domLight_ambient> domLight_ambientRef;
	typedef daeTArray<domLight_ambientRef> domLight_ambient_Array;

	class domLight_ambient : public daeElement
	{
	protected:  // Attributes
		domFloat4 attrValue;
		xsNCName attrParam;
		domGLES_MAX_LIGHTS_index attrIndex;


	public:	//Accessors and Mutators
		/**
		 * Gets the value array attribute.
		 * @return Returns a domFloat4 reference of the value array attribute.
		 */
		domFloat4 &getValue() { return attrValue; }
		/**
		 * Gets the value array attribute.
		 * @return Returns a constant domFloat4 reference of the value array attribute.
		 */
		const domFloat4 &getValue() const { return attrValue; }
		/**
		 * Sets the value array attribute.
		 * @param atValue The new value for the value array attribute.
		 */
		void setValue( const domFloat4 &atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

		/**
		 * Gets the index attribute.
		 * @return Returns a domGLES_MAX_LIGHTS_index of the index attribute.
		 */
		domGLES_MAX_LIGHTS_index getIndex() const { return attrIndex; }
		/**
		 * Sets the index attribute.
		 * @param atIndex The new value for the index attribute.
		 */
		void setIndex( domGLES_MAX_LIGHTS_index atIndex ) { attrIndex = atIndex; }

	protected:
		/**
		 * Constructor
		 */
		domLight_ambient() : attrValue(), attrParam(), attrIndex() {}
		/**
		 * Destructor
		 */
		virtual ~domLight_ambient() {}
		/**
		 * Copy Constructor
		 */
		domLight_ambient( const domLight_ambient &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domLight_ambient &operator=( const domLight_ambient &cpy ) { (void)cpy; return *this; }

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

	class domLight_diffuse;

	typedef daeSmartRef<domLight_diffuse> domLight_diffuseRef;
	typedef daeTArray<domLight_diffuseRef> domLight_diffuse_Array;

	class domLight_diffuse : public daeElement
	{
	protected:  // Attributes
		domFloat4 attrValue;
		xsNCName attrParam;
		domGLES_MAX_LIGHTS_index attrIndex;


	public:	//Accessors and Mutators
		/**
		 * Gets the value array attribute.
		 * @return Returns a domFloat4 reference of the value array attribute.
		 */
		domFloat4 &getValue() { return attrValue; }
		/**
		 * Gets the value array attribute.
		 * @return Returns a constant domFloat4 reference of the value array attribute.
		 */
		const domFloat4 &getValue() const { return attrValue; }
		/**
		 * Sets the value array attribute.
		 * @param atValue The new value for the value array attribute.
		 */
		void setValue( const domFloat4 &atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

		/**
		 * Gets the index attribute.
		 * @return Returns a domGLES_MAX_LIGHTS_index of the index attribute.
		 */
		domGLES_MAX_LIGHTS_index getIndex() const { return attrIndex; }
		/**
		 * Sets the index attribute.
		 * @param atIndex The new value for the index attribute.
		 */
		void setIndex( domGLES_MAX_LIGHTS_index atIndex ) { attrIndex = atIndex; }

	protected:
		/**
		 * Constructor
		 */
		domLight_diffuse() : attrValue(), attrParam(), attrIndex() {}
		/**
		 * Destructor
		 */
		virtual ~domLight_diffuse() {}
		/**
		 * Copy Constructor
		 */
		domLight_diffuse( const domLight_diffuse &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domLight_diffuse &operator=( const domLight_diffuse &cpy ) { (void)cpy; return *this; }

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

	class domLight_specular;

	typedef daeSmartRef<domLight_specular> domLight_specularRef;
	typedef daeTArray<domLight_specularRef> domLight_specular_Array;

	class domLight_specular : public daeElement
	{
	protected:  // Attributes
		domFloat4 attrValue;
		xsNCName attrParam;
		domGLES_MAX_LIGHTS_index attrIndex;


	public:	//Accessors and Mutators
		/**
		 * Gets the value array attribute.
		 * @return Returns a domFloat4 reference of the value array attribute.
		 */
		domFloat4 &getValue() { return attrValue; }
		/**
		 * Gets the value array attribute.
		 * @return Returns a constant domFloat4 reference of the value array attribute.
		 */
		const domFloat4 &getValue() const { return attrValue; }
		/**
		 * Sets the value array attribute.
		 * @param atValue The new value for the value array attribute.
		 */
		void setValue( const domFloat4 &atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

		/**
		 * Gets the index attribute.
		 * @return Returns a domGLES_MAX_LIGHTS_index of the index attribute.
		 */
		domGLES_MAX_LIGHTS_index getIndex() const { return attrIndex; }
		/**
		 * Sets the index attribute.
		 * @param atIndex The new value for the index attribute.
		 */
		void setIndex( domGLES_MAX_LIGHTS_index atIndex ) { attrIndex = atIndex; }

	protected:
		/**
		 * Constructor
		 */
		domLight_specular() : attrValue(), attrParam(), attrIndex() {}
		/**
		 * Destructor
		 */
		virtual ~domLight_specular() {}
		/**
		 * Copy Constructor
		 */
		domLight_specular( const domLight_specular &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domLight_specular &operator=( const domLight_specular &cpy ) { (void)cpy; return *this; }

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

	class domLight_position;

	typedef daeSmartRef<domLight_position> domLight_positionRef;
	typedef daeTArray<domLight_positionRef> domLight_position_Array;

	class domLight_position : public daeElement
	{
	protected:  // Attributes
		domFloat4 attrValue;
		xsNCName attrParam;
		domGLES_MAX_LIGHTS_index attrIndex;


	public:	//Accessors and Mutators
		/**
		 * Gets the value array attribute.
		 * @return Returns a domFloat4 reference of the value array attribute.
		 */
		domFloat4 &getValue() { return attrValue; }
		/**
		 * Gets the value array attribute.
		 * @return Returns a constant domFloat4 reference of the value array attribute.
		 */
		const domFloat4 &getValue() const { return attrValue; }
		/**
		 * Sets the value array attribute.
		 * @param atValue The new value for the value array attribute.
		 */
		void setValue( const domFloat4 &atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

		/**
		 * Gets the index attribute.
		 * @return Returns a domGLES_MAX_LIGHTS_index of the index attribute.
		 */
		domGLES_MAX_LIGHTS_index getIndex() const { return attrIndex; }
		/**
		 * Sets the index attribute.
		 * @param atIndex The new value for the index attribute.
		 */
		void setIndex( domGLES_MAX_LIGHTS_index atIndex ) { attrIndex = atIndex; }

	protected:
		/**
		 * Constructor
		 */
		domLight_position() : attrValue(), attrParam(), attrIndex() {}
		/**
		 * Destructor
		 */
		virtual ~domLight_position() {}
		/**
		 * Copy Constructor
		 */
		domLight_position( const domLight_position &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domLight_position &operator=( const domLight_position &cpy ) { (void)cpy; return *this; }

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

	class domLight_constant_attenuation;

	typedef daeSmartRef<domLight_constant_attenuation> domLight_constant_attenuationRef;
	typedef daeTArray<domLight_constant_attenuationRef> domLight_constant_attenuation_Array;

	class domLight_constant_attenuation : public daeElement
	{
	protected:  // Attributes
		domFloat attrValue;
		xsNCName attrParam;
		domGLES_MAX_LIGHTS_index attrIndex;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domFloat of the value attribute.
		 */
		domFloat getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domFloat atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

		/**
		 * Gets the index attribute.
		 * @return Returns a domGLES_MAX_LIGHTS_index of the index attribute.
		 */
		domGLES_MAX_LIGHTS_index getIndex() const { return attrIndex; }
		/**
		 * Sets the index attribute.
		 * @param atIndex The new value for the index attribute.
		 */
		void setIndex( domGLES_MAX_LIGHTS_index atIndex ) { attrIndex = atIndex; }

	protected:
		/**
		 * Constructor
		 */
		domLight_constant_attenuation() : attrValue(), attrParam(), attrIndex() {}
		/**
		 * Destructor
		 */
		virtual ~domLight_constant_attenuation() {}
		/**
		 * Copy Constructor
		 */
		domLight_constant_attenuation( const domLight_constant_attenuation &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domLight_constant_attenuation &operator=( const domLight_constant_attenuation &cpy ) { (void)cpy; return *this; }

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

	class domLight_linear_attenutation;

	typedef daeSmartRef<domLight_linear_attenutation> domLight_linear_attenutationRef;
	typedef daeTArray<domLight_linear_attenutationRef> domLight_linear_attenutation_Array;

	class domLight_linear_attenutation : public daeElement
	{
	protected:  // Attributes
		domFloat attrValue;
		xsNCName attrParam;
		domGLES_MAX_LIGHTS_index attrIndex;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domFloat of the value attribute.
		 */
		domFloat getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domFloat atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

		/**
		 * Gets the index attribute.
		 * @return Returns a domGLES_MAX_LIGHTS_index of the index attribute.
		 */
		domGLES_MAX_LIGHTS_index getIndex() const { return attrIndex; }
		/**
		 * Sets the index attribute.
		 * @param atIndex The new value for the index attribute.
		 */
		void setIndex( domGLES_MAX_LIGHTS_index atIndex ) { attrIndex = atIndex; }

	protected:
		/**
		 * Constructor
		 */
		domLight_linear_attenutation() : attrValue(), attrParam(), attrIndex() {}
		/**
		 * Destructor
		 */
		virtual ~domLight_linear_attenutation() {}
		/**
		 * Copy Constructor
		 */
		domLight_linear_attenutation( const domLight_linear_attenutation &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domLight_linear_attenutation &operator=( const domLight_linear_attenutation &cpy ) { (void)cpy; return *this; }

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

	class domLight_quadratic_attenuation;

	typedef daeSmartRef<domLight_quadratic_attenuation> domLight_quadratic_attenuationRef;
	typedef daeTArray<domLight_quadratic_attenuationRef> domLight_quadratic_attenuation_Array;

	class domLight_quadratic_attenuation : public daeElement
	{
	protected:  // Attributes
		domFloat attrValue;
		xsNCName attrParam;
		domGLES_MAX_LIGHTS_index attrIndex;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domFloat of the value attribute.
		 */
		domFloat getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domFloat atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

		/**
		 * Gets the index attribute.
		 * @return Returns a domGLES_MAX_LIGHTS_index of the index attribute.
		 */
		domGLES_MAX_LIGHTS_index getIndex() const { return attrIndex; }
		/**
		 * Sets the index attribute.
		 * @param atIndex The new value for the index attribute.
		 */
		void setIndex( domGLES_MAX_LIGHTS_index atIndex ) { attrIndex = atIndex; }

	protected:
		/**
		 * Constructor
		 */
		domLight_quadratic_attenuation() : attrValue(), attrParam(), attrIndex() {}
		/**
		 * Destructor
		 */
		virtual ~domLight_quadratic_attenuation() {}
		/**
		 * Copy Constructor
		 */
		domLight_quadratic_attenuation( const domLight_quadratic_attenuation &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domLight_quadratic_attenuation &operator=( const domLight_quadratic_attenuation &cpy ) { (void)cpy; return *this; }

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

	class domLight_spot_cutoff;

	typedef daeSmartRef<domLight_spot_cutoff> domLight_spot_cutoffRef;
	typedef daeTArray<domLight_spot_cutoffRef> domLight_spot_cutoff_Array;

	class domLight_spot_cutoff : public daeElement
	{
	protected:  // Attributes
		domFloat attrValue;
		xsNCName attrParam;
		domGLES_MAX_LIGHTS_index attrIndex;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domFloat of the value attribute.
		 */
		domFloat getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domFloat atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

		/**
		 * Gets the index attribute.
		 * @return Returns a domGLES_MAX_LIGHTS_index of the index attribute.
		 */
		domGLES_MAX_LIGHTS_index getIndex() const { return attrIndex; }
		/**
		 * Sets the index attribute.
		 * @param atIndex The new value for the index attribute.
		 */
		void setIndex( domGLES_MAX_LIGHTS_index atIndex ) { attrIndex = atIndex; }

	protected:
		/**
		 * Constructor
		 */
		domLight_spot_cutoff() : attrValue(), attrParam(), attrIndex() {}
		/**
		 * Destructor
		 */
		virtual ~domLight_spot_cutoff() {}
		/**
		 * Copy Constructor
		 */
		domLight_spot_cutoff( const domLight_spot_cutoff &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domLight_spot_cutoff &operator=( const domLight_spot_cutoff &cpy ) { (void)cpy; return *this; }

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

	class domLight_spot_direction;

	typedef daeSmartRef<domLight_spot_direction> domLight_spot_directionRef;
	typedef daeTArray<domLight_spot_directionRef> domLight_spot_direction_Array;

	class domLight_spot_direction : public daeElement
	{
	protected:  // Attributes
		domFloat3 attrValue;
		xsNCName attrParam;
		domGLES_MAX_LIGHTS_index attrIndex;


	public:	//Accessors and Mutators
		/**
		 * Gets the value array attribute.
		 * @return Returns a domFloat3 reference of the value array attribute.
		 */
		domFloat3 &getValue() { return attrValue; }
		/**
		 * Gets the value array attribute.
		 * @return Returns a constant domFloat3 reference of the value array attribute.
		 */
		const domFloat3 &getValue() const { return attrValue; }
		/**
		 * Sets the value array attribute.
		 * @param atValue The new value for the value array attribute.
		 */
		void setValue( const domFloat3 &atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

		/**
		 * Gets the index attribute.
		 * @return Returns a domGLES_MAX_LIGHTS_index of the index attribute.
		 */
		domGLES_MAX_LIGHTS_index getIndex() const { return attrIndex; }
		/**
		 * Sets the index attribute.
		 * @param atIndex The new value for the index attribute.
		 */
		void setIndex( domGLES_MAX_LIGHTS_index atIndex ) { attrIndex = atIndex; }

	protected:
		/**
		 * Constructor
		 */
		domLight_spot_direction() : attrValue(), attrParam(), attrIndex() {}
		/**
		 * Destructor
		 */
		virtual ~domLight_spot_direction() {}
		/**
		 * Copy Constructor
		 */
		domLight_spot_direction( const domLight_spot_direction &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domLight_spot_direction &operator=( const domLight_spot_direction &cpy ) { (void)cpy; return *this; }

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

	class domLight_spot_exponent;

	typedef daeSmartRef<domLight_spot_exponent> domLight_spot_exponentRef;
	typedef daeTArray<domLight_spot_exponentRef> domLight_spot_exponent_Array;

	class domLight_spot_exponent : public daeElement
	{
	protected:  // Attributes
		domFloat attrValue;
		xsNCName attrParam;
		domGLES_MAX_LIGHTS_index attrIndex;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domFloat of the value attribute.
		 */
		domFloat getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domFloat atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

		/**
		 * Gets the index attribute.
		 * @return Returns a domGLES_MAX_LIGHTS_index of the index attribute.
		 */
		domGLES_MAX_LIGHTS_index getIndex() const { return attrIndex; }
		/**
		 * Sets the index attribute.
		 * @param atIndex The new value for the index attribute.
		 */
		void setIndex( domGLES_MAX_LIGHTS_index atIndex ) { attrIndex = atIndex; }

	protected:
		/**
		 * Constructor
		 */
		domLight_spot_exponent() : attrValue(), attrParam(), attrIndex() {}
		/**
		 * Destructor
		 */
		virtual ~domLight_spot_exponent() {}
		/**
		 * Copy Constructor
		 */
		domLight_spot_exponent( const domLight_spot_exponent &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domLight_spot_exponent &operator=( const domLight_spot_exponent &cpy ) { (void)cpy; return *this; }

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

	class domLight_model_ambient;

	typedef daeSmartRef<domLight_model_ambient> domLight_model_ambientRef;
	typedef daeTArray<domLight_model_ambientRef> domLight_model_ambient_Array;

	class domLight_model_ambient : public daeElement
	{
	protected:  // Attributes
		domFloat4 attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value array attribute.
		 * @return Returns a domFloat4 reference of the value array attribute.
		 */
		domFloat4 &getValue() { return attrValue; }
		/**
		 * Gets the value array attribute.
		 * @return Returns a constant domFloat4 reference of the value array attribute.
		 */
		const domFloat4 &getValue() const { return attrValue; }
		/**
		 * Sets the value array attribute.
		 * @param atValue The new value for the value array attribute.
		 */
		void setValue( const domFloat4 &atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domLight_model_ambient() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domLight_model_ambient() {}
		/**
		 * Copy Constructor
		 */
		domLight_model_ambient( const domLight_model_ambient &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domLight_model_ambient &operator=( const domLight_model_ambient &cpy ) { (void)cpy; return *this; }

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

	class domLine_width;

	typedef daeSmartRef<domLine_width> domLine_widthRef;
	typedef daeTArray<domLine_widthRef> domLine_width_Array;

	class domLine_width : public daeElement
	{
	protected:  // Attributes
		domFloat attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domFloat of the value attribute.
		 */
		domFloat getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domFloat atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domLine_width() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domLine_width() {}
		/**
		 * Copy Constructor
		 */
		domLine_width( const domLine_width &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domLine_width &operator=( const domLine_width &cpy ) { (void)cpy; return *this; }

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

	class domMaterial_ambient;

	typedef daeSmartRef<domMaterial_ambient> domMaterial_ambientRef;
	typedef daeTArray<domMaterial_ambientRef> domMaterial_ambient_Array;

	class domMaterial_ambient : public daeElement
	{
	protected:  // Attributes
		domFloat4 attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value array attribute.
		 * @return Returns a domFloat4 reference of the value array attribute.
		 */
		domFloat4 &getValue() { return attrValue; }
		/**
		 * Gets the value array attribute.
		 * @return Returns a constant domFloat4 reference of the value array attribute.
		 */
		const domFloat4 &getValue() const { return attrValue; }
		/**
		 * Sets the value array attribute.
		 * @param atValue The new value for the value array attribute.
		 */
		void setValue( const domFloat4 &atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domMaterial_ambient() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domMaterial_ambient() {}
		/**
		 * Copy Constructor
		 */
		domMaterial_ambient( const domMaterial_ambient &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domMaterial_ambient &operator=( const domMaterial_ambient &cpy ) { (void)cpy; return *this; }

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

	class domMaterial_diffuse;

	typedef daeSmartRef<domMaterial_diffuse> domMaterial_diffuseRef;
	typedef daeTArray<domMaterial_diffuseRef> domMaterial_diffuse_Array;

	class domMaterial_diffuse : public daeElement
	{
	protected:  // Attributes
		domFloat4 attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value array attribute.
		 * @return Returns a domFloat4 reference of the value array attribute.
		 */
		domFloat4 &getValue() { return attrValue; }
		/**
		 * Gets the value array attribute.
		 * @return Returns a constant domFloat4 reference of the value array attribute.
		 */
		const domFloat4 &getValue() const { return attrValue; }
		/**
		 * Sets the value array attribute.
		 * @param atValue The new value for the value array attribute.
		 */
		void setValue( const domFloat4 &atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domMaterial_diffuse() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domMaterial_diffuse() {}
		/**
		 * Copy Constructor
		 */
		domMaterial_diffuse( const domMaterial_diffuse &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domMaterial_diffuse &operator=( const domMaterial_diffuse &cpy ) { (void)cpy; return *this; }

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

	class domMaterial_emission;

	typedef daeSmartRef<domMaterial_emission> domMaterial_emissionRef;
	typedef daeTArray<domMaterial_emissionRef> domMaterial_emission_Array;

	class domMaterial_emission : public daeElement
	{
	protected:  // Attributes
		domFloat4 attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value array attribute.
		 * @return Returns a domFloat4 reference of the value array attribute.
		 */
		domFloat4 &getValue() { return attrValue; }
		/**
		 * Gets the value array attribute.
		 * @return Returns a constant domFloat4 reference of the value array attribute.
		 */
		const domFloat4 &getValue() const { return attrValue; }
		/**
		 * Sets the value array attribute.
		 * @param atValue The new value for the value array attribute.
		 */
		void setValue( const domFloat4 &atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domMaterial_emission() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domMaterial_emission() {}
		/**
		 * Copy Constructor
		 */
		domMaterial_emission( const domMaterial_emission &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domMaterial_emission &operator=( const domMaterial_emission &cpy ) { (void)cpy; return *this; }

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

	class domMaterial_shininess;

	typedef daeSmartRef<domMaterial_shininess> domMaterial_shininessRef;
	typedef daeTArray<domMaterial_shininessRef> domMaterial_shininess_Array;

	class domMaterial_shininess : public daeElement
	{
	protected:  // Attributes
		domFloat attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domFloat of the value attribute.
		 */
		domFloat getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domFloat atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domMaterial_shininess() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domMaterial_shininess() {}
		/**
		 * Copy Constructor
		 */
		domMaterial_shininess( const domMaterial_shininess &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domMaterial_shininess &operator=( const domMaterial_shininess &cpy ) { (void)cpy; return *this; }

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

	class domMaterial_specular;

	typedef daeSmartRef<domMaterial_specular> domMaterial_specularRef;
	typedef daeTArray<domMaterial_specularRef> domMaterial_specular_Array;

	class domMaterial_specular : public daeElement
	{
	protected:  // Attributes
		domFloat4 attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value array attribute.
		 * @return Returns a domFloat4 reference of the value array attribute.
		 */
		domFloat4 &getValue() { return attrValue; }
		/**
		 * Gets the value array attribute.
		 * @return Returns a constant domFloat4 reference of the value array attribute.
		 */
		const domFloat4 &getValue() const { return attrValue; }
		/**
		 * Sets the value array attribute.
		 * @param atValue The new value for the value array attribute.
		 */
		void setValue( const domFloat4 &atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domMaterial_specular() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domMaterial_specular() {}
		/**
		 * Copy Constructor
		 */
		domMaterial_specular( const domMaterial_specular &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domMaterial_specular &operator=( const domMaterial_specular &cpy ) { (void)cpy; return *this; }

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

	class domModel_view_matrix;

	typedef daeSmartRef<domModel_view_matrix> domModel_view_matrixRef;
	typedef daeTArray<domModel_view_matrixRef> domModel_view_matrix_Array;

	class domModel_view_matrix : public daeElement
	{
	protected:  // Attributes
		domFloat4x4 attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value array attribute.
		 * @return Returns a domFloat4x4 reference of the value array attribute.
		 */
		domFloat4x4 &getValue() { return attrValue; }
		/**
		 * Gets the value array attribute.
		 * @return Returns a constant domFloat4x4 reference of the value array attribute.
		 */
		const domFloat4x4 &getValue() const { return attrValue; }
		/**
		 * Sets the value array attribute.
		 * @param atValue The new value for the value array attribute.
		 */
		void setValue( const domFloat4x4 &atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domModel_view_matrix() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domModel_view_matrix() {}
		/**
		 * Copy Constructor
		 */
		domModel_view_matrix( const domModel_view_matrix &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domModel_view_matrix &operator=( const domModel_view_matrix &cpy ) { (void)cpy; return *this; }

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

	class domPoint_distance_attenuation;

	typedef daeSmartRef<domPoint_distance_attenuation> domPoint_distance_attenuationRef;
	typedef daeTArray<domPoint_distance_attenuationRef> domPoint_distance_attenuation_Array;

	class domPoint_distance_attenuation : public daeElement
	{
	protected:  // Attributes
		domFloat3 attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value array attribute.
		 * @return Returns a domFloat3 reference of the value array attribute.
		 */
		domFloat3 &getValue() { return attrValue; }
		/**
		 * Gets the value array attribute.
		 * @return Returns a constant domFloat3 reference of the value array attribute.
		 */
		const domFloat3 &getValue() const { return attrValue; }
		/**
		 * Sets the value array attribute.
		 * @param atValue The new value for the value array attribute.
		 */
		void setValue( const domFloat3 &atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domPoint_distance_attenuation() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domPoint_distance_attenuation() {}
		/**
		 * Copy Constructor
		 */
		domPoint_distance_attenuation( const domPoint_distance_attenuation &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domPoint_distance_attenuation &operator=( const domPoint_distance_attenuation &cpy ) { (void)cpy; return *this; }

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

	class domPoint_fade_threshold_size;

	typedef daeSmartRef<domPoint_fade_threshold_size> domPoint_fade_threshold_sizeRef;
	typedef daeTArray<domPoint_fade_threshold_sizeRef> domPoint_fade_threshold_size_Array;

	class domPoint_fade_threshold_size : public daeElement
	{
	protected:  // Attributes
		domFloat attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domFloat of the value attribute.
		 */
		domFloat getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domFloat atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domPoint_fade_threshold_size() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domPoint_fade_threshold_size() {}
		/**
		 * Copy Constructor
		 */
		domPoint_fade_threshold_size( const domPoint_fade_threshold_size &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domPoint_fade_threshold_size &operator=( const domPoint_fade_threshold_size &cpy ) { (void)cpy; return *this; }

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

	class domPoint_size;

	typedef daeSmartRef<domPoint_size> domPoint_sizeRef;
	typedef daeTArray<domPoint_sizeRef> domPoint_size_Array;

	class domPoint_size : public daeElement
	{
	protected:  // Attributes
		domFloat attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domFloat of the value attribute.
		 */
		domFloat getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domFloat atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domPoint_size() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domPoint_size() {}
		/**
		 * Copy Constructor
		 */
		domPoint_size( const domPoint_size &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domPoint_size &operator=( const domPoint_size &cpy ) { (void)cpy; return *this; }

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

	class domPoint_size_min;

	typedef daeSmartRef<domPoint_size_min> domPoint_size_minRef;
	typedef daeTArray<domPoint_size_minRef> domPoint_size_min_Array;

	class domPoint_size_min : public daeElement
	{
	protected:  // Attributes
		domFloat attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domFloat of the value attribute.
		 */
		domFloat getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domFloat atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domPoint_size_min() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domPoint_size_min() {}
		/**
		 * Copy Constructor
		 */
		domPoint_size_min( const domPoint_size_min &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domPoint_size_min &operator=( const domPoint_size_min &cpy ) { (void)cpy; return *this; }

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

	class domPoint_size_max;

	typedef daeSmartRef<domPoint_size_max> domPoint_size_maxRef;
	typedef daeTArray<domPoint_size_maxRef> domPoint_size_max_Array;

	class domPoint_size_max : public daeElement
	{
	protected:  // Attributes
		domFloat attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domFloat of the value attribute.
		 */
		domFloat getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domFloat atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domPoint_size_max() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domPoint_size_max() {}
		/**
		 * Copy Constructor
		 */
		domPoint_size_max( const domPoint_size_max &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domPoint_size_max &operator=( const domPoint_size_max &cpy ) { (void)cpy; return *this; }

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

	class domPolygon_offset;

	typedef daeSmartRef<domPolygon_offset> domPolygon_offsetRef;
	typedef daeTArray<domPolygon_offsetRef> domPolygon_offset_Array;

	class domPolygon_offset : public daeElement
	{
	protected:  // Attributes
		domFloat2 attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value array attribute.
		 * @return Returns a domFloat2 reference of the value array attribute.
		 */
		domFloat2 &getValue() { return attrValue; }
		/**
		 * Gets the value array attribute.
		 * @return Returns a constant domFloat2 reference of the value array attribute.
		 */
		const domFloat2 &getValue() const { return attrValue; }
		/**
		 * Sets the value array attribute.
		 * @param atValue The new value for the value array attribute.
		 */
		void setValue( const domFloat2 &atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domPolygon_offset() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domPolygon_offset() {}
		/**
		 * Copy Constructor
		 */
		domPolygon_offset( const domPolygon_offset &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domPolygon_offset &operator=( const domPolygon_offset &cpy ) { (void)cpy; return *this; }

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

	class domProjection_matrix;

	typedef daeSmartRef<domProjection_matrix> domProjection_matrixRef;
	typedef daeTArray<domProjection_matrixRef> domProjection_matrix_Array;

	class domProjection_matrix : public daeElement
	{
	protected:  // Attributes
		domFloat4x4 attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value array attribute.
		 * @return Returns a domFloat4x4 reference of the value array attribute.
		 */
		domFloat4x4 &getValue() { return attrValue; }
		/**
		 * Gets the value array attribute.
		 * @return Returns a constant domFloat4x4 reference of the value array attribute.
		 */
		const domFloat4x4 &getValue() const { return attrValue; }
		/**
		 * Sets the value array attribute.
		 * @param atValue The new value for the value array attribute.
		 */
		void setValue( const domFloat4x4 &atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domProjection_matrix() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domProjection_matrix() {}
		/**
		 * Copy Constructor
		 */
		domProjection_matrix( const domProjection_matrix &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domProjection_matrix &operator=( const domProjection_matrix &cpy ) { (void)cpy; return *this; }

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

	class domScissor;

	typedef daeSmartRef<domScissor> domScissorRef;
	typedef daeTArray<domScissorRef> domScissor_Array;

	class domScissor : public daeElement
	{
	protected:  // Attributes
		domInt4 attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value array attribute.
		 * @return Returns a domInt4 reference of the value array attribute.
		 */
		domInt4 &getValue() { return attrValue; }
		/**
		 * Gets the value array attribute.
		 * @return Returns a constant domInt4 reference of the value array attribute.
		 */
		const domInt4 &getValue() const { return attrValue; }
		/**
		 * Sets the value array attribute.
		 * @param atValue The new value for the value array attribute.
		 */
		void setValue( const domInt4 &atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domScissor() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domScissor() {}
		/**
		 * Copy Constructor
		 */
		domScissor( const domScissor &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domScissor &operator=( const domScissor &cpy ) { (void)cpy; return *this; }

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

	class domShade_model;

	typedef daeSmartRef<domShade_model> domShade_modelRef;
	typedef daeTArray<domShade_modelRef> domShade_model_Array;

	class domShade_model : public daeElement
	{
	protected:  // Attributes
		domGl_shade_model_type attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domGl_shade_model_type of the value attribute.
		 */
		domGl_shade_model_type getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domGl_shade_model_type atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domShade_model() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domShade_model() {}
		/**
		 * Copy Constructor
		 */
		domShade_model( const domShade_model &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domShade_model &operator=( const domShade_model &cpy ) { (void)cpy; return *this; }

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

	class domStencil_func;

	typedef daeSmartRef<domStencil_func> domStencil_funcRef;
	typedef daeTArray<domStencil_funcRef> domStencil_func_Array;

	class domStencil_func : public daeElement
	{
	public:
		class domFunc;

		typedef daeSmartRef<domFunc> domFuncRef;
		typedef daeTArray<domFuncRef> domFunc_Array;

		class domFunc : public daeElement
		{
		protected:  // Attributes
			domGl_func_type attrValue;
			xsNCName attrParam;


		public:	//Accessors and Mutators
			/**
			 * Gets the value attribute.
			 * @return Returns a domGl_func_type of the value attribute.
			 */
			domGl_func_type getValue() const { return attrValue; }
			/**
			 * Sets the value attribute.
			 * @param atValue The new value for the value attribute.
			 */
			void setValue( domGl_func_type atValue ) { attrValue = atValue; }

			/**
			 * Gets the param attribute.
			 * @return Returns a xsNCName of the param attribute.
			 */
			xsNCName getParam() const { return attrParam; }
			/**
			 * Sets the param attribute.
			 * @param atParam The new value for the param attribute.
			 */
			void setParam( xsNCName atParam ) { attrParam = atParam; }

		protected:
			/**
			 * Constructor
			 */
			domFunc() : attrValue(), attrParam() {}
			/**
			 * Destructor
			 */
			virtual ~domFunc() {}
			/**
			 * Copy Constructor
			 */
			domFunc( const domFunc &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domFunc &operator=( const domFunc &cpy ) { (void)cpy; return *this; }

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

		class domRef;

		typedef daeSmartRef<domRef> domRefRef;
		typedef daeTArray<domRefRef> domRef_Array;

		class domRef : public daeElement
		{
		protected:  // Attributes
			xsUnsignedByte attrValue;
			xsNCName attrParam;


		public:	//Accessors and Mutators
			/**
			 * Gets the value attribute.
			 * @return Returns a xsUnsignedByte of the value attribute.
			 */
			xsUnsignedByte getValue() const { return attrValue; }
			/**
			 * Sets the value attribute.
			 * @param atValue The new value for the value attribute.
			 */
			void setValue( xsUnsignedByte atValue ) { attrValue = atValue; }

			/**
			 * Gets the param attribute.
			 * @return Returns a xsNCName of the param attribute.
			 */
			xsNCName getParam() const { return attrParam; }
			/**
			 * Sets the param attribute.
			 * @param atParam The new value for the param attribute.
			 */
			void setParam( xsNCName atParam ) { attrParam = atParam; }

		protected:
			/**
			 * Constructor
			 */
			domRef() : attrValue(), attrParam() {}
			/**
			 * Destructor
			 */
			virtual ~domRef() {}
			/**
			 * Copy Constructor
			 */
			domRef( const domRef &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domRef &operator=( const domRef &cpy ) { (void)cpy; return *this; }

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

		class domMask;

		typedef daeSmartRef<domMask> domMaskRef;
		typedef daeTArray<domMaskRef> domMask_Array;

		class domMask : public daeElement
		{
		protected:  // Attributes
			xsUnsignedByte attrValue;
			xsNCName attrParam;


		public:	//Accessors and Mutators
			/**
			 * Gets the value attribute.
			 * @return Returns a xsUnsignedByte of the value attribute.
			 */
			xsUnsignedByte getValue() const { return attrValue; }
			/**
			 * Sets the value attribute.
			 * @param atValue The new value for the value attribute.
			 */
			void setValue( xsUnsignedByte atValue ) { attrValue = atValue; }

			/**
			 * Gets the param attribute.
			 * @return Returns a xsNCName of the param attribute.
			 */
			xsNCName getParam() const { return attrParam; }
			/**
			 * Sets the param attribute.
			 * @param atParam The new value for the param attribute.
			 */
			void setParam( xsNCName atParam ) { attrParam = atParam; }

		protected:
			/**
			 * Constructor
			 */
			domMask() : attrValue(), attrParam() {}
			/**
			 * Destructor
			 */
			virtual ~domMask() {}
			/**
			 * Copy Constructor
			 */
			domMask( const domMask &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domMask &operator=( const domMask &cpy ) { (void)cpy; return *this; }

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
		domFuncRef elemFunc;
		domRefRef elemRef;
		domMaskRef elemMask;

	public:	//Accessors and Mutators
		/**
		 * Gets the func element.
		 * @return a daeSmartRef to the func element.
		 */
		const domFuncRef getFunc() const { return elemFunc; }
		/**
		 * Gets the ref element.
		 * @return a daeSmartRef to the ref element.
		 */
		const domRefRef getRef() const { return elemRef; }
		/**
		 * Gets the mask element.
		 * @return a daeSmartRef to the mask element.
		 */
		const domMaskRef getMask() const { return elemMask; }
	protected:
		/**
		 * Constructor
		 */
		domStencil_func() : elemFunc(), elemRef(), elemMask() {}
		/**
		 * Destructor
		 */
		virtual ~domStencil_func() {}
		/**
		 * Copy Constructor
		 */
		domStencil_func( const domStencil_func &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domStencil_func &operator=( const domStencil_func &cpy ) { (void)cpy; return *this; }

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

	class domStencil_mask;

	typedef daeSmartRef<domStencil_mask> domStencil_maskRef;
	typedef daeTArray<domStencil_maskRef> domStencil_mask_Array;

	class domStencil_mask : public daeElement
	{
	protected:  // Attributes
		domInt attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domInt of the value attribute.
		 */
		domInt getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domInt atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domStencil_mask() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domStencil_mask() {}
		/**
		 * Copy Constructor
		 */
		domStencil_mask( const domStencil_mask &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domStencil_mask &operator=( const domStencil_mask &cpy ) { (void)cpy; return *this; }

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

	class domStencil_op;

	typedef daeSmartRef<domStencil_op> domStencil_opRef;
	typedef daeTArray<domStencil_opRef> domStencil_op_Array;

	class domStencil_op : public daeElement
	{
	public:
		class domFail;

		typedef daeSmartRef<domFail> domFailRef;
		typedef daeTArray<domFailRef> domFail_Array;

		class domFail : public daeElement
		{
		protected:  // Attributes
			domGles_stencil_op_type attrValue;
			xsNCName attrParam;


		public:	//Accessors and Mutators
			/**
			 * Gets the value attribute.
			 * @return Returns a domGles_stencil_op_type of the value attribute.
			 */
			domGles_stencil_op_type getValue() const { return attrValue; }
			/**
			 * Sets the value attribute.
			 * @param atValue The new value for the value attribute.
			 */
			void setValue( domGles_stencil_op_type atValue ) { attrValue = atValue; }

			/**
			 * Gets the param attribute.
			 * @return Returns a xsNCName of the param attribute.
			 */
			xsNCName getParam() const { return attrParam; }
			/**
			 * Sets the param attribute.
			 * @param atParam The new value for the param attribute.
			 */
			void setParam( xsNCName atParam ) { attrParam = atParam; }

		protected:
			/**
			 * Constructor
			 */
			domFail() : attrValue(), attrParam() {}
			/**
			 * Destructor
			 */
			virtual ~domFail() {}
			/**
			 * Copy Constructor
			 */
			domFail( const domFail &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domFail &operator=( const domFail &cpy ) { (void)cpy; return *this; }

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

		class domZfail;

		typedef daeSmartRef<domZfail> domZfailRef;
		typedef daeTArray<domZfailRef> domZfail_Array;

		class domZfail : public daeElement
		{
		protected:  // Attributes
			domGles_stencil_op_type attrValue;
			xsNCName attrParam;


		public:	//Accessors and Mutators
			/**
			 * Gets the value attribute.
			 * @return Returns a domGles_stencil_op_type of the value attribute.
			 */
			domGles_stencil_op_type getValue() const { return attrValue; }
			/**
			 * Sets the value attribute.
			 * @param atValue The new value for the value attribute.
			 */
			void setValue( domGles_stencil_op_type atValue ) { attrValue = atValue; }

			/**
			 * Gets the param attribute.
			 * @return Returns a xsNCName of the param attribute.
			 */
			xsNCName getParam() const { return attrParam; }
			/**
			 * Sets the param attribute.
			 * @param atParam The new value for the param attribute.
			 */
			void setParam( xsNCName atParam ) { attrParam = atParam; }

		protected:
			/**
			 * Constructor
			 */
			domZfail() : attrValue(), attrParam() {}
			/**
			 * Destructor
			 */
			virtual ~domZfail() {}
			/**
			 * Copy Constructor
			 */
			domZfail( const domZfail &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domZfail &operator=( const domZfail &cpy ) { (void)cpy; return *this; }

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

		class domZpass;

		typedef daeSmartRef<domZpass> domZpassRef;
		typedef daeTArray<domZpassRef> domZpass_Array;

		class domZpass : public daeElement
		{
		protected:  // Attributes
			domGles_stencil_op_type attrValue;
			xsNCName attrParam;


		public:	//Accessors and Mutators
			/**
			 * Gets the value attribute.
			 * @return Returns a domGles_stencil_op_type of the value attribute.
			 */
			domGles_stencil_op_type getValue() const { return attrValue; }
			/**
			 * Sets the value attribute.
			 * @param atValue The new value for the value attribute.
			 */
			void setValue( domGles_stencil_op_type atValue ) { attrValue = atValue; }

			/**
			 * Gets the param attribute.
			 * @return Returns a xsNCName of the param attribute.
			 */
			xsNCName getParam() const { return attrParam; }
			/**
			 * Sets the param attribute.
			 * @param atParam The new value for the param attribute.
			 */
			void setParam( xsNCName atParam ) { attrParam = atParam; }

		protected:
			/**
			 * Constructor
			 */
			domZpass() : attrValue(), attrParam() {}
			/**
			 * Destructor
			 */
			virtual ~domZpass() {}
			/**
			 * Copy Constructor
			 */
			domZpass( const domZpass &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domZpass &operator=( const domZpass &cpy ) { (void)cpy; return *this; }

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
		domFailRef elemFail;
		domZfailRef elemZfail;
		domZpassRef elemZpass;

	public:	//Accessors and Mutators
		/**
		 * Gets the fail element.
		 * @return a daeSmartRef to the fail element.
		 */
		const domFailRef getFail() const { return elemFail; }
		/**
		 * Gets the zfail element.
		 * @return a daeSmartRef to the zfail element.
		 */
		const domZfailRef getZfail() const { return elemZfail; }
		/**
		 * Gets the zpass element.
		 * @return a daeSmartRef to the zpass element.
		 */
		const domZpassRef getZpass() const { return elemZpass; }
	protected:
		/**
		 * Constructor
		 */
		domStencil_op() : elemFail(), elemZfail(), elemZpass() {}
		/**
		 * Destructor
		 */
		virtual ~domStencil_op() {}
		/**
		 * Copy Constructor
		 */
		domStencil_op( const domStencil_op &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domStencil_op &operator=( const domStencil_op &cpy ) { (void)cpy; return *this; }

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

	class domAlpha_test_enable;

	typedef daeSmartRef<domAlpha_test_enable> domAlpha_test_enableRef;
	typedef daeTArray<domAlpha_test_enableRef> domAlpha_test_enable_Array;

	class domAlpha_test_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domAlpha_test_enable() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domAlpha_test_enable() {}
		/**
		 * Copy Constructor
		 */
		domAlpha_test_enable( const domAlpha_test_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domAlpha_test_enable &operator=( const domAlpha_test_enable &cpy ) { (void)cpy; return *this; }

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

	class domBlend_enable;

	typedef daeSmartRef<domBlend_enable> domBlend_enableRef;
	typedef daeTArray<domBlend_enableRef> domBlend_enable_Array;

	class domBlend_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domBlend_enable() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domBlend_enable() {}
		/**
		 * Copy Constructor
		 */
		domBlend_enable( const domBlend_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBlend_enable &operator=( const domBlend_enable &cpy ) { (void)cpy; return *this; }

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

	class domClip_plane_enable;

	typedef daeSmartRef<domClip_plane_enable> domClip_plane_enableRef;
	typedef daeTArray<domClip_plane_enableRef> domClip_plane_enable_Array;

	class domClip_plane_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;
		domGLES_MAX_CLIP_PLANES_index attrIndex;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

		/**
		 * Gets the index attribute.
		 * @return Returns a domGLES_MAX_CLIP_PLANES_index of the index attribute.
		 */
		domGLES_MAX_CLIP_PLANES_index getIndex() const { return attrIndex; }
		/**
		 * Sets the index attribute.
		 * @param atIndex The new value for the index attribute.
		 */
		void setIndex( domGLES_MAX_CLIP_PLANES_index atIndex ) { attrIndex = atIndex; }

	protected:
		/**
		 * Constructor
		 */
		domClip_plane_enable() : attrValue(), attrParam(), attrIndex() {}
		/**
		 * Destructor
		 */
		virtual ~domClip_plane_enable() {}
		/**
		 * Copy Constructor
		 */
		domClip_plane_enable( const domClip_plane_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domClip_plane_enable &operator=( const domClip_plane_enable &cpy ) { (void)cpy; return *this; }

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

	class domColor_logic_op_enable;

	typedef daeSmartRef<domColor_logic_op_enable> domColor_logic_op_enableRef;
	typedef daeTArray<domColor_logic_op_enableRef> domColor_logic_op_enable_Array;

	class domColor_logic_op_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domColor_logic_op_enable() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domColor_logic_op_enable() {}
		/**
		 * Copy Constructor
		 */
		domColor_logic_op_enable( const domColor_logic_op_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domColor_logic_op_enable &operator=( const domColor_logic_op_enable &cpy ) { (void)cpy; return *this; }

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

	class domColor_material_enable;

	typedef daeSmartRef<domColor_material_enable> domColor_material_enableRef;
	typedef daeTArray<domColor_material_enableRef> domColor_material_enable_Array;

	class domColor_material_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domColor_material_enable() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domColor_material_enable() {}
		/**
		 * Copy Constructor
		 */
		domColor_material_enable( const domColor_material_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domColor_material_enable &operator=( const domColor_material_enable &cpy ) { (void)cpy; return *this; }

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

	class domCull_face_enable;

	typedef daeSmartRef<domCull_face_enable> domCull_face_enableRef;
	typedef daeTArray<domCull_face_enableRef> domCull_face_enable_Array;

	class domCull_face_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domCull_face_enable() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domCull_face_enable() {}
		/**
		 * Copy Constructor
		 */
		domCull_face_enable( const domCull_face_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domCull_face_enable &operator=( const domCull_face_enable &cpy ) { (void)cpy; return *this; }

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

	class domDepth_test_enable;

	typedef daeSmartRef<domDepth_test_enable> domDepth_test_enableRef;
	typedef daeTArray<domDepth_test_enableRef> domDepth_test_enable_Array;

	class domDepth_test_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domDepth_test_enable() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domDepth_test_enable() {}
		/**
		 * Copy Constructor
		 */
		domDepth_test_enable( const domDepth_test_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domDepth_test_enable &operator=( const domDepth_test_enable &cpy ) { (void)cpy; return *this; }

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

	class domDither_enable;

	typedef daeSmartRef<domDither_enable> domDither_enableRef;
	typedef daeTArray<domDither_enableRef> domDither_enable_Array;

	class domDither_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domDither_enable() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domDither_enable() {}
		/**
		 * Copy Constructor
		 */
		domDither_enable( const domDither_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domDither_enable &operator=( const domDither_enable &cpy ) { (void)cpy; return *this; }

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

	class domFog_enable;

	typedef daeSmartRef<domFog_enable> domFog_enableRef;
	typedef daeTArray<domFog_enableRef> domFog_enable_Array;

	class domFog_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domFog_enable() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domFog_enable() {}
		/**
		 * Copy Constructor
		 */
		domFog_enable( const domFog_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domFog_enable &operator=( const domFog_enable &cpy ) { (void)cpy; return *this; }

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

	class domTexture_pipeline_enable;

	typedef daeSmartRef<domTexture_pipeline_enable> domTexture_pipeline_enableRef;
	typedef daeTArray<domTexture_pipeline_enableRef> domTexture_pipeline_enable_Array;

	class domTexture_pipeline_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domTexture_pipeline_enable() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domTexture_pipeline_enable() {}
		/**
		 * Copy Constructor
		 */
		domTexture_pipeline_enable( const domTexture_pipeline_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domTexture_pipeline_enable &operator=( const domTexture_pipeline_enable &cpy ) { (void)cpy; return *this; }

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

	class domLight_enable;

	typedef daeSmartRef<domLight_enable> domLight_enableRef;
	typedef daeTArray<domLight_enableRef> domLight_enable_Array;

	class domLight_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;
		domGLES_MAX_LIGHTS_index attrIndex;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

		/**
		 * Gets the index attribute.
		 * @return Returns a domGLES_MAX_LIGHTS_index of the index attribute.
		 */
		domGLES_MAX_LIGHTS_index getIndex() const { return attrIndex; }
		/**
		 * Sets the index attribute.
		 * @param atIndex The new value for the index attribute.
		 */
		void setIndex( domGLES_MAX_LIGHTS_index atIndex ) { attrIndex = atIndex; }

	protected:
		/**
		 * Constructor
		 */
		domLight_enable() : attrValue(), attrParam(), attrIndex() {}
		/**
		 * Destructor
		 */
		virtual ~domLight_enable() {}
		/**
		 * Copy Constructor
		 */
		domLight_enable( const domLight_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domLight_enable &operator=( const domLight_enable &cpy ) { (void)cpy; return *this; }

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

	class domLighting_enable;

	typedef daeSmartRef<domLighting_enable> domLighting_enableRef;
	typedef daeTArray<domLighting_enableRef> domLighting_enable_Array;

	class domLighting_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domLighting_enable() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domLighting_enable() {}
		/**
		 * Copy Constructor
		 */
		domLighting_enable( const domLighting_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domLighting_enable &operator=( const domLighting_enable &cpy ) { (void)cpy; return *this; }

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

	class domLight_model_two_side_enable;

	typedef daeSmartRef<domLight_model_two_side_enable> domLight_model_two_side_enableRef;
	typedef daeTArray<domLight_model_two_side_enableRef> domLight_model_two_side_enable_Array;

	class domLight_model_two_side_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domLight_model_two_side_enable() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domLight_model_two_side_enable() {}
		/**
		 * Copy Constructor
		 */
		domLight_model_two_side_enable( const domLight_model_two_side_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domLight_model_two_side_enable &operator=( const domLight_model_two_side_enable &cpy ) { (void)cpy; return *this; }

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

	class domLine_smooth_enable;

	typedef daeSmartRef<domLine_smooth_enable> domLine_smooth_enableRef;
	typedef daeTArray<domLine_smooth_enableRef> domLine_smooth_enable_Array;

	class domLine_smooth_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domLine_smooth_enable() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domLine_smooth_enable() {}
		/**
		 * Copy Constructor
		 */
		domLine_smooth_enable( const domLine_smooth_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domLine_smooth_enable &operator=( const domLine_smooth_enable &cpy ) { (void)cpy; return *this; }

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

	class domMultisample_enable;

	typedef daeSmartRef<domMultisample_enable> domMultisample_enableRef;
	typedef daeTArray<domMultisample_enableRef> domMultisample_enable_Array;

	class domMultisample_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domMultisample_enable() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domMultisample_enable() {}
		/**
		 * Copy Constructor
		 */
		domMultisample_enable( const domMultisample_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domMultisample_enable &operator=( const domMultisample_enable &cpy ) { (void)cpy; return *this; }

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

	class domNormalize_enable;

	typedef daeSmartRef<domNormalize_enable> domNormalize_enableRef;
	typedef daeTArray<domNormalize_enableRef> domNormalize_enable_Array;

	class domNormalize_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domNormalize_enable() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domNormalize_enable() {}
		/**
		 * Copy Constructor
		 */
		domNormalize_enable( const domNormalize_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domNormalize_enable &operator=( const domNormalize_enable &cpy ) { (void)cpy; return *this; }

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

	class domPoint_smooth_enable;

	typedef daeSmartRef<domPoint_smooth_enable> domPoint_smooth_enableRef;
	typedef daeTArray<domPoint_smooth_enableRef> domPoint_smooth_enable_Array;

	class domPoint_smooth_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domPoint_smooth_enable() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domPoint_smooth_enable() {}
		/**
		 * Copy Constructor
		 */
		domPoint_smooth_enable( const domPoint_smooth_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domPoint_smooth_enable &operator=( const domPoint_smooth_enable &cpy ) { (void)cpy; return *this; }

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

	class domPolygon_offset_fill_enable;

	typedef daeSmartRef<domPolygon_offset_fill_enable> domPolygon_offset_fill_enableRef;
	typedef daeTArray<domPolygon_offset_fill_enableRef> domPolygon_offset_fill_enable_Array;

	class domPolygon_offset_fill_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domPolygon_offset_fill_enable() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domPolygon_offset_fill_enable() {}
		/**
		 * Copy Constructor
		 */
		domPolygon_offset_fill_enable( const domPolygon_offset_fill_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domPolygon_offset_fill_enable &operator=( const domPolygon_offset_fill_enable &cpy ) { (void)cpy; return *this; }

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

	class domRescale_normal_enable;

	typedef daeSmartRef<domRescale_normal_enable> domRescale_normal_enableRef;
	typedef daeTArray<domRescale_normal_enableRef> domRescale_normal_enable_Array;

	class domRescale_normal_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domRescale_normal_enable() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domRescale_normal_enable() {}
		/**
		 * Copy Constructor
		 */
		domRescale_normal_enable( const domRescale_normal_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domRescale_normal_enable &operator=( const domRescale_normal_enable &cpy ) { (void)cpy; return *this; }

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

	class domSample_alpha_to_coverage_enable;

	typedef daeSmartRef<domSample_alpha_to_coverage_enable> domSample_alpha_to_coverage_enableRef;
	typedef daeTArray<domSample_alpha_to_coverage_enableRef> domSample_alpha_to_coverage_enable_Array;

	class domSample_alpha_to_coverage_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domSample_alpha_to_coverage_enable() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domSample_alpha_to_coverage_enable() {}
		/**
		 * Copy Constructor
		 */
		domSample_alpha_to_coverage_enable( const domSample_alpha_to_coverage_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domSample_alpha_to_coverage_enable &operator=( const domSample_alpha_to_coverage_enable &cpy ) { (void)cpy; return *this; }

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

	class domSample_alpha_to_one_enable;

	typedef daeSmartRef<domSample_alpha_to_one_enable> domSample_alpha_to_one_enableRef;
	typedef daeTArray<domSample_alpha_to_one_enableRef> domSample_alpha_to_one_enable_Array;

	class domSample_alpha_to_one_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domSample_alpha_to_one_enable() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domSample_alpha_to_one_enable() {}
		/**
		 * Copy Constructor
		 */
		domSample_alpha_to_one_enable( const domSample_alpha_to_one_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domSample_alpha_to_one_enable &operator=( const domSample_alpha_to_one_enable &cpy ) { (void)cpy; return *this; }

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

	class domSample_coverage_enable;

	typedef daeSmartRef<domSample_coverage_enable> domSample_coverage_enableRef;
	typedef daeTArray<domSample_coverage_enableRef> domSample_coverage_enable_Array;

	class domSample_coverage_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domSample_coverage_enable() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domSample_coverage_enable() {}
		/**
		 * Copy Constructor
		 */
		domSample_coverage_enable( const domSample_coverage_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domSample_coverage_enable &operator=( const domSample_coverage_enable &cpy ) { (void)cpy; return *this; }

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

	class domScissor_test_enable;

	typedef daeSmartRef<domScissor_test_enable> domScissor_test_enableRef;
	typedef daeTArray<domScissor_test_enableRef> domScissor_test_enable_Array;

	class domScissor_test_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domScissor_test_enable() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domScissor_test_enable() {}
		/**
		 * Copy Constructor
		 */
		domScissor_test_enable( const domScissor_test_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domScissor_test_enable &operator=( const domScissor_test_enable &cpy ) { (void)cpy; return *this; }

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

	class domStencil_test_enable;

	typedef daeSmartRef<domStencil_test_enable> domStencil_test_enableRef;
	typedef daeTArray<domStencil_test_enableRef> domStencil_test_enable_Array;

	class domStencil_test_enable : public daeElement
	{
	protected:  // Attributes
		domBool attrValue;
		xsNCName attrParam;


	public:	//Accessors and Mutators
		/**
		 * Gets the value attribute.
		 * @return Returns a domBool of the value attribute.
		 */
		domBool getValue() const { return attrValue; }
		/**
		 * Sets the value attribute.
		 * @param atValue The new value for the value attribute.
		 */
		void setValue( domBool atValue ) { attrValue = atValue; }

		/**
		 * Gets the param attribute.
		 * @return Returns a xsNCName of the param attribute.
		 */
		xsNCName getParam() const { return attrParam; }
		/**
		 * Sets the param attribute.
		 * @param atParam The new value for the param attribute.
		 */
		void setParam( xsNCName atParam ) { attrParam = atParam; }

	protected:
		/**
		 * Constructor
		 */
		domStencil_test_enable() : attrValue(), attrParam() {}
		/**
		 * Destructor
		 */
		virtual ~domStencil_test_enable() {}
		/**
		 * Copy Constructor
		 */
		domStencil_test_enable( const domStencil_test_enable &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domStencil_test_enable &operator=( const domStencil_test_enable &cpy ) { (void)cpy; return *this; }

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
	domAlpha_funcRef elemAlpha_func;
	domBlend_funcRef elemBlend_func;
	domClear_colorRef elemClear_color;
	domClear_stencilRef elemClear_stencil;
	domClear_depthRef elemClear_depth;
	domClip_planeRef elemClip_plane;
	domColor_maskRef elemColor_mask;
	domCull_faceRef elemCull_face;
	domDepth_funcRef elemDepth_func;
	domDepth_maskRef elemDepth_mask;
	domDepth_rangeRef elemDepth_range;
	domFog_colorRef elemFog_color;
	domFog_densityRef elemFog_density;
	domFog_modeRef elemFog_mode;
	domFog_startRef elemFog_start;
	domFog_endRef elemFog_end;
	domFront_faceRef elemFront_face;
	domTexture_pipelineRef elemTexture_pipeline;
	domLogic_opRef elemLogic_op;
	domLight_ambientRef elemLight_ambient;
	domLight_diffuseRef elemLight_diffuse;
	domLight_specularRef elemLight_specular;
	domLight_positionRef elemLight_position;
	domLight_constant_attenuationRef elemLight_constant_attenuation;
	domLight_linear_attenutationRef elemLight_linear_attenutation;
	domLight_quadratic_attenuationRef elemLight_quadratic_attenuation;
	domLight_spot_cutoffRef elemLight_spot_cutoff;
	domLight_spot_directionRef elemLight_spot_direction;
	domLight_spot_exponentRef elemLight_spot_exponent;
	domLight_model_ambientRef elemLight_model_ambient;
	domLine_widthRef elemLine_width;
	domMaterial_ambientRef elemMaterial_ambient;
	domMaterial_diffuseRef elemMaterial_diffuse;
	domMaterial_emissionRef elemMaterial_emission;
	domMaterial_shininessRef elemMaterial_shininess;
	domMaterial_specularRef elemMaterial_specular;
	domModel_view_matrixRef elemModel_view_matrix;
	domPoint_distance_attenuationRef elemPoint_distance_attenuation;
	domPoint_fade_threshold_sizeRef elemPoint_fade_threshold_size;
	domPoint_sizeRef elemPoint_size;
	domPoint_size_minRef elemPoint_size_min;
	domPoint_size_maxRef elemPoint_size_max;
	domPolygon_offsetRef elemPolygon_offset;
	domProjection_matrixRef elemProjection_matrix;
	domScissorRef elemScissor;
	domShade_modelRef elemShade_model;
	domStencil_funcRef elemStencil_func;
	domStencil_maskRef elemStencil_mask;
	domStencil_opRef elemStencil_op;
	domAlpha_test_enableRef elemAlpha_test_enable;
	domBlend_enableRef elemBlend_enable;
	domClip_plane_enableRef elemClip_plane_enable;
	domColor_logic_op_enableRef elemColor_logic_op_enable;
	domColor_material_enableRef elemColor_material_enable;
	domCull_face_enableRef elemCull_face_enable;
	domDepth_test_enableRef elemDepth_test_enable;
	domDither_enableRef elemDither_enable;
	domFog_enableRef elemFog_enable;
	domTexture_pipeline_enableRef elemTexture_pipeline_enable;
	domLight_enableRef elemLight_enable;
	domLighting_enableRef elemLighting_enable;
	domLight_model_two_side_enableRef elemLight_model_two_side_enable;
	domLine_smooth_enableRef elemLine_smooth_enable;
	domMultisample_enableRef elemMultisample_enable;
	domNormalize_enableRef elemNormalize_enable;
	domPoint_smooth_enableRef elemPoint_smooth_enable;
	domPolygon_offset_fill_enableRef elemPolygon_offset_fill_enable;
	domRescale_normal_enableRef elemRescale_normal_enable;
	domSample_alpha_to_coverage_enableRef elemSample_alpha_to_coverage_enable;
	domSample_alpha_to_one_enableRef elemSample_alpha_to_one_enable;
	domSample_coverage_enableRef elemSample_coverage_enable;
	domScissor_test_enableRef elemScissor_test_enable;
	domStencil_test_enableRef elemStencil_test_enable;
	/**
	 * Used to preserve order in elements that do not specify strict sequencing of sub-elements.
	 */
	daeElementRefArray _contents;


public:	//Accessors and Mutators
	/**
	 * Gets the alpha_func element.
	 * @return a daeSmartRef to the alpha_func element.
	 */
	const domAlpha_funcRef getAlpha_func() const { return elemAlpha_func; }
	/**
	 * Gets the blend_func element.
	 * @return a daeSmartRef to the blend_func element.
	 */
	const domBlend_funcRef getBlend_func() const { return elemBlend_func; }
	/**
	 * Gets the clear_color element.
	 * @return a daeSmartRef to the clear_color element.
	 */
	const domClear_colorRef getClear_color() const { return elemClear_color; }
	/**
	 * Gets the clear_stencil element.
	 * @return a daeSmartRef to the clear_stencil element.
	 */
	const domClear_stencilRef getClear_stencil() const { return elemClear_stencil; }
	/**
	 * Gets the clear_depth element.
	 * @return a daeSmartRef to the clear_depth element.
	 */
	const domClear_depthRef getClear_depth() const { return elemClear_depth; }
	/**
	 * Gets the clip_plane element.
	 * @return a daeSmartRef to the clip_plane element.
	 */
	const domClip_planeRef getClip_plane() const { return elemClip_plane; }
	/**
	 * Gets the color_mask element.
	 * @return a daeSmartRef to the color_mask element.
	 */
	const domColor_maskRef getColor_mask() const { return elemColor_mask; }
	/**
	 * Gets the cull_face element.
	 * @return a daeSmartRef to the cull_face element.
	 */
	const domCull_faceRef getCull_face() const { return elemCull_face; }
	/**
	 * Gets the depth_func element.
	 * @return a daeSmartRef to the depth_func element.
	 */
	const domDepth_funcRef getDepth_func() const { return elemDepth_func; }
	/**
	 * Gets the depth_mask element.
	 * @return a daeSmartRef to the depth_mask element.
	 */
	const domDepth_maskRef getDepth_mask() const { return elemDepth_mask; }
	/**
	 * Gets the depth_range element.
	 * @return a daeSmartRef to the depth_range element.
	 */
	const domDepth_rangeRef getDepth_range() const { return elemDepth_range; }
	/**
	 * Gets the fog_color element.
	 * @return a daeSmartRef to the fog_color element.
	 */
	const domFog_colorRef getFog_color() const { return elemFog_color; }
	/**
	 * Gets the fog_density element.
	 * @return a daeSmartRef to the fog_density element.
	 */
	const domFog_densityRef getFog_density() const { return elemFog_density; }
	/**
	 * Gets the fog_mode element.
	 * @return a daeSmartRef to the fog_mode element.
	 */
	const domFog_modeRef getFog_mode() const { return elemFog_mode; }
	/**
	 * Gets the fog_start element.
	 * @return a daeSmartRef to the fog_start element.
	 */
	const domFog_startRef getFog_start() const { return elemFog_start; }
	/**
	 * Gets the fog_end element.
	 * @return a daeSmartRef to the fog_end element.
	 */
	const domFog_endRef getFog_end() const { return elemFog_end; }
	/**
	 * Gets the front_face element.
	 * @return a daeSmartRef to the front_face element.
	 */
	const domFront_faceRef getFront_face() const { return elemFront_face; }
	/**
	 * Gets the texture_pipeline element.
	 * @return a daeSmartRef to the texture_pipeline element.
	 */
	const domTexture_pipelineRef getTexture_pipeline() const { return elemTexture_pipeline; }
	/**
	 * Gets the logic_op element.
	 * @return a daeSmartRef to the logic_op element.
	 */
	const domLogic_opRef getLogic_op() const { return elemLogic_op; }
	/**
	 * Gets the light_ambient element.
	 * @return a daeSmartRef to the light_ambient element.
	 */
	const domLight_ambientRef getLight_ambient() const { return elemLight_ambient; }
	/**
	 * Gets the light_diffuse element.
	 * @return a daeSmartRef to the light_diffuse element.
	 */
	const domLight_diffuseRef getLight_diffuse() const { return elemLight_diffuse; }
	/**
	 * Gets the light_specular element.
	 * @return a daeSmartRef to the light_specular element.
	 */
	const domLight_specularRef getLight_specular() const { return elemLight_specular; }
	/**
	 * Gets the light_position element.
	 * @return a daeSmartRef to the light_position element.
	 */
	const domLight_positionRef getLight_position() const { return elemLight_position; }
	/**
	 * Gets the light_constant_attenuation element.
	 * @return a daeSmartRef to the light_constant_attenuation element.
	 */
	const domLight_constant_attenuationRef getLight_constant_attenuation() const { return elemLight_constant_attenuation; }
	/**
	 * Gets the light_linear_attenutation element.
	 * @return a daeSmartRef to the light_linear_attenutation element.
	 */
	const domLight_linear_attenutationRef getLight_linear_attenutation() const { return elemLight_linear_attenutation; }
	/**
	 * Gets the light_quadratic_attenuation element.
	 * @return a daeSmartRef to the light_quadratic_attenuation element.
	 */
	const domLight_quadratic_attenuationRef getLight_quadratic_attenuation() const { return elemLight_quadratic_attenuation; }
	/**
	 * Gets the light_spot_cutoff element.
	 * @return a daeSmartRef to the light_spot_cutoff element.
	 */
	const domLight_spot_cutoffRef getLight_spot_cutoff() const { return elemLight_spot_cutoff; }
	/**
	 * Gets the light_spot_direction element.
	 * @return a daeSmartRef to the light_spot_direction element.
	 */
	const domLight_spot_directionRef getLight_spot_direction() const { return elemLight_spot_direction; }
	/**
	 * Gets the light_spot_exponent element.
	 * @return a daeSmartRef to the light_spot_exponent element.
	 */
	const domLight_spot_exponentRef getLight_spot_exponent() const { return elemLight_spot_exponent; }
	/**
	 * Gets the light_model_ambient element.
	 * @return a daeSmartRef to the light_model_ambient element.
	 */
	const domLight_model_ambientRef getLight_model_ambient() const { return elemLight_model_ambient; }
	/**
	 * Gets the line_width element.
	 * @return a daeSmartRef to the line_width element.
	 */
	const domLine_widthRef getLine_width() const { return elemLine_width; }
	/**
	 * Gets the material_ambient element.
	 * @return a daeSmartRef to the material_ambient element.
	 */
	const domMaterial_ambientRef getMaterial_ambient() const { return elemMaterial_ambient; }
	/**
	 * Gets the material_diffuse element.
	 * @return a daeSmartRef to the material_diffuse element.
	 */
	const domMaterial_diffuseRef getMaterial_diffuse() const { return elemMaterial_diffuse; }
	/**
	 * Gets the material_emission element.
	 * @return a daeSmartRef to the material_emission element.
	 */
	const domMaterial_emissionRef getMaterial_emission() const { return elemMaterial_emission; }
	/**
	 * Gets the material_shininess element.
	 * @return a daeSmartRef to the material_shininess element.
	 */
	const domMaterial_shininessRef getMaterial_shininess() const { return elemMaterial_shininess; }
	/**
	 * Gets the material_specular element.
	 * @return a daeSmartRef to the material_specular element.
	 */
	const domMaterial_specularRef getMaterial_specular() const { return elemMaterial_specular; }
	/**
	 * Gets the model_view_matrix element.
	 * @return a daeSmartRef to the model_view_matrix element.
	 */
	const domModel_view_matrixRef getModel_view_matrix() const { return elemModel_view_matrix; }
	/**
	 * Gets the point_distance_attenuation element.
	 * @return a daeSmartRef to the point_distance_attenuation element.
	 */
	const domPoint_distance_attenuationRef getPoint_distance_attenuation() const { return elemPoint_distance_attenuation; }
	/**
	 * Gets the point_fade_threshold_size element.
	 * @return a daeSmartRef to the point_fade_threshold_size element.
	 */
	const domPoint_fade_threshold_sizeRef getPoint_fade_threshold_size() const { return elemPoint_fade_threshold_size; }
	/**
	 * Gets the point_size element.
	 * @return a daeSmartRef to the point_size element.
	 */
	const domPoint_sizeRef getPoint_size() const { return elemPoint_size; }
	/**
	 * Gets the point_size_min element.
	 * @return a daeSmartRef to the point_size_min element.
	 */
	const domPoint_size_minRef getPoint_size_min() const { return elemPoint_size_min; }
	/**
	 * Gets the point_size_max element.
	 * @return a daeSmartRef to the point_size_max element.
	 */
	const domPoint_size_maxRef getPoint_size_max() const { return elemPoint_size_max; }
	/**
	 * Gets the polygon_offset element.
	 * @return a daeSmartRef to the polygon_offset element.
	 */
	const domPolygon_offsetRef getPolygon_offset() const { return elemPolygon_offset; }
	/**
	 * Gets the projection_matrix element.
	 * @return a daeSmartRef to the projection_matrix element.
	 */
	const domProjection_matrixRef getProjection_matrix() const { return elemProjection_matrix; }
	/**
	 * Gets the scissor element.
	 * @return a daeSmartRef to the scissor element.
	 */
	const domScissorRef getScissor() const { return elemScissor; }
	/**
	 * Gets the shade_model element.
	 * @return a daeSmartRef to the shade_model element.
	 */
	const domShade_modelRef getShade_model() const { return elemShade_model; }
	/**
	 * Gets the stencil_func element.
	 * @return a daeSmartRef to the stencil_func element.
	 */
	const domStencil_funcRef getStencil_func() const { return elemStencil_func; }
	/**
	 * Gets the stencil_mask element.
	 * @return a daeSmartRef to the stencil_mask element.
	 */
	const domStencil_maskRef getStencil_mask() const { return elemStencil_mask; }
	/**
	 * Gets the stencil_op element.
	 * @return a daeSmartRef to the stencil_op element.
	 */
	const domStencil_opRef getStencil_op() const { return elemStencil_op; }
	/**
	 * Gets the alpha_test_enable element.
	 * @return a daeSmartRef to the alpha_test_enable element.
	 */
	const domAlpha_test_enableRef getAlpha_test_enable() const { return elemAlpha_test_enable; }
	/**
	 * Gets the blend_enable element.
	 * @return a daeSmartRef to the blend_enable element.
	 */
	const domBlend_enableRef getBlend_enable() const { return elemBlend_enable; }
	/**
	 * Gets the clip_plane_enable element.
	 * @return a daeSmartRef to the clip_plane_enable element.
	 */
	const domClip_plane_enableRef getClip_plane_enable() const { return elemClip_plane_enable; }
	/**
	 * Gets the color_logic_op_enable element.
	 * @return a daeSmartRef to the color_logic_op_enable element.
	 */
	const domColor_logic_op_enableRef getColor_logic_op_enable() const { return elemColor_logic_op_enable; }
	/**
	 * Gets the color_material_enable element.
	 * @return a daeSmartRef to the color_material_enable element.
	 */
	const domColor_material_enableRef getColor_material_enable() const { return elemColor_material_enable; }
	/**
	 * Gets the cull_face_enable element.
	 * @return a daeSmartRef to the cull_face_enable element.
	 */
	const domCull_face_enableRef getCull_face_enable() const { return elemCull_face_enable; }
	/**
	 * Gets the depth_test_enable element.
	 * @return a daeSmartRef to the depth_test_enable element.
	 */
	const domDepth_test_enableRef getDepth_test_enable() const { return elemDepth_test_enable; }
	/**
	 * Gets the dither_enable element.
	 * @return a daeSmartRef to the dither_enable element.
	 */
	const domDither_enableRef getDither_enable() const { return elemDither_enable; }
	/**
	 * Gets the fog_enable element.
	 * @return a daeSmartRef to the fog_enable element.
	 */
	const domFog_enableRef getFog_enable() const { return elemFog_enable; }
	/**
	 * Gets the texture_pipeline_enable element.
	 * @return a daeSmartRef to the texture_pipeline_enable element.
	 */
	const domTexture_pipeline_enableRef getTexture_pipeline_enable() const { return elemTexture_pipeline_enable; }
	/**
	 * Gets the light_enable element.
	 * @return a daeSmartRef to the light_enable element.
	 */
	const domLight_enableRef getLight_enable() const { return elemLight_enable; }
	/**
	 * Gets the lighting_enable element.
	 * @return a daeSmartRef to the lighting_enable element.
	 */
	const domLighting_enableRef getLighting_enable() const { return elemLighting_enable; }
	/**
	 * Gets the light_model_two_side_enable element.
	 * @return a daeSmartRef to the light_model_two_side_enable element.
	 */
	const domLight_model_two_side_enableRef getLight_model_two_side_enable() const { return elemLight_model_two_side_enable; }
	/**
	 * Gets the line_smooth_enable element.
	 * @return a daeSmartRef to the line_smooth_enable element.
	 */
	const domLine_smooth_enableRef getLine_smooth_enable() const { return elemLine_smooth_enable; }
	/**
	 * Gets the multisample_enable element.
	 * @return a daeSmartRef to the multisample_enable element.
	 */
	const domMultisample_enableRef getMultisample_enable() const { return elemMultisample_enable; }
	/**
	 * Gets the normalize_enable element.
	 * @return a daeSmartRef to the normalize_enable element.
	 */
	const domNormalize_enableRef getNormalize_enable() const { return elemNormalize_enable; }
	/**
	 * Gets the point_smooth_enable element.
	 * @return a daeSmartRef to the point_smooth_enable element.
	 */
	const domPoint_smooth_enableRef getPoint_smooth_enable() const { return elemPoint_smooth_enable; }
	/**
	 * Gets the polygon_offset_fill_enable element.
	 * @return a daeSmartRef to the polygon_offset_fill_enable element.
	 */
	const domPolygon_offset_fill_enableRef getPolygon_offset_fill_enable() const { return elemPolygon_offset_fill_enable; }
	/**
	 * Gets the rescale_normal_enable element.
	 * @return a daeSmartRef to the rescale_normal_enable element.
	 */
	const domRescale_normal_enableRef getRescale_normal_enable() const { return elemRescale_normal_enable; }
	/**
	 * Gets the sample_alpha_to_coverage_enable element.
	 * @return a daeSmartRef to the sample_alpha_to_coverage_enable element.
	 */
	const domSample_alpha_to_coverage_enableRef getSample_alpha_to_coverage_enable() const { return elemSample_alpha_to_coverage_enable; }
	/**
	 * Gets the sample_alpha_to_one_enable element.
	 * @return a daeSmartRef to the sample_alpha_to_one_enable element.
	 */
	const domSample_alpha_to_one_enableRef getSample_alpha_to_one_enable() const { return elemSample_alpha_to_one_enable; }
	/**
	 * Gets the sample_coverage_enable element.
	 * @return a daeSmartRef to the sample_coverage_enable element.
	 */
	const domSample_coverage_enableRef getSample_coverage_enable() const { return elemSample_coverage_enable; }
	/**
	 * Gets the scissor_test_enable element.
	 * @return a daeSmartRef to the scissor_test_enable element.
	 */
	const domScissor_test_enableRef getScissor_test_enable() const { return elemScissor_test_enable; }
	/**
	 * Gets the stencil_test_enable element.
	 * @return a daeSmartRef to the stencil_test_enable element.
	 */
	const domStencil_test_enableRef getStencil_test_enable() const { return elemStencil_test_enable; }
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
	domGles_pipeline_settings() : elemAlpha_func(), elemBlend_func(), elemClear_color(), elemClear_stencil(), elemClear_depth(), elemClip_plane(), elemColor_mask(), elemCull_face(), elemDepth_func(), elemDepth_mask(), elemDepth_range(), elemFog_color(), elemFog_density(), elemFog_mode(), elemFog_start(), elemFog_end(), elemFront_face(), elemTexture_pipeline(), elemLogic_op(), elemLight_ambient(), elemLight_diffuse(), elemLight_specular(), elemLight_position(), elemLight_constant_attenuation(), elemLight_linear_attenutation(), elemLight_quadratic_attenuation(), elemLight_spot_cutoff(), elemLight_spot_direction(), elemLight_spot_exponent(), elemLight_model_ambient(), elemLine_width(), elemMaterial_ambient(), elemMaterial_diffuse(), elemMaterial_emission(), elemMaterial_shininess(), elemMaterial_specular(), elemModel_view_matrix(), elemPoint_distance_attenuation(), elemPoint_fade_threshold_size(), elemPoint_size(), elemPoint_size_min(), elemPoint_size_max(), elemPolygon_offset(), elemProjection_matrix(), elemScissor(), elemShade_model(), elemStencil_func(), elemStencil_mask(), elemStencil_op(), elemAlpha_test_enable(), elemBlend_enable(), elemClip_plane_enable(), elemColor_logic_op_enable(), elemColor_material_enable(), elemCull_face_enable(), elemDepth_test_enable(), elemDither_enable(), elemFog_enable(), elemTexture_pipeline_enable(), elemLight_enable(), elemLighting_enable(), elemLight_model_two_side_enable(), elemLine_smooth_enable(), elemMultisample_enable(), elemNormalize_enable(), elemPoint_smooth_enable(), elemPolygon_offset_fill_enable(), elemRescale_normal_enable(), elemSample_alpha_to_coverage_enable(), elemSample_alpha_to_one_enable(), elemSample_coverage_enable(), elemScissor_test_enable(), elemStencil_test_enable() {}
	/**
	 * Destructor
	 */
	virtual ~domGles_pipeline_settings() {}
	/**
	 * Copy Constructor
	 */
	domGles_pipeline_settings( const domGles_pipeline_settings &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGles_pipeline_settings &operator=( const domGles_pipeline_settings &cpy ) { (void)cpy; return *this; }

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
