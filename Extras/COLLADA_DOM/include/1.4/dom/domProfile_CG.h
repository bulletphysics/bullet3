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
#ifndef __domProfile_CG_h__
#define __domProfile_CG_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domFx_profile_abstract.h>
#include <dom/domAsset.h>
#include <dom/domImage.h>
#include <dom/domExtra.h>
#include <dom/domFx_code_profile.h>
#include <dom/domFx_include_common.h>
#include <dom/domCg_newparam.h>
#include <dom/domFx_annotate_common.h>
#include <dom/domCg_setparam.h>
#include <dom/domGl_pipeline_settings.h>
#include <dom/domFx_colortarget_common.h>
#include <dom/domFx_depthtarget_common.h>
#include <dom/domFx_stenciltarget_common.h>
#include <dom/domFx_clearcolor_common.h>
#include <dom/domFx_cleardepth_common.h>
#include <dom/domFx_clearstencil_common.h>
#include <dom/domCg_param_type.h>

/**
 * Opens a block of CG platform-specific data types and technique declarations.
 */
class domProfile_CG : public domFx_profile_abstract
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::PROFILE_CG; }
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
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::TECHNIQUE; }
	public:
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
			COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::PASS; }
		public:
			class domDraw;

			typedef daeSmartRef<domDraw> domDrawRef;
			typedef daeTArray<domDrawRef> domDraw_Array;

			class domDraw : public daeElement
			{
			public:
				COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::DRAW; }

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

			class domShader;

			typedef daeSmartRef<domShader> domShaderRef;
			typedef daeTArray<domShaderRef> domShader_Array;

/**
 * Declare and prepare a shader for execution in the rendering pipeline of
 * a pass.
 */
			class domShader : public daeElement
			{
			public:
				COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::SHADER; }
			public:
				class domCompiler_target;

				typedef daeSmartRef<domCompiler_target> domCompiler_targetRef;
				typedef daeTArray<domCompiler_targetRef> domCompiler_target_Array;

				class domCompiler_target : public daeElement
				{
				public:
					COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::COMPILER_TARGET; }

				protected:  // Value
					/**
					 * The xsNMTOKEN value of the text data of this element. 
					 */
					xsNMTOKEN _value;

				public:	//Accessors and Mutators
					/**
					 * Gets the value of this element.
					 * @return Returns a xsNMTOKEN of the value.
					 */
					xsNMTOKEN getValue() const { return _value; }
					/**
					 * Sets the _value of this element.
					 * @param val The new value for this element.
					 */
					void setValue( xsNMTOKEN val ) { *(daeStringRef*)&_value = val; }

				protected:
					/**
					 * Constructor
					 */
					domCompiler_target() : _value() {}
					/**
					 * Destructor
					 */
					virtual ~domCompiler_target() {}
					/**
					 * Copy Constructor
					 */
					domCompiler_target( const domCompiler_target &cpy ) : daeElement() { (void)cpy; }
					/**
					 * Overloaded assignment operator
					 */
					virtual domCompiler_target &operator=( const domCompiler_target &cpy ) { (void)cpy; return *this; }

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

				class domCompiler_options;

				typedef daeSmartRef<domCompiler_options> domCompiler_optionsRef;
				typedef daeTArray<domCompiler_optionsRef> domCompiler_options_Array;

/**
 * A string containing command-line operations for the shader compiler.
 */
				class domCompiler_options : public daeElement
				{
				public:
					COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::COMPILER_OPTIONS; }

				protected:  // Value
					/**
					 * The xsString value of the text data of this element. 
					 */
					xsString _value;

				public:	//Accessors and Mutators
					/**
					 * Gets the value of this element.
					 * @return Returns a xsString of the value.
					 */
					xsString getValue() const { return _value; }
					/**
					 * Sets the _value of this element.
					 * @param val The new value for this element.
					 */
					void setValue( xsString val ) { *(daeStringRef*)&_value = val; }

				protected:
					/**
					 * Constructor
					 */
					domCompiler_options() : _value() {}
					/**
					 * Destructor
					 */
					virtual ~domCompiler_options() {}
					/**
					 * Copy Constructor
					 */
					domCompiler_options( const domCompiler_options &cpy ) : daeElement() { (void)cpy; }
					/**
					 * Overloaded assignment operator
					 */
					virtual domCompiler_options &operator=( const domCompiler_options &cpy ) { (void)cpy; return *this; }

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

				class domName;

				typedef daeSmartRef<domName> domNameRef;
				typedef daeTArray<domNameRef> domName_Array;

/**
 * The entry symbol for the shader function.
 */
				class domName : public daeElement
				{
				public:
					COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::NAME; }
				protected:  // Attribute
					xsNCName attrSource;

				protected:  // Value
					/**
					 * The xsNCName value of the text data of this element. 
					 */
					xsNCName _value;

				public:	//Accessors and Mutators
					/**
					 * Gets the source attribute.
					 * @return Returns a xsNCName of the source attribute.
					 */
					xsNCName getSource() const { return attrSource; }
					/**
					 * Sets the source attribute.
					 * @param atSource The new value for the source attribute.
					 */
					void setSource( xsNCName atSource ) { *(daeStringRef*)&attrSource = atSource;				
	 _validAttributeArray[0] = true; }

					/**
					 * Gets the value of this element.
					 * @return Returns a xsNCName of the value.
					 */
					xsNCName getValue() const { return _value; }
					/**
					 * Sets the _value of this element.
					 * @param val The new value for this element.
					 */
					void setValue( xsNCName val ) { *(daeStringRef*)&_value = val; }

				protected:
					/**
					 * Constructor
					 */
					domName() : attrSource(), _value() {}
					/**
					 * Destructor
					 */
					virtual ~domName() {}
					/**
					 * Copy Constructor
					 */
					domName( const domName &cpy ) : daeElement() { (void)cpy; }
					/**
					 * Overloaded assignment operator
					 */
					virtual domName &operator=( const domName &cpy ) { (void)cpy; return *this; }

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

				class domBind;

				typedef daeSmartRef<domBind> domBindRef;
				typedef daeTArray<domBindRef> domBind_Array;

/**
 * Binds values to uniform inputs of a shader.
 */
				class domBind : public daeElement
				{
				public:
					COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BIND; }
				public:
					class domParam;

					typedef daeSmartRef<domParam> domParamRef;
					typedef daeTArray<domParamRef> domParam_Array;

/**
 * References a predefined parameter in shader binding declarations.
 */
					class domParam : public daeElement
					{
					public:
						COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::PARAM; }
					protected:  // Attribute
						xsNCName attrRef;


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

					protected:
						/**
						 * Constructor
						 */
						domParam() : attrRef() {}
						/**
						 * Destructor
						 */
						virtual ~domParam() {}
						/**
						 * Copy Constructor
						 */
						domParam( const domParam &cpy ) : daeElement() { (void)cpy; }
						/**
						 * Overloaded assignment operator
						 */
						virtual domParam &operator=( const domParam &cpy ) { (void)cpy; return *this; }

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


				protected:  // Attribute
/**
 *  The identifier for a uniform input parameter to the shader (a formal function
 * parameter or in-scope  global) that will be bound to an external resource.
 */
					xsNCName attrSymbol;

				protected:  // Elements
					domCg_param_typeRef elemCg_param_type;
/**
 * References a predefined parameter in shader binding declarations. @see
 * domParam
 */
					domParamRef elemParam;
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
					 * Gets the symbol attribute.
					 * @return Returns a xsNCName of the symbol attribute.
					 */
					xsNCName getSymbol() const { return attrSymbol; }
					/**
					 * Sets the symbol attribute.
					 * @param atSymbol The new value for the symbol attribute.
					 */
					void setSymbol( xsNCName atSymbol ) { *(daeStringRef*)&attrSymbol = atSymbol;				
	 _validAttributeArray[0] = true; }

					/**
					 * Gets the cg_param_type element.
					 * @return a daeSmartRef to the cg_param_type element.
					 */
					const domCg_param_typeRef getCg_param_type() const { return elemCg_param_type; }
					/**
					 * Gets the param element.
					 * @return a daeSmartRef to the param element.
					 */
					const domParamRef getParam() const { return elemParam; }
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
					domBind() : attrSymbol(), elemCg_param_type(), elemParam() {}
					/**
					 * Destructor
					 */
					virtual ~domBind() {}
					/**
					 * Copy Constructor
					 */
					domBind( const domBind &cpy ) : daeElement() { (void)cpy; }
					/**
					 * Overloaded assignment operator
					 */
					virtual domBind &operator=( const domBind &cpy ) { (void)cpy; return *this; }

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


			protected:  // Attribute
/**
 *  In which pipeline stage this programmable shader is designed to execute,
 * for example, VERTEX, FRAGMENT, etc. 
 */
				domCg_pipeline_stage attrStage;

			protected:  // Elements
				domFx_annotate_common_Array elemAnnotate_array;
				domCompiler_targetRef elemCompiler_target;
/**
 * A string containing command-line operations for the shader compiler. @see
 * domCompiler_options
 */
				domCompiler_optionsRef elemCompiler_options;
/**
 * The entry symbol for the shader function. @see domName
 */
				domNameRef elemName;
/**
 * Binds values to uniform inputs of a shader. @see domBind
 */
				domBind_Array elemBind_array;

			public:	//Accessors and Mutators
				/**
				 * Gets the stage attribute.
				 * @return Returns a domCg_pipeline_stage of the stage attribute.
				 */
				domCg_pipeline_stage getStage() const { return attrStage; }
				/**
				 * Sets the stage attribute.
				 * @param atStage The new value for the stage attribute.
				 */
				void setStage( domCg_pipeline_stage atStage ) { attrStage = atStage;			
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
				 * Gets the compiler_target element.
				 * @return a daeSmartRef to the compiler_target element.
				 */
				const domCompiler_targetRef getCompiler_target() const { return elemCompiler_target; }
				/**
				 * Gets the compiler_options element.
				 * @return a daeSmartRef to the compiler_options element.
				 */
				const domCompiler_optionsRef getCompiler_options() const { return elemCompiler_options; }
				/**
				 * Gets the name element.
				 * @return a daeSmartRef to the name element.
				 */
				const domNameRef getName() const { return elemName; }
				/**
				 * Gets the bind element array.
				 * @return Returns a reference to the array of bind elements.
				 */
				domBind_Array &getBind_array() { return elemBind_array; }
				/**
				 * Gets the bind element array.
				 * @return Returns a constant reference to the array of bind elements.
				 */
				const domBind_Array &getBind_array() const { return elemBind_array; }
			protected:
				/**
				 * Constructor
				 */
				domShader() : attrStage(), elemAnnotate_array(), elemCompiler_target(), elemCompiler_options(), elemName(), elemBind_array() {}
				/**
				 * Destructor
				 */
				virtual ~domShader() {}
				/**
				 * Copy Constructor
				 */
				domShader( const domShader &cpy ) : daeElement() { (void)cpy; }
				/**
				 * Overloaded assignment operator
				 */
				virtual domShader &operator=( const domShader &cpy ) { (void)cpy; return *this; }

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


		protected:  // Attribute
/**
 *  The sid attribute is a text string value containing the sub-identifier
 * of this element.  This value must be unique within the scope of the parent
 * element. Optional attribute. 
 */
			xsNCName attrSid;

		protected:  // Elements
			domFx_annotate_common_Array elemAnnotate_array;
			domFx_colortarget_common_Array elemColor_target_array;
			domFx_depthtarget_common_Array elemDepth_target_array;
			domFx_stenciltarget_common_Array elemStencil_target_array;
			domFx_clearcolor_common_Array elemColor_clear_array;
			domFx_cleardepth_common_Array elemDepth_clear_array;
			domFx_clearstencil_common_Array elemStencil_clear_array;
			domDrawRef elemDraw;
			domGl_pipeline_settings_Array elemGl_pipeline_settings_array;
/**
 * Declare and prepare a shader for execution in the rendering pipeline of
 * a pass. @see domShader
 */
			domShader_Array elemShader_array;
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
			 * Gets the color_target element array.
			 * @return Returns a reference to the array of color_target elements.
			 */
			domFx_colortarget_common_Array &getColor_target_array() { return elemColor_target_array; }
			/**
			 * Gets the color_target element array.
			 * @return Returns a constant reference to the array of color_target elements.
			 */
			const domFx_colortarget_common_Array &getColor_target_array() const { return elemColor_target_array; }
			/**
			 * Gets the depth_target element array.
			 * @return Returns a reference to the array of depth_target elements.
			 */
			domFx_depthtarget_common_Array &getDepth_target_array() { return elemDepth_target_array; }
			/**
			 * Gets the depth_target element array.
			 * @return Returns a constant reference to the array of depth_target elements.
			 */
			const domFx_depthtarget_common_Array &getDepth_target_array() const { return elemDepth_target_array; }
			/**
			 * Gets the stencil_target element array.
			 * @return Returns a reference to the array of stencil_target elements.
			 */
			domFx_stenciltarget_common_Array &getStencil_target_array() { return elemStencil_target_array; }
			/**
			 * Gets the stencil_target element array.
			 * @return Returns a constant reference to the array of stencil_target elements.
			 */
			const domFx_stenciltarget_common_Array &getStencil_target_array() const { return elemStencil_target_array; }
			/**
			 * Gets the color_clear element array.
			 * @return Returns a reference to the array of color_clear elements.
			 */
			domFx_clearcolor_common_Array &getColor_clear_array() { return elemColor_clear_array; }
			/**
			 * Gets the color_clear element array.
			 * @return Returns a constant reference to the array of color_clear elements.
			 */
			const domFx_clearcolor_common_Array &getColor_clear_array() const { return elemColor_clear_array; }
			/**
			 * Gets the depth_clear element array.
			 * @return Returns a reference to the array of depth_clear elements.
			 */
			domFx_cleardepth_common_Array &getDepth_clear_array() { return elemDepth_clear_array; }
			/**
			 * Gets the depth_clear element array.
			 * @return Returns a constant reference to the array of depth_clear elements.
			 */
			const domFx_cleardepth_common_Array &getDepth_clear_array() const { return elemDepth_clear_array; }
			/**
			 * Gets the stencil_clear element array.
			 * @return Returns a reference to the array of stencil_clear elements.
			 */
			domFx_clearstencil_common_Array &getStencil_clear_array() { return elemStencil_clear_array; }
			/**
			 * Gets the stencil_clear element array.
			 * @return Returns a constant reference to the array of stencil_clear elements.
			 */
			const domFx_clearstencil_common_Array &getStencil_clear_array() const { return elemStencil_clear_array; }
			/**
			 * Gets the draw element.
			 * @return a daeSmartRef to the draw element.
			 */
			const domDrawRef getDraw() const { return elemDraw; }
			/**
			 * Gets the gl_pipeline_settings element array.
			 * @return Returns a reference to the array of gl_pipeline_settings elements.
			 */
			domGl_pipeline_settings_Array &getGl_pipeline_settings_array() { return elemGl_pipeline_settings_array; }
			/**
			 * Gets the gl_pipeline_settings element array.
			 * @return Returns a constant reference to the array of gl_pipeline_settings elements.
			 */
			const domGl_pipeline_settings_Array &getGl_pipeline_settings_array() const { return elemGl_pipeline_settings_array; }
			/**
			 * Gets the shader element array.
			 * @return Returns a reference to the array of shader elements.
			 */
			domShader_Array &getShader_array() { return elemShader_array; }
			/**
			 * Gets the shader element array.
			 * @return Returns a constant reference to the array of shader elements.
			 */
			const domShader_Array &getShader_array() const { return elemShader_array; }
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
			domPass() : attrSid(), elemAnnotate_array(), elemColor_target_array(), elemDepth_target_array(), elemStencil_target_array(), elemColor_clear_array(), elemDepth_clear_array(), elemStencil_clear_array(), elemDraw(), elemGl_pipeline_settings_array(), elemShader_array(), elemExtra_array() {}
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
		domFx_annotate_common_Array elemAnnotate_array;
		domFx_code_profile_Array elemCode_array;
		domFx_include_common_Array elemInclude_array;
		domImage_Array elemImage_array;
		domCg_newparam_Array elemNewparam_array;
		domCg_setparam_Array elemSetparam_array;
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
		 * Gets the code element array.
		 * @return Returns a reference to the array of code elements.
		 */
		domFx_code_profile_Array &getCode_array() { return elemCode_array; }
		/**
		 * Gets the code element array.
		 * @return Returns a constant reference to the array of code elements.
		 */
		const domFx_code_profile_Array &getCode_array() const { return elemCode_array; }
		/**
		 * Gets the include element array.
		 * @return Returns a reference to the array of include elements.
		 */
		domFx_include_common_Array &getInclude_array() { return elemInclude_array; }
		/**
		 * Gets the include element array.
		 * @return Returns a constant reference to the array of include elements.
		 */
		const domFx_include_common_Array &getInclude_array() const { return elemInclude_array; }
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
		domCg_newparam_Array &getNewparam_array() { return elemNewparam_array; }
		/**
		 * Gets the newparam element array.
		 * @return Returns a constant reference to the array of newparam elements.
		 */
		const domCg_newparam_Array &getNewparam_array() const { return elemNewparam_array; }
		/**
		 * Gets the setparam element array.
		 * @return Returns a reference to the array of setparam elements.
		 */
		domCg_setparam_Array &getSetparam_array() { return elemSetparam_array; }
		/**
		 * Gets the setparam element array.
		 * @return Returns a constant reference to the array of setparam elements.
		 */
		const domCg_setparam_Array &getSetparam_array() const { return elemSetparam_array; }
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
		domTechnique() : attrId(), attrSid(), elemAsset(), elemAnnotate_array(), elemCode_array(), elemInclude_array(), elemImage_array(), elemNewparam_array(), elemSetparam_array(), elemPass_array(), elemExtra_array() {}
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
 *  The type of platform. This is a vendor-defined character string that indicates
 * the platform or capability target for the technique. Optional 
 */
	xsNCName attrPlatform;

protected:  // Elements
	domAssetRef elemAsset;
	domFx_code_profile_Array elemCode_array;
	domFx_include_common_Array elemInclude_array;
	domImage_Array elemImage_array;
	domCg_newparam_Array elemNewparam_array;
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
	 * Gets the code element array.
	 * @return Returns a reference to the array of code elements.
	 */
	domFx_code_profile_Array &getCode_array() { return elemCode_array; }
	/**
	 * Gets the code element array.
	 * @return Returns a constant reference to the array of code elements.
	 */
	const domFx_code_profile_Array &getCode_array() const { return elemCode_array; }
	/**
	 * Gets the include element array.
	 * @return Returns a reference to the array of include elements.
	 */
	domFx_include_common_Array &getInclude_array() { return elemInclude_array; }
	/**
	 * Gets the include element array.
	 * @return Returns a constant reference to the array of include elements.
	 */
	const domFx_include_common_Array &getInclude_array() const { return elemInclude_array; }
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
	domCg_newparam_Array &getNewparam_array() { return elemNewparam_array; }
	/**
	 * Gets the newparam element array.
	 * @return Returns a constant reference to the array of newparam elements.
	 */
	const domCg_newparam_Array &getNewparam_array() const { return elemNewparam_array; }
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
	domProfile_CG() : attrId(), attrPlatform(), elemAsset(), elemCode_array(), elemInclude_array(), elemImage_array(), elemNewparam_array(), elemTechnique_array(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domProfile_CG() {}
	/**
	 * Copy Constructor
	 */
	domProfile_CG( const domProfile_CG &cpy ) : domFx_profile_abstract() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domProfile_CG &operator=( const domProfile_CG &cpy ) { (void)cpy; return *this; }

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
