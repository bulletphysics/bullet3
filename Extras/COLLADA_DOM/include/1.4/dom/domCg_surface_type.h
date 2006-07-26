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
#ifndef __domCg_surface_type_h__
#define __domCg_surface_type_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domFx_surface_common.h>
#include <dom/domFx_annotate_common.h>
#include <dom/domFx_code_profile.h>
#include <dom/domFx_include_common.h>
#include <dom/domCg_setparam_simple.h>

/**
 * Declares a resource that can be used both as the source for texture samples
 * and as the target of a rendering pass.
 */
class domCg_surface_type_complexType : public domFx_surface_common_complexType
{
public:
	class domGenerator;

	typedef daeSmartRef<domGenerator> domGeneratorRef;
	typedef daeTArray<domGeneratorRef> domGenerator_Array;

/**
 * A procedural surface generator for the cg profile.
 */
	class domGenerator : public daeElement
	{
	public:
		class domName;

		typedef daeSmartRef<domName> domNameRef;
		typedef daeTArray<domNameRef> domName_Array;

/**
 * The entry symbol for the shader function.
 */
		class domName : public daeElement
		{
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
 * The annotate element allows you to specify an annotation for this generator.
 * @see domAnnotate
 */
		domFx_annotate_common_Array elemAnnotate_array;
/**
 * The code element allows you to embed cg sourcecode for the surface generator.
 * @see domCode
 */
		domFx_code_profile_Array elemCode_array;
/**
 * The include element imports cg source code or precompiled binary shaders
 * into the FX Runtime by referencing an external resource. @see domInclude
 */
		domFx_include_common_Array elemInclude_array;
/**
 * The entry symbol for the shader function. @see domName
 */
		domNameRef elemName;
/**
 * Assigns a new value to a previously defined parameter. @see domSetparam
 */
		domCg_setparam_simple_Array elemSetparam_array;
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
		 * Gets the name element.
		 * @return a daeSmartRef to the name element.
		 */
		const domNameRef getName() const { return elemName; }
		/**
		 * Gets the setparam element array.
		 * @return Returns a reference to the array of setparam elements.
		 */
		domCg_setparam_simple_Array &getSetparam_array() { return elemSetparam_array; }
		/**
		 * Gets the setparam element array.
		 * @return Returns a constant reference to the array of setparam elements.
		 */
		const domCg_setparam_simple_Array &getSetparam_array() const { return elemSetparam_array; }
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
		domGenerator() : elemAnnotate_array(), elemCode_array(), elemInclude_array(), elemName(), elemSetparam_array() {}
		/**
		 * Destructor
		 */
		virtual ~domGenerator() {}
		/**
		 * Copy Constructor
		 */
		domGenerator( const domGenerator &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domGenerator &operator=( const domGenerator &cpy ) { (void)cpy; return *this; }

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



protected:  // Element
/**
 * A procedural surface generator for the cg profile. @see domGenerator
 */
	domGeneratorRef elemGenerator;

public:	//Accessors and Mutators
	/**
	 * Gets the generator element.
	 * @return a daeSmartRef to the generator element.
	 */
	const domGeneratorRef getGenerator() const { return elemGenerator; }
protected:
	/**
	 * Constructor
	 */
	domCg_surface_type_complexType() : elemGenerator() {}
	/**
	 * Destructor
	 */
	virtual ~domCg_surface_type_complexType() {}
	/**
	 * Copy Constructor
	 */
	domCg_surface_type_complexType( const domCg_surface_type_complexType &cpy ) : domFx_surface_common_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domCg_surface_type_complexType &operator=( const domCg_surface_type_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domCg_surface_type_complexType.
 */
class domCg_surface_type : public daeElement, public domCg_surface_type_complexType
{
protected:
	/**
	 * Constructor
	 */
	domCg_surface_type() {}
	/**
	 * Destructor
	 */
	virtual ~domCg_surface_type() {}
	/**
	 * Copy Constructor
	 */
	domCg_surface_type( const domCg_surface_type &cpy ) : daeElement(), domCg_surface_type_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domCg_surface_type &operator=( const domCg_surface_type &cpy ) { (void)cpy; return *this; }

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
