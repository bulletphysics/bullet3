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
#ifndef __domInstance_material_h__
#define __domInstance_material_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domExtra.h>

/**
 * The instance_material element declares the instantiation of a COLLADA material
 * resource.
 */
class domInstance_material : public daeElement
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INSTANCE_MATERIAL; }
public:
	class domBind;

	typedef daeSmartRef<domBind> domBindRef;
	typedef daeTArray<domBindRef> domBind_Array;

/**
 * The bind element binds values to effect parameters upon instantiation.
 */
	class domBind : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BIND; }
	protected:  // Attributes
/**
 *  The semantic attribute specifies which effect parameter to bind. 
 */
		xsNCName attrSemantic;
/**
 *  The target attribute specifies the location of the value to bind to the
 * specified semantic.  This text string is a path-name following a simple
 * syntax described in the “Addressing Syntax”  section. 
 */
		xsToken attrTarget;


	public:	//Accessors and Mutators
		/**
		 * Gets the semantic attribute.
		 * @return Returns a xsNCName of the semantic attribute.
		 */
		xsNCName getSemantic() const { return attrSemantic; }
		/**
		 * Sets the semantic attribute.
		 * @param atSemantic The new value for the semantic attribute.
		 */
		void setSemantic( xsNCName atSemantic ) { *(daeStringRef*)&attrSemantic = atSemantic;	
	 _validAttributeArray[0] = true; }

		/**
		 * Gets the target attribute.
		 * @return Returns a xsToken of the target attribute.
		 */
		xsToken getTarget() const { return attrTarget; }
		/**
		 * Sets the target attribute.
		 * @param atTarget The new value for the target attribute.
		 */
		void setTarget( xsToken atTarget ) { *(daeStringRef*)&attrTarget = atTarget;	
	 _validAttributeArray[1] = true; }

	protected:
		/**
		 * Constructor
		 */
		domBind() : attrSemantic(), attrTarget() {}
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

	class domBind_vertex_input;

	typedef daeSmartRef<domBind_vertex_input> domBind_vertex_inputRef;
	typedef daeTArray<domBind_vertex_inputRef> domBind_vertex_input_Array;

/**
 * The bind_vertex_input element binds vertex inputs to effect parameters
 * upon instantiation.
 */
	class domBind_vertex_input : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::BIND_VERTEX_INPUT; }
	protected:  // Attributes
/**
 *  The semantic attribute specifies which effect parameter to bind. 
 */
		xsNCName attrSemantic;
/**
 *  The input_semantic attribute specifies which input semantic to bind. 
 */
		xsNCName attrInput_semantic;
/**
 *  The input_set attribute specifies which input set to bind.  
 */
		domUint attrInput_set;


	public:	//Accessors and Mutators
		/**
		 * Gets the semantic attribute.
		 * @return Returns a xsNCName of the semantic attribute.
		 */
		xsNCName getSemantic() const { return attrSemantic; }
		/**
		 * Sets the semantic attribute.
		 * @param atSemantic The new value for the semantic attribute.
		 */
		void setSemantic( xsNCName atSemantic ) { *(daeStringRef*)&attrSemantic = atSemantic;	
	 _validAttributeArray[0] = true; }

		/**
		 * Gets the input_semantic attribute.
		 * @return Returns a xsNCName of the input_semantic attribute.
		 */
		xsNCName getInput_semantic() const { return attrInput_semantic; }
		/**
		 * Sets the input_semantic attribute.
		 * @param atInput_semantic The new value for the input_semantic attribute.
		 */
		void setInput_semantic( xsNCName atInput_semantic ) { *(daeStringRef*)&attrInput_semantic = atInput_semantic;	
	 _validAttributeArray[1] = true; }

		/**
		 * Gets the input_set attribute.
		 * @return Returns a domUint of the input_set attribute.
		 */
		domUint getInput_set() const { return attrInput_set; }
		/**
		 * Sets the input_set attribute.
		 * @param atInput_set The new value for the input_set attribute.
		 */
		void setInput_set( domUint atInput_set ) { attrInput_set = atInput_set;	
	 _validAttributeArray[2] = true; }

	protected:
		/**
		 * Constructor
		 */
		domBind_vertex_input() : attrSemantic(), attrInput_semantic(), attrInput_set() {}
		/**
		 * Destructor
		 */
		virtual ~domBind_vertex_input() {}
		/**
		 * Copy Constructor
		 */
		domBind_vertex_input( const domBind_vertex_input &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domBind_vertex_input &operator=( const domBind_vertex_input &cpy ) { (void)cpy; return *this; }

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
 *  The symbol attribute specifies which symbol defined from within the geometry
 * this material binds to. 
 */
	xsNCName attrSymbol;
/**
 *  The target attribute specifies the URL of the location of the object to
 * instantiate. 
 */
	xsAnyURI attrTarget;
/**
 *  The sid attribute is a text string value containing the sub-identifier
 * of this element. This  value must be unique within the scope of the parent
 * element. Optional attribute. 
 */
	xsNCName attrSid;
/**
 *  The name attribute is the text string name of this element. Optional attribute.
 */
	xsNCName attrName;

protected:  // Elements
/**
 * The bind element binds values to effect parameters upon instantiation.
 * @see domBind
 */
	domBind_Array elemBind_array;
/**
 * The bind_vertex_input element binds vertex inputs to effect parameters
 * upon instantiation. @see domBind_vertex_input
 */
	domBind_vertex_input_Array elemBind_vertex_input_array;
/**
 *  The extra element may appear any number of times.  @see domExtra
 */
	domExtra_Array elemExtra_array;

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
	 * Gets the target attribute.
	 * @return Returns a xsAnyURI reference of the target attribute.
	 */
	xsAnyURI &getTarget() { return attrTarget; }
	/**
	 * Gets the target attribute.
	 * @return Returns a constant xsAnyURI reference of the target attribute.
	 */
	const xsAnyURI &getTarget() const { return attrTarget; }
	/**
	 * Sets the target attribute.
	 * @param atTarget The new value for the target attribute.
	 */
	void setTarget( const xsAnyURI &atTarget ) { attrTarget = atTarget;
	 _validAttributeArray[1] = true; }

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
	 _validAttributeArray[2] = true; }

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
	 _validAttributeArray[3] = true; }

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
	/**
	 * Gets the bind_vertex_input element array.
	 * @return Returns a reference to the array of bind_vertex_input elements.
	 */
	domBind_vertex_input_Array &getBind_vertex_input_array() { return elemBind_vertex_input_array; }
	/**
	 * Gets the bind_vertex_input element array.
	 * @return Returns a constant reference to the array of bind_vertex_input elements.
	 */
	const domBind_vertex_input_Array &getBind_vertex_input_array() const { return elemBind_vertex_input_array; }
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
	domInstance_material() : attrSymbol(), attrTarget(), attrSid(), attrName(), elemBind_array(), elemBind_vertex_input_array(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domInstance_material() {}
	/**
	 * Copy Constructor
	 */
	domInstance_material( const domInstance_material &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domInstance_material &operator=( const domInstance_material &cpy ) { (void)cpy; return *this; }

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
