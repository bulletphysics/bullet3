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
	class domBind;

	typedef daeSmartRef<domBind> domBindRef;
	typedef daeTArray<domBindRef> domBind_Array;

/**
 * The bind element binds values to effect parameters upon instantiation.
 */
	class domBind : public daeElement
	{
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
		void setSemantic( xsNCName atSemantic ) { attrSemantic = atSemantic; }

		/**
		 * Gets the target attribute.
		 * @return Returns a xsToken of the target attribute.
		 */
		xsToken getTarget() const { return attrTarget; }
		/**
		 * Sets the target attribute.
		 * @param atTarget The new value for the target attribute.
		 */
		void setTarget( xsToken atTarget ) { attrTarget = atTarget; }

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
 *  The symbol attribute specifies which symbol defined from within the geometry
 * this material binds to. 
 */
	xsNCName attrSymbol;
/**
 *  The target attribute specifies the URL of the location of the object to
 * instantiate. 
 */
	xsAnyURI attrTarget;

protected:  // Elements
/**
 * The bind element binds values to effect parameters upon instantiation.
 * @see domBind
 */
	domBind_Array elemBind_array;
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
	void setSymbol( xsNCName atSymbol ) { attrSymbol = atSymbol; }

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
	void setTarget( const xsAnyURI &atTarget ) { attrTarget.setURI( atTarget.getURI() ); }

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
	domInstance_material() : attrSymbol(), attrTarget(), elemBind_array(), elemExtra_array() {}
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
