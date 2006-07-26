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
#ifndef __domMorph_h__
#define __domMorph_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domSource.h>
#include <dom/domExtra.h>
#include <dom/domInputLocal.h>

/**
 * The morph element describes the data required to blend between sets of
 * static meshes. Each  possible mesh that can be blended (a morph target)
 * must be specified.
 */
class domMorph : public daeElement
{
public:
	class domTargets;

	typedef daeSmartRef<domTargets> domTargetsRef;
	typedef daeTArray<domTargetsRef> domTargets_Array;

/**
 * The targets element declares the morph targets, their weights and any user
 * defined attributes  associated with them.
 */
	class domTargets : public daeElement
	{

	protected:  // Elements
/**
 * The input element must occur at least twice. These inputs are local inputs.
 * @see domInput
 */
		domInputLocal_Array elemInput_array;
/**
 *  The extra element may appear any number of times.  @see domExtra
 */
		domExtra_Array elemExtra_array;

	public:	//Accessors and Mutators
		/**
		 * Gets the input element array.
		 * @return Returns a reference to the array of input elements.
		 */
		domInputLocal_Array &getInput_array() { return elemInput_array; }
		/**
		 * Gets the input element array.
		 * @return Returns a constant reference to the array of input elements.
		 */
		const domInputLocal_Array &getInput_array() const { return elemInput_array; }
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
		domTargets() : elemInput_array(), elemExtra_array() {}
		/**
		 * Destructor
		 */
		virtual ~domTargets() {}
		/**
		 * Copy Constructor
		 */
		domTargets( const domTargets &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domTargets &operator=( const domTargets &cpy ) { (void)cpy; return *this; }

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
 *  The method attribute specifies the which blending technique to use. The
 * accepted values are  NORMALIZED, and RELATIVE. The default value if not
 * specified is NORMALIZED.  Optional attribute. 
 */
	domMorphMethodType attrMethod;
/**
 *  The source attribute indicates the base mesh. Required attribute. 
 */
	xsAnyURI attrSource;

protected:  // Elements
/**
 *  The morph element must contain at least two source elements.  @see domSource
 */
	domSource_Array elemSource_array;
/**
 * The targets element declares the morph targets, their weights and any user
 * defined attributes  associated with them. @see domTargets
 */
	domTargetsRef elemTargets;
/**
 *  The extra element may appear any number of times.  @see domExtra
 */
	domExtra_Array elemExtra_array;

public:	//Accessors and Mutators
	/**
	 * Gets the method attribute.
	 * @return Returns a domMorphMethodType of the method attribute.
	 */
	domMorphMethodType getMethod() const { return attrMethod; }
	/**
	 * Sets the method attribute.
	 * @param atMethod The new value for the method attribute.
	 */
	void setMethod( domMorphMethodType atMethod ) { attrMethod = atMethod;
	 _validAttributeArray[0] = true; }

	/**
	 * Gets the source attribute.
	 * @return Returns a xsAnyURI reference of the source attribute.
	 */
	xsAnyURI &getSource() { return attrSource; }
	/**
	 * Gets the source attribute.
	 * @return Returns a constant xsAnyURI reference of the source attribute.
	 */
	const xsAnyURI &getSource() const { return attrSource; }
	/**
	 * Sets the source attribute.
	 * @param atSource The new value for the source attribute.
	 */
	void setSource( const xsAnyURI &atSource ) { attrSource.setURI( atSource.getURI() );
	 _validAttributeArray[1] = true; }

	/**
	 * Gets the source element array.
	 * @return Returns a reference to the array of source elements.
	 */
	domSource_Array &getSource_array() { return elemSource_array; }
	/**
	 * Gets the source element array.
	 * @return Returns a constant reference to the array of source elements.
	 */
	const domSource_Array &getSource_array() const { return elemSource_array; }
	/**
	 * Gets the targets element.
	 * @return a daeSmartRef to the targets element.
	 */
	const domTargetsRef getTargets() const { return elemTargets; }
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
	domMorph() : attrMethod(), attrSource(), elemSource_array(), elemTargets(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domMorph() {}
	/**
	 * Copy Constructor
	 */
	domMorph( const domMorph &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domMorph &operator=( const domMorph &cpy ) { (void)cpy; return *this; }

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
