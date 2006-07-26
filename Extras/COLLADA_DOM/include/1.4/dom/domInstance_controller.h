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
#ifndef __domInstance_controller_h__
#define __domInstance_controller_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domBind_material.h>
#include <dom/domExtra.h>

/**
 * The instance_controller element declares the instantiation of a COLLADA
 * controller resource.
 */
class domInstance_controller : public daeElement
{
public:
	class domSkeleton;

	typedef daeSmartRef<domSkeleton> domSkeletonRef;
	typedef daeTArray<domSkeletonRef> domSkeleton_Array;

/**
 * The skeleton element is used to indicate where a skin controller is to
 * start to search for  the joint nodes it needs.  This element is meaningless
 * for morph controllers.
 */
	class domSkeleton : public daeElement
	{

	protected:  // Value
		/**
		 * The xsAnyURI value of the text data of this element. 
		 */
		xsAnyURI _value;

	public:	//Accessors and Mutators
		/**
		 * Gets the value of this element.
		 * @return Returns a xsAnyURI of the value.
		 */
		xsAnyURI &getValue() { return _value; }
		/**
		 * Gets the value of this element.
		 * @return Returns a constant xsAnyURI of the value.
		 */
		const xsAnyURI &getValue() const { return _value; }
		/**
		 * Sets the _value of this element.
		 * @param val The new value for this element.
		 */
		void setValue( const xsAnyURI &val ) { _value.setURI( val.getURI() ); }

	protected:
		/**
		 * Constructor
		 */
		domSkeleton() : _value() {}
		/**
		 * Destructor
		 */
		virtual ~domSkeleton() {}
		/**
		 * Copy Constructor
		 */
		domSkeleton( const domSkeleton &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domSkeleton &operator=( const domSkeleton &cpy ) { (void)cpy; return *this; }

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
 *  The url attribute refers to resource. This may refer to a local resource
 * using a relative  URL fragment identifier that begins with the “#”
 * character. The url attribute may refer to an  external resource using an
 * absolute or relative URL. 
 */
	xsAnyURI attrUrl;
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
 * The skeleton element is used to indicate where a skin controller is to
 * start to search for  the joint nodes it needs.  This element is meaningless
 * for morph controllers. @see domSkeleton
 */
	domSkeleton_Array elemSkeleton_array;
/**
 *  Bind a specific material to a piece of geometry, binding varying and uniform
 * parameters at the  same time.  @see domBind_material
 */
	domBind_materialRef elemBind_material;
/**
 *  The extra element may appear any number of times.  @see domExtra
 */
	domExtra_Array elemExtra_array;

public:	//Accessors and Mutators
	/**
	 * Gets the url attribute.
	 * @return Returns a xsAnyURI reference of the url attribute.
	 */
	xsAnyURI &getUrl() { return attrUrl; }
	/**
	 * Gets the url attribute.
	 * @return Returns a constant xsAnyURI reference of the url attribute.
	 */
	const xsAnyURI &getUrl() const { return attrUrl; }
	/**
	 * Sets the url attribute.
	 * @param atUrl The new value for the url attribute.
	 */
	void setUrl( const xsAnyURI &atUrl ) { attrUrl.setURI( atUrl.getURI() );
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
	 * Gets the name attribute.
	 * @return Returns a xsNCName of the name attribute.
	 */
	xsNCName getName() const { return attrName; }
	/**
	 * Sets the name attribute.
	 * @param atName The new value for the name attribute.
	 */
	void setName( xsNCName atName ) { *(daeStringRef*)&attrName = atName;
	 _validAttributeArray[2] = true; }

	/**
	 * Gets the skeleton element array.
	 * @return Returns a reference to the array of skeleton elements.
	 */
	domSkeleton_Array &getSkeleton_array() { return elemSkeleton_array; }
	/**
	 * Gets the skeleton element array.
	 * @return Returns a constant reference to the array of skeleton elements.
	 */
	const domSkeleton_Array &getSkeleton_array() const { return elemSkeleton_array; }
	/**
	 * Gets the bind_material element.
	 * @return a daeSmartRef to the bind_material element.
	 */
	const domBind_materialRef getBind_material() const { return elemBind_material; }
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
	domInstance_controller() : attrUrl(), attrSid(), attrName(), elemSkeleton_array(), elemBind_material(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domInstance_controller() {}
	/**
	 * Copy Constructor
	 */
	domInstance_controller( const domInstance_controller &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domInstance_controller &operator=( const domInstance_controller &cpy ) { (void)cpy; return *this; }

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
