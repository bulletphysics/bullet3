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
#ifndef __domCg_connect_param_h__
#define __domCg_connect_param_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>


/**
 * Creates a symbolic connection between two previously defined parameters.
 */
class domCg_connect_param_complexType 
{
protected:  // Attribute
	domCg_identifier attrRef;


public:	//Accessors and Mutators
	/**
	 * Gets the ref attribute.
	 * @return Returns a domCg_identifier of the ref attribute.
	 */
	domCg_identifier getRef() const { return attrRef; }
	/**
	 * Sets the ref attribute.
	 * @param atRef The new value for the ref attribute.
	 */
	void setRef( domCg_identifier atRef ) { attrRef = atRef; }

protected:
	/**
	 * Constructor
	 */
	domCg_connect_param_complexType() : attrRef() {}
	/**
	 * Destructor
	 */
	virtual ~domCg_connect_param_complexType() {}
	/**
	 * Copy Constructor
	 */
	domCg_connect_param_complexType( const domCg_connect_param_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domCg_connect_param_complexType &operator=( const domCg_connect_param_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domCg_connect_param_complexType.
 */
class domCg_connect_param : public daeElement, public domCg_connect_param_complexType
{
protected:
	/**
	 * Constructor
	 */
	domCg_connect_param() {}
	/**
	 * Destructor
	 */
	virtual ~domCg_connect_param() {}
	/**
	 * Copy Constructor
	 */
	domCg_connect_param( const domCg_connect_param &cpy ) : daeElement(), domCg_connect_param_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domCg_connect_param &operator=( const domCg_connect_param &cpy ) { (void)cpy; return *this; }

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
