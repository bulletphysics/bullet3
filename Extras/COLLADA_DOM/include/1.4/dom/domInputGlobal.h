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
#ifndef __domInputGlobal_h__
#define __domInputGlobal_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>


/**
 * The InputGlobal type is used to represent inputs that can reference external
 * resources.
 */
class domInputGlobal_complexType 
{
protected:  // Attributes
/**
 *  The semantic attribute is the user-defined meaning of the input connection.
 * Required attribute. 
 */
	xsNMTOKEN attrSemantic;
/**
 *  The source attribute indicates the location of the data source. Required
 * attribute. 
 */
	xsAnyURI attrSource;


public:	//Accessors and Mutators
	/**
	 * Gets the semantic attribute.
	 * @return Returns a xsNMTOKEN of the semantic attribute.
	 */
	xsNMTOKEN getSemantic() const { return attrSemantic; }
	/**
	 * Sets the semantic attribute.
	 * @param atSemantic The new value for the semantic attribute.
	 */
	void setSemantic( xsNMTOKEN atSemantic ) { attrSemantic = atSemantic; }

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
	void setSource( const xsAnyURI &atSource ) { attrSource.setURI( atSource.getURI() ); }

protected:
	/**
	 * Constructor
	 */
	domInputGlobal_complexType() : attrSemantic(), attrSource() {}
	/**
	 * Destructor
	 */
	virtual ~domInputGlobal_complexType() {}
	/**
	 * Copy Constructor
	 */
	domInputGlobal_complexType( const domInputGlobal_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domInputGlobal_complexType &operator=( const domInputGlobal_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domInputGlobal_complexType.
 */
class domInputGlobal : public daeElement, public domInputGlobal_complexType
{
protected:
	/**
	 * Constructor
	 */
	domInputGlobal() {}
	/**
	 * Destructor
	 */
	virtual ~domInputGlobal() {}
	/**
	 * Copy Constructor
	 */
	domInputGlobal( const domInputGlobal &cpy ) : daeElement(), domInputGlobal_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domInputGlobal &operator=( const domInputGlobal &cpy ) { (void)cpy; return *this; }

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
