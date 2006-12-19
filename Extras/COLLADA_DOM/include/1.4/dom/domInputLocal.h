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
#ifndef __domInputLocal_h__
#define __domInputLocal_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>


/**
 * The InputLocal type is used to represent inputs that can only reference
 * resources declared in the same document.
 */
class domInputLocal_complexType 
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
	domURIFragmentType attrSource;


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
	void setSemantic( xsNMTOKEN atSemantic ) { *(daeStringRef*)&attrSemantic = atSemantic; }

	/**
	 * Gets the source attribute.
	 * @return Returns a domURIFragmentType reference of the source attribute.
	 */
	domURIFragmentType &getSource() { return attrSource; }
	/**
	 * Gets the source attribute.
	 * @return Returns a constant domURIFragmentType reference of the source attribute.
	 */
	const domURIFragmentType &getSource() const { return attrSource; }
	/**
	 * Sets the source attribute.
	 * @param atSource The new value for the source attribute.
	 */
	void setSource( const domURIFragmentType &atSource ) { attrSource = atSource; }

protected:
	/**
	 * Constructor
	 */
	domInputLocal_complexType() : attrSemantic(), attrSource() {}
	/**
	 * Destructor
	 */
	virtual ~domInputLocal_complexType() {}
	/**
	 * Copy Constructor
	 */
	domInputLocal_complexType( const domInputLocal_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domInputLocal_complexType &operator=( const domInputLocal_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domInputLocal_complexType.
 */
class domInputLocal : public daeElement, public domInputLocal_complexType
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INPUTLOCAL; }

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
	void setSemantic( xsNMTOKEN atSemantic ) { *(daeStringRef*)&attrSemantic = atSemantic;
	 _validAttributeArray[0] = true; }

	/**
	 * Gets the source attribute.
	 * @return Returns a domURIFragmentType reference of the source attribute.
	 */
	domURIFragmentType &getSource() { return attrSource; }
	/**
	 * Gets the source attribute.
	 * @return Returns a constant domURIFragmentType reference of the source attribute.
	 */
	const domURIFragmentType &getSource() const { return attrSource; }
	/**
	 * Sets the source attribute.
	 * @param atSource The new value for the source attribute.
	 */
	void setSource( const domURIFragmentType &atSource ) { attrSource = atSource;
	 _validAttributeArray[1] = true; }

protected:
	/**
	 * Constructor
	 */
	domInputLocal() {}
	/**
	 * Destructor
	 */
	virtual ~domInputLocal() {}
	/**
	 * Copy Constructor
	 */
	domInputLocal( const domInputLocal &cpy ) : daeElement(), domInputLocal_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domInputLocal &operator=( const domInputLocal &cpy ) { (void)cpy; return *this; }

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
