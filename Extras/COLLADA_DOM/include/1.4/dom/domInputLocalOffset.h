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
#ifndef __domInputLocalOffset_h__
#define __domInputLocalOffset_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>


/**
 * The InputLocalOffset type is used to represent indexed inputs that can
 * only reference resources declared in the same document.
 */
class domInputLocalOffset_complexType 
{
protected:  // Attributes
/**
 *  The offset attribute represents the offset into the list of indices. 
 * If two input elements share  the same offset, they will be indexed the
 * same.  This works as a simple form of compression for the  list of indices
 * as well as defining the order the inputs should be used in.  Required attribute.
 */
	domUint attrOffset;
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
/**
 *  The set attribute indicates which inputs should be grouped together as
 * a single set. This is helpful  when multiple inputs share the same semantics.
 */
	domUint attrSet;


public:	//Accessors and Mutators
	/**
	 * Gets the offset attribute.
	 * @return Returns a domUint of the offset attribute.
	 */
	domUint getOffset() const { return attrOffset; }
	/**
	 * Sets the offset attribute.
	 * @param atOffset The new value for the offset attribute.
	 */
	void setOffset( domUint atOffset ) { attrOffset = atOffset; }

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
	void setSource( const domURIFragmentType &atSource ) { attrSource.setURI( atSource.getURI() ); }

	/**
	 * Gets the set attribute.
	 * @return Returns a domUint of the set attribute.
	 */
	domUint getSet() const { return attrSet; }
	/**
	 * Sets the set attribute.
	 * @param atSet The new value for the set attribute.
	 */
	void setSet( domUint atSet ) { attrSet = atSet; }

protected:
	/**
	 * Constructor
	 */
	domInputLocalOffset_complexType() : attrOffset(), attrSemantic(), attrSource(), attrSet() {}
	/**
	 * Destructor
	 */
	virtual ~domInputLocalOffset_complexType() {}
	/**
	 * Copy Constructor
	 */
	domInputLocalOffset_complexType( const domInputLocalOffset_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domInputLocalOffset_complexType &operator=( const domInputLocalOffset_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domInputLocalOffset_complexType.
 */
class domInputLocalOffset : public daeElement, public domInputLocalOffset_complexType
{
protected:
	/**
	 * Constructor
	 */
	domInputLocalOffset() {}
	/**
	 * Destructor
	 */
	virtual ~domInputLocalOffset() {}
	/**
	 * Copy Constructor
	 */
	domInputLocalOffset( const domInputLocalOffset &cpy ) : daeElement(), domInputLocalOffset_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domInputLocalOffset &operator=( const domInputLocalOffset &cpy ) { (void)cpy; return *this; }

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
