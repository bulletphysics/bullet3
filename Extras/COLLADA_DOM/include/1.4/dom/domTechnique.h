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
#ifndef __domTechnique_h__
#define __domTechnique_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>


/**
 * The technique element declares the information used to process some portion
 * of the content. Each  technique conforms to an associated profile. Techniques
 * generally act as a “switch”. If more than  one is present for a particular
 * portion of content, on import, one or the other is picked, but  usually
 * not both. Selection should be based on which profile the importing application
 * can support. Techniques contain application data and programs, making them
 * assets that can be managed as a unit.
 */
class domTechnique : public daeElement
{
protected:  // Attribute
	/**
	 * This element may specify its own xmlns.
	 */
	xsAnyURI attrXmlns;
/**
 *  The profile attribute indicates the type of profile. This is a vendor
 * defined character  string that indicates the platform or capability target
 * for the technique. Required attribute. 
 */
	xsNMTOKEN attrProfile;

protected:  // Element
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
	 * Gets the xmlns attribute.
	 * @return Returns a xsAnyURI reference of the xmlns attribute.
	 */
	xsAnyURI &getXmlns() { return attrXmlns; }
	/**
	 * Gets the xmlns attribute.
	 * @return Returns a constant xsAnyURI reference of the xmlns attribute.
	 */
	const xsAnyURI &getXmlns() const { return attrXmlns; }
	/**
	 * Sets the xmlns attribute.
	 * @param xmlns The new value for the xmlns attribute.
	 */
	void setXmlns( const xsAnyURI &xmlns ) { attrXmlns.setURI( xmlns.getURI() );
	 _validAttributeArray[0] = true; }

	/**
	 * Gets the profile attribute.
	 * @return Returns a xsNMTOKEN of the profile attribute.
	 */
	xsNMTOKEN getProfile() const { return attrProfile; }
	/**
	 * Sets the profile attribute.
	 * @param atProfile The new value for the profile attribute.
	 */
	void setProfile( xsNMTOKEN atProfile ) { *(daeStringRef*)&attrProfile = atProfile;
	 _validAttributeArray[1] = true; }

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
	domTechnique() : attrProfile() {}
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
