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
#ifndef __domFx_include_common_h__
#define __domFx_include_common_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>


/**
 * The include element is used to import source code or precompiled binary
 * shaders into the FX Runtime by referencing an external resource.
 */
class domFx_include_common_complexType 
{
protected:  // Attributes
/**
 *  The sid attribute is a text string value containing the sub-identifier
 * of this element.  This value must be unique within the scope of the parent
 * element. Optional attribute. 
 */
	xsNCName attrSid;
/**
 *  The url attribute refers to resource.  This may refer to a local resource
 * using a relative URL  fragment identifier that begins with the “#”
 * character. The url attribute may refer to an external  resource using an
 * absolute or relative URL. 
 */
	xsAnyURI attrUrl;


public:	//Accessors and Mutators
	/**
	 * Gets the sid attribute.
	 * @return Returns a xsNCName of the sid attribute.
	 */
	xsNCName getSid() const { return attrSid; }
	/**
	 * Sets the sid attribute.
	 * @param atSid The new value for the sid attribute.
	 */
	void setSid( xsNCName atSid ) { attrSid = atSid; }

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
	void setUrl( const xsAnyURI &atUrl ) { attrUrl.setURI( atUrl.getURI() ); }

protected:
	/**
	 * Constructor
	 */
	domFx_include_common_complexType() : attrSid(), attrUrl() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_include_common_complexType() {}
	/**
	 * Copy Constructor
	 */
	domFx_include_common_complexType( const domFx_include_common_complexType &cpy ) { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_include_common_complexType &operator=( const domFx_include_common_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domFx_include_common_complexType.
 */
class domFx_include_common : public daeElement, public domFx_include_common_complexType
{
protected:
	/**
	 * Constructor
	 */
	domFx_include_common() {}
	/**
	 * Destructor
	 */
	virtual ~domFx_include_common() {}
	/**
	 * Copy Constructor
	 */
	domFx_include_common( const domFx_include_common &cpy ) : daeElement(), domFx_include_common_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_include_common &operator=( const domFx_include_common &cpy ) { (void)cpy; return *this; }

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
