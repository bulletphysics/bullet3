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
#ifndef __domChannel_h__
#define __domChannel_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>


/**
 * The channel element declares an output channel of an animation.
 */
class domChannel : public daeElement
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::CHANNEL; }
protected:  // Attributes
/**
 *  The source attribute indicates the location of the sampler using a URL
 * expression.  The sampler must be declared within the same document. Required
 * attribute. 
 */
	domURIFragmentType attrSource;
/**
 *  The target attribute indicates the location of the element bound to the
 * output of the sampler.  This text string is a path-name following a simple
 * syntax described in Address Syntax.  Required attribute. 
 */
	xsToken attrTarget;


public:	//Accessors and Mutators
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
	domChannel() : attrSource(), attrTarget() {}
	/**
	 * Destructor
	 */
	virtual ~domChannel() {}
	/**
	 * Copy Constructor
	 */
	domChannel( const domChannel &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domChannel &operator=( const domChannel &cpy ) { (void)cpy; return *this; }

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
