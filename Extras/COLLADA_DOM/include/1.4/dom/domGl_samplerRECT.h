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
#ifndef __domGl_samplerRECT_h__
#define __domGl_samplerRECT_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domFx_samplerRECT_common.h>

/**
 * A two-dimensional texture sampler for the GLSL profile.
 */
class domGl_samplerRECT_complexType : public domFx_samplerRECT_common_complexType
{

protected:
	/**
	 * Constructor
	 */
	domGl_samplerRECT_complexType() {}
	/**
	 * Destructor
	 */
	virtual ~domGl_samplerRECT_complexType() {}
	/**
	 * Copy Constructor
	 */
	domGl_samplerRECT_complexType( const domGl_samplerRECT_complexType &cpy ) : domFx_samplerRECT_common_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGl_samplerRECT_complexType &operator=( const domGl_samplerRECT_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domGl_samplerRECT_complexType.
 */
class domGl_samplerRECT : public daeElement, public domGl_samplerRECT_complexType
{
protected:
	/**
	 * Constructor
	 */
	domGl_samplerRECT() {}
	/**
	 * Destructor
	 */
	virtual ~domGl_samplerRECT() {}
	/**
	 * Copy Constructor
	 */
	domGl_samplerRECT( const domGl_samplerRECT &cpy ) : daeElement(), domGl_samplerRECT_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGl_samplerRECT &operator=( const domGl_samplerRECT &cpy ) { (void)cpy; return *this; }

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
