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
#ifndef __domGl_samplerCUBE_h__
#define __domGl_samplerCUBE_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domFx_samplerCUBE_common.h>

/**
 * A cube map texture sampler for the GLSL profile.
 */
class domGl_samplerCUBE_complexType : public domFx_samplerCUBE_common_complexType
{

protected:
	/**
	 * Constructor
	 */
	domGl_samplerCUBE_complexType() {}
	/**
	 * Destructor
	 */
	virtual ~domGl_samplerCUBE_complexType() {}
	/**
	 * Copy Constructor
	 */
	domGl_samplerCUBE_complexType( const domGl_samplerCUBE_complexType &cpy ) : domFx_samplerCUBE_common_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGl_samplerCUBE_complexType &operator=( const domGl_samplerCUBE_complexType &cpy ) { (void)cpy; return *this; }
};

/**
 * An element of type domGl_samplerCUBE_complexType.
 */
class domGl_samplerCUBE : public daeElement, public domGl_samplerCUBE_complexType
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::GL_SAMPLERCUBE; }
protected:
	/**
	 * Constructor
	 */
	domGl_samplerCUBE() {}
	/**
	 * Destructor
	 */
	virtual ~domGl_samplerCUBE() {}
	/**
	 * Copy Constructor
	 */
	domGl_samplerCUBE( const domGl_samplerCUBE &cpy ) : daeElement(), domGl_samplerCUBE_complexType() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domGl_samplerCUBE &operator=( const domGl_samplerCUBE &cpy ) { (void)cpy; return *this; }

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
