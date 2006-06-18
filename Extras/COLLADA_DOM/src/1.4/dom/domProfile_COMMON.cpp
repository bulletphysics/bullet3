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

#include <dae/daeDom.h>
#include <dom/domProfile_COMMON.h>

daeElementRef
domProfile_COMMON::create(daeInt bytes)
{
	domProfile_COMMONRef ref = new(bytes) domProfile_COMMON;
	return ref;
}


daeMetaElement *
domProfile_COMMON::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "profile_COMMON" );
	_Meta->setStaticPointerAddress(&domProfile_COMMON::_Meta);
	_Meta->registerConstructor(domProfile_COMMON::create);

	// Add elements: image, newparam, technique, extra
    _Meta->appendArrayElement(domImage::registerElement(),daeOffsetOf(domProfile_COMMON,elemImage_array));
    _Meta->appendArrayElement(domCommon_newparam_type::registerElement(),daeOffsetOf(domProfile_COMMON,elemNewparam_array),"newparam"); 
    _Meta->appendElement(domProfile_COMMON::domTechnique::registerElement(),daeOffsetOf(domProfile_COMMON,elemTechnique));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domProfile_COMMON,elemExtra_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domProfile_COMMON,_contents));

	
	
	_Meta->setElementSize(sizeof(domProfile_COMMON));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_COMMON::domTechnique::create(daeInt bytes)
{
	domProfile_COMMON::domTechniqueRef ref = new(bytes) domProfile_COMMON::domTechnique;
	return ref;
}


daeMetaElement *
domProfile_COMMON::domTechnique::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "technique" );
	_Meta->setStaticPointerAddress(&domProfile_COMMON::domTechnique::_Meta);
	_Meta->registerConstructor(domProfile_COMMON::domTechnique::create);

	// Add elements: asset, image, newparam, constant, lambert, phong, blinn, extra
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique,elemAsset));
    _Meta->appendArrayElement(domImage::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique,elemImage_array));
    _Meta->appendArrayElement(domCommon_newparam_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique,elemNewparam_array),"newparam"); 
    _Meta->appendElement(domProfile_COMMON::domTechnique::domConstant::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique,elemConstant));
    _Meta->appendElement(domProfile_COMMON::domTechnique::domLambert::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique,elemLambert));
    _Meta->appendElement(domProfile_COMMON::domTechnique::domPhong::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique,elemPhong));
    _Meta->appendElement(domProfile_COMMON::domTechnique::domBlinn::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique,elemBlinn));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique,elemExtra_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domProfile_COMMON::domTechnique,_contents));


	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domProfile_COMMON::domTechnique , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_COMMON::domTechnique , attrSid ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_COMMON::domTechnique));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_COMMON::domTechnique::domConstant::create(daeInt bytes)
{
	domProfile_COMMON::domTechnique::domConstantRef ref = new(bytes) domProfile_COMMON::domTechnique::domConstant;
	return ref;
}


daeMetaElement *
domProfile_COMMON::domTechnique::domConstant::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "constant" );
	_Meta->setStaticPointerAddress(&domProfile_COMMON::domTechnique::domConstant::_Meta);
	_Meta->registerConstructor(domProfile_COMMON::domTechnique::domConstant::create);

	// Add elements: emission, reflective, reflectivity, transparent, transparency, index_of_refraction
    _Meta->appendElement(domCommon_color_or_texture_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domConstant,elemEmission),"emission"); 
    _Meta->appendElement(domCommon_color_or_texture_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domConstant,elemReflective),"reflective"); 
    _Meta->appendElement(domCommon_float_or_param_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domConstant,elemReflectivity),"reflectivity"); 
    _Meta->appendElement(domCommon_color_or_texture_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domConstant,elemTransparent),"transparent"); 
    _Meta->appendElement(domCommon_float_or_param_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domConstant,elemTransparency),"transparency"); 
    _Meta->appendElement(domCommon_float_or_param_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domConstant,elemIndex_of_refraction),"index_of_refraction"); 
	
	
	_Meta->setElementSize(sizeof(domProfile_COMMON::domTechnique::domConstant));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_COMMON::domTechnique::domLambert::create(daeInt bytes)
{
	domProfile_COMMON::domTechnique::domLambertRef ref = new(bytes) domProfile_COMMON::domTechnique::domLambert;
	return ref;
}


daeMetaElement *
domProfile_COMMON::domTechnique::domLambert::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "lambert" );
	_Meta->setStaticPointerAddress(&domProfile_COMMON::domTechnique::domLambert::_Meta);
	_Meta->registerConstructor(domProfile_COMMON::domTechnique::domLambert::create);

	// Add elements: emission, ambient, diffuse, reflective, reflectivity, transparent, transparency, index_of_refraction
    _Meta->appendElement(domCommon_color_or_texture_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domLambert,elemEmission),"emission"); 
    _Meta->appendElement(domCommon_color_or_texture_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domLambert,elemAmbient),"ambient"); 
    _Meta->appendElement(domCommon_color_or_texture_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domLambert,elemDiffuse),"diffuse"); 
    _Meta->appendElement(domCommon_color_or_texture_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domLambert,elemReflective),"reflective"); 
    _Meta->appendElement(domCommon_float_or_param_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domLambert,elemReflectivity),"reflectivity"); 
    _Meta->appendElement(domCommon_color_or_texture_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domLambert,elemTransparent),"transparent"); 
    _Meta->appendElement(domCommon_float_or_param_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domLambert,elemTransparency),"transparency"); 
    _Meta->appendElement(domCommon_float_or_param_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domLambert,elemIndex_of_refraction),"index_of_refraction"); 
	
	
	_Meta->setElementSize(sizeof(domProfile_COMMON::domTechnique::domLambert));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_COMMON::domTechnique::domPhong::create(daeInt bytes)
{
	domProfile_COMMON::domTechnique::domPhongRef ref = new(bytes) domProfile_COMMON::domTechnique::domPhong;
	return ref;
}


daeMetaElement *
domProfile_COMMON::domTechnique::domPhong::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "phong" );
	_Meta->setStaticPointerAddress(&domProfile_COMMON::domTechnique::domPhong::_Meta);
	_Meta->registerConstructor(domProfile_COMMON::domTechnique::domPhong::create);

	// Add elements: emission, ambient, diffuse, specular, shininess, reflective, reflectivity, transparent, transparency, index_of_refraction
    _Meta->appendElement(domCommon_color_or_texture_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domPhong,elemEmission),"emission"); 
    _Meta->appendElement(domCommon_color_or_texture_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domPhong,elemAmbient),"ambient"); 
    _Meta->appendElement(domCommon_color_or_texture_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domPhong,elemDiffuse),"diffuse"); 
    _Meta->appendElement(domCommon_color_or_texture_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domPhong,elemSpecular),"specular"); 
    _Meta->appendElement(domCommon_float_or_param_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domPhong,elemShininess),"shininess"); 
    _Meta->appendElement(domCommon_color_or_texture_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domPhong,elemReflective),"reflective"); 
    _Meta->appendElement(domCommon_float_or_param_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domPhong,elemReflectivity),"reflectivity"); 
    _Meta->appendElement(domCommon_color_or_texture_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domPhong,elemTransparent),"transparent"); 
    _Meta->appendElement(domCommon_float_or_param_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domPhong,elemTransparency),"transparency"); 
    _Meta->appendElement(domCommon_float_or_param_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domPhong,elemIndex_of_refraction),"index_of_refraction"); 
	
	
	_Meta->setElementSize(sizeof(domProfile_COMMON::domTechnique::domPhong));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_COMMON::domTechnique::domBlinn::create(daeInt bytes)
{
	domProfile_COMMON::domTechnique::domBlinnRef ref = new(bytes) domProfile_COMMON::domTechnique::domBlinn;
	return ref;
}


daeMetaElement *
domProfile_COMMON::domTechnique::domBlinn::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "blinn" );
	_Meta->setStaticPointerAddress(&domProfile_COMMON::domTechnique::domBlinn::_Meta);
	_Meta->registerConstructor(domProfile_COMMON::domTechnique::domBlinn::create);

	// Add elements: emission, ambient, diffuse, specular, shininess, reflective, reflectivity, transparent, transparency, index_of_refraction
    _Meta->appendElement(domCommon_color_or_texture_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domBlinn,elemEmission),"emission"); 
    _Meta->appendElement(domCommon_color_or_texture_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domBlinn,elemAmbient),"ambient"); 
    _Meta->appendElement(domCommon_color_or_texture_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domBlinn,elemDiffuse),"diffuse"); 
    _Meta->appendElement(domCommon_color_or_texture_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domBlinn,elemSpecular),"specular"); 
    _Meta->appendElement(domCommon_float_or_param_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domBlinn,elemShininess),"shininess"); 
    _Meta->appendElement(domCommon_color_or_texture_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domBlinn,elemReflective),"reflective"); 
    _Meta->appendElement(domCommon_float_or_param_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domBlinn,elemReflectivity),"reflectivity"); 
    _Meta->appendElement(domCommon_color_or_texture_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domBlinn,elemTransparent),"transparent"); 
    _Meta->appendElement(domCommon_float_or_param_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domBlinn,elemTransparency),"transparency"); 
    _Meta->appendElement(domCommon_float_or_param_type::registerElement(),daeOffsetOf(domProfile_COMMON::domTechnique::domBlinn,elemIndex_of_refraction),"index_of_refraction"); 
	
	
	_Meta->setElementSize(sizeof(domProfile_COMMON::domTechnique::domBlinn));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domProfile_COMMON::_Meta = NULL;
daeMetaElement * domProfile_COMMON::domTechnique::_Meta = NULL;
daeMetaElement * domProfile_COMMON::domTechnique::domConstant::_Meta = NULL;
daeMetaElement * domProfile_COMMON::domTechnique::domLambert::_Meta = NULL;
daeMetaElement * domProfile_COMMON::domTechnique::domPhong::_Meta = NULL;
daeMetaElement * domProfile_COMMON::domTechnique::domBlinn::_Meta = NULL;


