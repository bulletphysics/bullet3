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
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

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
	_Meta->registerConstructor(domProfile_COMMON::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON,elemAsset) );
	mea->setElementType( domAsset::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaChoice( _Meta, cm, 1, 0, -1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "image" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON,elemImage_array) );
	mea->setElementType( domImage::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "newparam" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON,elemNewparam_array) );
	mea->setElementType( domCommon_newparam_type::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	mea = new daeMetaElementAttribute( _Meta, cm, 3002, 1, 1 );
	mea->setName( "technique" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON,elemTechnique) );
	mea->setElementType( domProfile_COMMON::domTechnique::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3003, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 3003 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domProfile_COMMON,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domProfile_COMMON,_contentsOrder));


	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domProfile_COMMON , attrId ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
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
	_Meta->registerConstructor(domProfile_COMMON::domTechnique::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique,elemAsset) );
	mea->setElementType( domAsset::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaChoice( _Meta, cm, 1, 0, -1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "image" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique,elemImage_array) );
	mea->setElementType( domImage::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "newparam" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique,elemNewparam_array) );
	mea->setElementType( domCommon_newparam_type::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	cm = new daeMetaChoice( _Meta, cm, 3002, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "constant" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique,elemConstant) );
	mea->setElementType( domProfile_COMMON::domTechnique::domConstant::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "lambert" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique,elemLambert) );
	mea->setElementType( domProfile_COMMON::domTechnique::domLambert::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "phong" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique,elemPhong) );
	mea->setElementType( domProfile_COMMON::domTechnique::domPhong::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "blinn" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique,elemBlinn) );
	mea->setElementType( domProfile_COMMON::domTechnique::domBlinn::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3003, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 3003 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domProfile_COMMON::domTechnique,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domProfile_COMMON::domTechnique,_contentsOrder));


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
	_Meta->registerConstructor(domProfile_COMMON::domTechnique::domConstant::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "emission" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domConstant,elemEmission) );
	mea->setElementType( domCommon_color_or_texture_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "reflective" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domConstant,elemReflective) );
	mea->setElementType( domCommon_color_or_texture_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "reflectivity" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domConstant,elemReflectivity) );
	mea->setElementType( domCommon_float_or_param_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 3, 0, 1 );
	mea->setName( "transparent" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domConstant,elemTransparent) );
	mea->setElementType( domCommon_transparent_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 4, 0, 1 );
	mea->setName( "transparency" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domConstant,elemTransparency) );
	mea->setElementType( domCommon_float_or_param_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 5, 0, 1 );
	mea->setName( "index_of_refraction" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domConstant,elemIndex_of_refraction) );
	mea->setElementType( domCommon_float_or_param_type::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 5 );
	_Meta->setCMRoot( cm );	
	
	
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
	_Meta->registerConstructor(domProfile_COMMON::domTechnique::domLambert::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "emission" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domLambert,elemEmission) );
	mea->setElementType( domCommon_color_or_texture_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "ambient" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domLambert,elemAmbient) );
	mea->setElementType( domCommon_color_or_texture_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "diffuse" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domLambert,elemDiffuse) );
	mea->setElementType( domCommon_color_or_texture_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 3, 0, 1 );
	mea->setName( "reflective" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domLambert,elemReflective) );
	mea->setElementType( domCommon_color_or_texture_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 4, 0, 1 );
	mea->setName( "reflectivity" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domLambert,elemReflectivity) );
	mea->setElementType( domCommon_float_or_param_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 5, 0, 1 );
	mea->setName( "transparent" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domLambert,elemTransparent) );
	mea->setElementType( domCommon_transparent_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 6, 0, 1 );
	mea->setName( "transparency" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domLambert,elemTransparency) );
	mea->setElementType( domCommon_float_or_param_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 7, 0, 1 );
	mea->setName( "index_of_refraction" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domLambert,elemIndex_of_refraction) );
	mea->setElementType( domCommon_float_or_param_type::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 7 );
	_Meta->setCMRoot( cm );	
	
	
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
	_Meta->registerConstructor(domProfile_COMMON::domTechnique::domPhong::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "emission" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domPhong,elemEmission) );
	mea->setElementType( domCommon_color_or_texture_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "ambient" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domPhong,elemAmbient) );
	mea->setElementType( domCommon_color_or_texture_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "diffuse" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domPhong,elemDiffuse) );
	mea->setElementType( domCommon_color_or_texture_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 3, 0, 1 );
	mea->setName( "specular" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domPhong,elemSpecular) );
	mea->setElementType( domCommon_color_or_texture_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 4, 0, 1 );
	mea->setName( "shininess" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domPhong,elemShininess) );
	mea->setElementType( domCommon_float_or_param_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 5, 0, 1 );
	mea->setName( "reflective" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domPhong,elemReflective) );
	mea->setElementType( domCommon_color_or_texture_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 6, 0, 1 );
	mea->setName( "reflectivity" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domPhong,elemReflectivity) );
	mea->setElementType( domCommon_float_or_param_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 7, 0, 1 );
	mea->setName( "transparent" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domPhong,elemTransparent) );
	mea->setElementType( domCommon_transparent_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 8, 0, 1 );
	mea->setName( "transparency" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domPhong,elemTransparency) );
	mea->setElementType( domCommon_float_or_param_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 9, 0, 1 );
	mea->setName( "index_of_refraction" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domPhong,elemIndex_of_refraction) );
	mea->setElementType( domCommon_float_or_param_type::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 9 );
	_Meta->setCMRoot( cm );	
	
	
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
	_Meta->registerConstructor(domProfile_COMMON::domTechnique::domBlinn::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "emission" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domBlinn,elemEmission) );
	mea->setElementType( domCommon_color_or_texture_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "ambient" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domBlinn,elemAmbient) );
	mea->setElementType( domCommon_color_or_texture_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "diffuse" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domBlinn,elemDiffuse) );
	mea->setElementType( domCommon_color_or_texture_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 3, 0, 1 );
	mea->setName( "specular" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domBlinn,elemSpecular) );
	mea->setElementType( domCommon_color_or_texture_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 4, 0, 1 );
	mea->setName( "shininess" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domBlinn,elemShininess) );
	mea->setElementType( domCommon_float_or_param_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 5, 0, 1 );
	mea->setName( "reflective" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domBlinn,elemReflective) );
	mea->setElementType( domCommon_color_or_texture_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 6, 0, 1 );
	mea->setName( "reflectivity" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domBlinn,elemReflectivity) );
	mea->setElementType( domCommon_float_or_param_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 7, 0, 1 );
	mea->setName( "transparent" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domBlinn,elemTransparent) );
	mea->setElementType( domCommon_transparent_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 8, 0, 1 );
	mea->setName( "transparency" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domBlinn,elemTransparency) );
	mea->setElementType( domCommon_float_or_param_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 9, 0, 1 );
	mea->setName( "index_of_refraction" );
	mea->setOffset( daeOffsetOf(domProfile_COMMON::domTechnique::domBlinn,elemIndex_of_refraction) );
	mea->setElementType( domCommon_float_or_param_type::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 9 );
	_Meta->setCMRoot( cm );	
	
	
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


