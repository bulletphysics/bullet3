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
#include <dom/domLight.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domLight::create(daeInt bytes)
{
	domLightRef ref = new(bytes) domLight;
	return ref;
}


daeMetaElement *
domLight::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light" );
	_Meta->registerConstructor(domLight::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domLight,elemAsset) );
	mea->setElementType( domAsset::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 1, 1 );
	mea->setName( "technique_common" );
	mea->setOffset( daeOffsetOf(domLight,elemTechnique_common) );
	mea->setElementType( domLight::domTechnique_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 2, 0, -1 );
	mea->setName( "technique" );
	mea->setOffset( daeOffsetOf(domLight,elemTechnique_array) );
	mea->setElementType( domTechnique::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domLight,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 3 );
	_Meta->setCMRoot( cm );	

	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domLight , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domLight , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domLight));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domLight::domTechnique_common::create(daeInt bytes)
{
	domLight::domTechnique_commonRef ref = new(bytes) domLight::domTechnique_common;
	return ref;
}


daeMetaElement *
domLight::domTechnique_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "technique_common" );
	_Meta->registerConstructor(domLight::domTechnique_common::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "ambient" );
	mea->setOffset( daeOffsetOf(domLight::domTechnique_common,elemAmbient) );
	mea->setElementType( domLight::domTechnique_common::domAmbient::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "directional" );
	mea->setOffset( daeOffsetOf(domLight::domTechnique_common,elemDirectional) );
	mea->setElementType( domLight::domTechnique_common::domDirectional::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "point" );
	mea->setOffset( daeOffsetOf(domLight::domTechnique_common,elemPoint) );
	mea->setElementType( domLight::domTechnique_common::domPoint::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "spot" );
	mea->setOffset( daeOffsetOf(domLight::domTechnique_common,elemSpot) );
	mea->setElementType( domLight::domTechnique_common::domSpot::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domLight::domTechnique_common,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domLight::domTechnique_common,_contentsOrder));

	
	
	_Meta->setElementSize(sizeof(domLight::domTechnique_common));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domLight::domTechnique_common::domAmbient::create(daeInt bytes)
{
	domLight::domTechnique_common::domAmbientRef ref = new(bytes) domLight::domTechnique_common::domAmbient;
	return ref;
}


daeMetaElement *
domLight::domTechnique_common::domAmbient::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "ambient" );
	_Meta->registerConstructor(domLight::domTechnique_common::domAmbient::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "color" );
	mea->setOffset( daeOffsetOf(domLight::domTechnique_common::domAmbient,elemColor) );
	mea->setElementType( domTargetableFloat3::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domLight::domTechnique_common::domAmbient));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domLight::domTechnique_common::domDirectional::create(daeInt bytes)
{
	domLight::domTechnique_common::domDirectionalRef ref = new(bytes) domLight::domTechnique_common::domDirectional;
	return ref;
}


daeMetaElement *
domLight::domTechnique_common::domDirectional::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "directional" );
	_Meta->registerConstructor(domLight::domTechnique_common::domDirectional::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "color" );
	mea->setOffset( daeOffsetOf(domLight::domTechnique_common::domDirectional,elemColor) );
	mea->setElementType( domTargetableFloat3::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domLight::domTechnique_common::domDirectional));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domLight::domTechnique_common::domPoint::create(daeInt bytes)
{
	domLight::domTechnique_common::domPointRef ref = new(bytes) domLight::domTechnique_common::domPoint;
	return ref;
}


daeMetaElement *
domLight::domTechnique_common::domPoint::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "point" );
	_Meta->registerConstructor(domLight::domTechnique_common::domPoint::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "color" );
	mea->setOffset( daeOffsetOf(domLight::domTechnique_common::domPoint,elemColor) );
	mea->setElementType( domTargetableFloat3::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "constant_attenuation" );
	mea->setOffset( daeOffsetOf(domLight::domTechnique_common::domPoint,elemConstant_attenuation) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "linear_attenuation" );
	mea->setOffset( daeOffsetOf(domLight::domTechnique_common::domPoint,elemLinear_attenuation) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 3, 0, 1 );
	mea->setName( "quadratic_attenuation" );
	mea->setOffset( daeOffsetOf(domLight::domTechnique_common::domPoint,elemQuadratic_attenuation) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 3 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domLight::domTechnique_common::domPoint));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domLight::domTechnique_common::domSpot::create(daeInt bytes)
{
	domLight::domTechnique_common::domSpotRef ref = new(bytes) domLight::domTechnique_common::domSpot;
	return ref;
}


daeMetaElement *
domLight::domTechnique_common::domSpot::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "spot" );
	_Meta->registerConstructor(domLight::domTechnique_common::domSpot::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "color" );
	mea->setOffset( daeOffsetOf(domLight::domTechnique_common::domSpot,elemColor) );
	mea->setElementType( domTargetableFloat3::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "constant_attenuation" );
	mea->setOffset( daeOffsetOf(domLight::domTechnique_common::domSpot,elemConstant_attenuation) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "linear_attenuation" );
	mea->setOffset( daeOffsetOf(domLight::domTechnique_common::domSpot,elemLinear_attenuation) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 3, 0, 1 );
	mea->setName( "quadratic_attenuation" );
	mea->setOffset( daeOffsetOf(domLight::domTechnique_common::domSpot,elemQuadratic_attenuation) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 4, 0, 1 );
	mea->setName( "falloff_angle" );
	mea->setOffset( daeOffsetOf(domLight::domTechnique_common::domSpot,elemFalloff_angle) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 5, 0, 1 );
	mea->setName( "falloff_exponent" );
	mea->setOffset( daeOffsetOf(domLight::domTechnique_common::domSpot,elemFalloff_exponent) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 5 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domLight::domTechnique_common::domSpot));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domLight::_Meta = NULL;
daeMetaElement * domLight::domTechnique_common::_Meta = NULL;
daeMetaElement * domLight::domTechnique_common::domAmbient::_Meta = NULL;
daeMetaElement * domLight::domTechnique_common::domDirectional::_Meta = NULL;
daeMetaElement * domLight::domTechnique_common::domPoint::_Meta = NULL;
daeMetaElement * domLight::domTechnique_common::domSpot::_Meta = NULL;


