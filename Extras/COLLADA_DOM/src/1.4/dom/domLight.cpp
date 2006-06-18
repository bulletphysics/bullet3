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
	_Meta->setStaticPointerAddress(&domLight::_Meta);
	_Meta->registerConstructor(domLight::create);

	// Add elements: asset, technique_common, technique, extra
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domLight,elemAsset));
    _Meta->appendElement(domLight::domTechnique_common::registerElement(),daeOffsetOf(domLight,elemTechnique_common));
    _Meta->appendArrayElement(domTechnique::registerElement(),daeOffsetOf(domLight,elemTechnique_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domLight,elemExtra_array));

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
	_Meta->setStaticPointerAddress(&domLight::domTechnique_common::_Meta);
	_Meta->registerConstructor(domLight::domTechnique_common::create);

	// Add elements: ambient, directional, point, spot
    _Meta->appendElement(domLight::domTechnique_common::domAmbient::registerElement(),daeOffsetOf(domLight::domTechnique_common,elemAmbient));
    _Meta->appendElement(domLight::domTechnique_common::domDirectional::registerElement(),daeOffsetOf(domLight::domTechnique_common,elemDirectional));
    _Meta->appendElement(domLight::domTechnique_common::domPoint::registerElement(),daeOffsetOf(domLight::domTechnique_common,elemPoint));
    _Meta->appendElement(domLight::domTechnique_common::domSpot::registerElement(),daeOffsetOf(domLight::domTechnique_common,elemSpot));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domLight::domTechnique_common,_contents));

	
	
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
	_Meta->setStaticPointerAddress(&domLight::domTechnique_common::domAmbient::_Meta);
	_Meta->registerConstructor(domLight::domTechnique_common::domAmbient::create);

	// Add elements: color
    _Meta->appendElement(domTargetableFloat3::registerElement(),daeOffsetOf(domLight::domTechnique_common::domAmbient,elemColor),"color"); 
	
	
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
	_Meta->setStaticPointerAddress(&domLight::domTechnique_common::domDirectional::_Meta);
	_Meta->registerConstructor(domLight::domTechnique_common::domDirectional::create);

	// Add elements: color
    _Meta->appendElement(domTargetableFloat3::registerElement(),daeOffsetOf(domLight::domTechnique_common::domDirectional,elemColor),"color"); 
	
	
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
	_Meta->setStaticPointerAddress(&domLight::domTechnique_common::domPoint::_Meta);
	_Meta->registerConstructor(domLight::domTechnique_common::domPoint::create);

	// Add elements: color, constant_attenuation, linear_attenuation, quadratic_attenuation
    _Meta->appendElement(domTargetableFloat3::registerElement(),daeOffsetOf(domLight::domTechnique_common::domPoint,elemColor),"color"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domLight::domTechnique_common::domPoint,elemConstant_attenuation),"constant_attenuation"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domLight::domTechnique_common::domPoint,elemLinear_attenuation),"linear_attenuation"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domLight::domTechnique_common::domPoint,elemQuadratic_attenuation),"quadratic_attenuation"); 
	
	
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
	_Meta->setStaticPointerAddress(&domLight::domTechnique_common::domSpot::_Meta);
	_Meta->registerConstructor(domLight::domTechnique_common::domSpot::create);

	// Add elements: color, constant_attenuation, linear_attenuation, quadratic_attenuation, falloff_angle, falloff_exponent
    _Meta->appendElement(domTargetableFloat3::registerElement(),daeOffsetOf(domLight::domTechnique_common::domSpot,elemColor),"color"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domLight::domTechnique_common::domSpot,elemConstant_attenuation),"constant_attenuation"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domLight::domTechnique_common::domSpot,elemLinear_attenuation),"linear_attenuation"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domLight::domTechnique_common::domSpot,elemQuadratic_attenuation),"quadratic_attenuation"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domLight::domTechnique_common::domSpot,elemFalloff_angle),"falloff_angle"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domLight::domTechnique_common::domSpot,elemFalloff_exponent),"falloff_exponent"); 
	
	
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


