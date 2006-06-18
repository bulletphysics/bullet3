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
#include <dom/domInstance_rigid_body.h>

daeElementRef
domInstance_rigid_body::create(daeInt bytes)
{
	domInstance_rigid_bodyRef ref = new(bytes) domInstance_rigid_body;
	ref->attrTarget.setContainer( (domInstance_rigid_body*)ref );
	return ref;
}


daeMetaElement *
domInstance_rigid_body::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "instance_rigid_body" );
	_Meta->setStaticPointerAddress(&domInstance_rigid_body::_Meta);
	_Meta->registerConstructor(domInstance_rigid_body::create);

	// Add elements: technique_common, technique, extra
    _Meta->appendElement(domInstance_rigid_body::domTechnique_common::registerElement(),daeOffsetOf(domInstance_rigid_body,elemTechnique_common));
    _Meta->appendArrayElement(domTechnique::registerElement(),daeOffsetOf(domInstance_rigid_body,elemTechnique_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domInstance_rigid_body,elemExtra_array));

	//	Add attribute: body
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "body" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_rigid_body , attrBody ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_rigid_body , attrSid ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: target
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "target" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domInstance_rigid_body , attrTarget ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domInstance_rigid_body));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domInstance_rigid_body::domTechnique_common::create(daeInt bytes)
{
	domInstance_rigid_body::domTechnique_commonRef ref = new(bytes) domInstance_rigid_body::domTechnique_common;
	return ref;
}


daeMetaElement *
domInstance_rigid_body::domTechnique_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "technique_common" );
	_Meta->setStaticPointerAddress(&domInstance_rigid_body::domTechnique_common::_Meta);
	_Meta->registerConstructor(domInstance_rigid_body::domTechnique_common::create);

	// Add elements: angular_velocity, velocity, dynamic, mass, mass_frame, inertia, instance_physics_material, physics_material, shape
    _Meta->appendElement(domInstance_rigid_body::domTechnique_common::domAngular_velocity::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common,elemAngular_velocity));
    _Meta->appendElement(domInstance_rigid_body::domTechnique_common::domVelocity::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common,elemVelocity));
    _Meta->appendElement(domInstance_rigid_body::domTechnique_common::domDynamic::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common,elemDynamic));
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common,elemMass),"mass"); 
    _Meta->appendElement(domInstance_rigid_body::domTechnique_common::domMass_frame::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common,elemMass_frame));
    _Meta->appendElement(domTargetableFloat3::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common,elemInertia),"inertia"); 
    _Meta->appendElement(domInstance_physics_material::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common,elemInstance_physics_material));
    _Meta->appendElement(domPhysics_material::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common,elemPhysics_material));
    _Meta->appendArrayElement(domInstance_rigid_body::domTechnique_common::domShape::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common,elemShape_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domInstance_rigid_body::domTechnique_common,_contents));

	
	
	_Meta->setElementSize(sizeof(domInstance_rigid_body::domTechnique_common));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domInstance_rigid_body::domTechnique_common::domAngular_velocity::create(daeInt bytes)
{
	domInstance_rigid_body::domTechnique_common::domAngular_velocityRef ref = new(bytes) domInstance_rigid_body::domTechnique_common::domAngular_velocity;
	return ref;
}


daeMetaElement *
domInstance_rigid_body::domTechnique_common::domAngular_velocity::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "angular_velocity" );
	_Meta->setStaticPointerAddress(&domInstance_rigid_body::domTechnique_common::domAngular_velocity::_Meta);
	_Meta->registerConstructor(domInstance_rigid_body::domTechnique_common::domAngular_velocity::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float3"));
		ma->setOffset( daeOffsetOf( domInstance_rigid_body::domTechnique_common::domAngular_velocity , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domInstance_rigid_body::domTechnique_common::domAngular_velocity));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domInstance_rigid_body::domTechnique_common::domVelocity::create(daeInt bytes)
{
	domInstance_rigid_body::domTechnique_common::domVelocityRef ref = new(bytes) domInstance_rigid_body::domTechnique_common::domVelocity;
	return ref;
}


daeMetaElement *
domInstance_rigid_body::domTechnique_common::domVelocity::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "velocity" );
	_Meta->setStaticPointerAddress(&domInstance_rigid_body::domTechnique_common::domVelocity::_Meta);
	_Meta->registerConstructor(domInstance_rigid_body::domTechnique_common::domVelocity::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float3"));
		ma->setOffset( daeOffsetOf( domInstance_rigid_body::domTechnique_common::domVelocity , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domInstance_rigid_body::domTechnique_common::domVelocity));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domInstance_rigid_body::domTechnique_common::domDynamic::create(daeInt bytes)
{
	domInstance_rigid_body::domTechnique_common::domDynamicRef ref = new(bytes) domInstance_rigid_body::domTechnique_common::domDynamic;
	return ref;
}


daeMetaElement *
domInstance_rigid_body::domTechnique_common::domDynamic::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "dynamic" );
	_Meta->setStaticPointerAddress(&domInstance_rigid_body::domTechnique_common::domDynamic::_Meta);
	_Meta->registerConstructor(domInstance_rigid_body::domTechnique_common::domDynamic::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domInstance_rigid_body::domTechnique_common::domDynamic , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_rigid_body::domTechnique_common::domDynamic , attrSid ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domInstance_rigid_body::domTechnique_common::domDynamic));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domInstance_rigid_body::domTechnique_common::domMass_frame::create(daeInt bytes)
{
	domInstance_rigid_body::domTechnique_common::domMass_frameRef ref = new(bytes) domInstance_rigid_body::domTechnique_common::domMass_frame;
	return ref;
}


daeMetaElement *
domInstance_rigid_body::domTechnique_common::domMass_frame::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "mass_frame" );
	_Meta->setStaticPointerAddress(&domInstance_rigid_body::domTechnique_common::domMass_frame::_Meta);
	_Meta->registerConstructor(domInstance_rigid_body::domTechnique_common::domMass_frame::create);

	// Add elements: translate, rotate
    _Meta->appendArrayElement(domTranslate::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common::domMass_frame,elemTranslate_array));
    _Meta->appendArrayElement(domRotate::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common::domMass_frame,elemRotate_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domInstance_rigid_body::domTechnique_common::domMass_frame,_contents));

	
	
	_Meta->setElementSize(sizeof(domInstance_rigid_body::domTechnique_common::domMass_frame));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domInstance_rigid_body::domTechnique_common::domShape::create(daeInt bytes)
{
	domInstance_rigid_body::domTechnique_common::domShapeRef ref = new(bytes) domInstance_rigid_body::domTechnique_common::domShape;
	return ref;
}


daeMetaElement *
domInstance_rigid_body::domTechnique_common::domShape::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "shape" );
	_Meta->setStaticPointerAddress(&domInstance_rigid_body::domTechnique_common::domShape::_Meta);
	_Meta->registerConstructor(domInstance_rigid_body::domTechnique_common::domShape::create);

	// Add elements: hollow, mass, density, instance_physics_material, physics_material, instance_geometry, plane, box, sphere, cylinder, tapered_cylinder, capsule, tapered_capsule, translate, rotate, extra
    _Meta->appendElement(domInstance_rigid_body::domTechnique_common::domShape::domHollow::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemHollow));
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemMass),"mass"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemDensity),"density"); 
    _Meta->appendElement(domInstance_physics_material::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemInstance_physics_material));
    _Meta->appendElement(domPhysics_material::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemPhysics_material));
    _Meta->appendElement(domInstance_geometry::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemInstance_geometry));
    _Meta->appendElement(domPlane::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemPlane));
    _Meta->appendElement(domBox::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemBox));
    _Meta->appendElement(domSphere::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemSphere));
    _Meta->appendElement(domCylinder::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemCylinder));
    _Meta->appendElement(domTapered_cylinder::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemTapered_cylinder));
    _Meta->appendElement(domCapsule::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemCapsule));
    _Meta->appendElement(domTapered_capsule::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemTapered_capsule));
    _Meta->appendArrayElement(domTranslate::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemTranslate_array));
    _Meta->appendArrayElement(domRotate::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemRotate_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemExtra_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,_contents));

	
	
	_Meta->setElementSize(sizeof(domInstance_rigid_body::domTechnique_common::domShape));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domInstance_rigid_body::domTechnique_common::domShape::domHollow::create(daeInt bytes)
{
	domInstance_rigid_body::domTechnique_common::domShape::domHollowRef ref = new(bytes) domInstance_rigid_body::domTechnique_common::domShape::domHollow;
	return ref;
}


daeMetaElement *
domInstance_rigid_body::domTechnique_common::domShape::domHollow::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "hollow" );
	_Meta->setStaticPointerAddress(&domInstance_rigid_body::domTechnique_common::domShape::domHollow::_Meta);
	_Meta->registerConstructor(domInstance_rigid_body::domTechnique_common::domShape::domHollow::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domInstance_rigid_body::domTechnique_common::domShape::domHollow , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_rigid_body::domTechnique_common::domShape::domHollow , attrSid ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domInstance_rigid_body::domTechnique_common::domShape::domHollow));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domInstance_rigid_body::_Meta = NULL;
daeMetaElement * domInstance_rigid_body::domTechnique_common::_Meta = NULL;
daeMetaElement * domInstance_rigid_body::domTechnique_common::domAngular_velocity::_Meta = NULL;
daeMetaElement * domInstance_rigid_body::domTechnique_common::domVelocity::_Meta = NULL;
daeMetaElement * domInstance_rigid_body::domTechnique_common::domDynamic::_Meta = NULL;
daeMetaElement * domInstance_rigid_body::domTechnique_common::domMass_frame::_Meta = NULL;
daeMetaElement * domInstance_rigid_body::domTechnique_common::domShape::_Meta = NULL;
daeMetaElement * domInstance_rigid_body::domTechnique_common::domShape::domHollow::_Meta = NULL;


