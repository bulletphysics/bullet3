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
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

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
	_Meta->registerConstructor(domInstance_rigid_body::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "technique_common" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body,elemTechnique_common) );
	mea->setElementType( domInstance_rigid_body::domTechnique_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 1, 0, -1 );
	mea->setName( "technique" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body,elemTechnique_array) );
	mea->setElementType( domTechnique::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 2 );
	_Meta->setCMRoot( cm );	

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

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_rigid_body , attrName ));
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
	_Meta->registerConstructor(domInstance_rigid_body::domTechnique_common::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "angular_velocity" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common,elemAngular_velocity) );
	mea->setElementType( domInstance_rigid_body::domTechnique_common::domAngular_velocity::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "velocity" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common,elemVelocity) );
	mea->setElementType( domInstance_rigid_body::domTechnique_common::domVelocity::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "dynamic" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common,elemDynamic) );
	mea->setElementType( domInstance_rigid_body::domTechnique_common::domDynamic::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 3, 0, 1 );
	mea->setName( "mass" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common,elemMass) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 4, 0, 1 );
	mea->setName( "mass_frame" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common,elemMass_frame) );
	mea->setElementType( domInstance_rigid_body::domTechnique_common::domMass_frame::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 5, 0, 1 );
	mea->setName( "inertia" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common,elemInertia) );
	mea->setElementType( domTargetableFloat3::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaChoice( _Meta, cm, 6, 0, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "instance_physics_material" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common,elemInstance_physics_material) );
	mea->setElementType( domInstance_physics_material::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "physics_material" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common,elemPhysics_material) );
	mea->setElementType( domPhysics_material::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 7, 0, -1 );
	mea->setName( "shape" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common,elemShape_array) );
	mea->setElementType( domInstance_rigid_body::domTechnique_common::domShape::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 7 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domInstance_rigid_body::domTechnique_common,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domInstance_rigid_body::domTechnique_common,_contentsOrder));

	
	
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
	_Meta->registerConstructor(domInstance_rigid_body::domTechnique_common::domAngular_velocity::create);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerConstructor(domInstance_rigid_body::domTechnique_common::domVelocity::create);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerConstructor(domInstance_rigid_body::domTechnique_common::domDynamic::create);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerConstructor(domInstance_rigid_body::domTechnique_common::domMass_frame::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( _Meta, cm, 0, 1, -1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "translate" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common::domMass_frame,elemTranslate_array) );
	mea->setElementType( domTranslate::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "rotate" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common::domMass_frame,elemRotate_array) );
	mea->setElementType( domRotate::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domInstance_rigid_body::domTechnique_common::domMass_frame,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domInstance_rigid_body::domTechnique_common::domMass_frame,_contentsOrder));

	
	
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
	_Meta->registerConstructor(domInstance_rigid_body::domTechnique_common::domShape::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "hollow" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemHollow) );
	mea->setElementType( domInstance_rigid_body::domTechnique_common::domShape::domHollow::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "mass" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemMass) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "density" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemDensity) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaChoice( _Meta, cm, 3, 0, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "instance_physics_material" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemInstance_physics_material) );
	mea->setElementType( domInstance_physics_material::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "physics_material" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemPhysics_material) );
	mea->setElementType( domPhysics_material::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	cm = new daeMetaChoice( _Meta, cm, 4, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "instance_geometry" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemInstance_geometry) );
	mea->setElementType( domInstance_geometry::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "plane" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemPlane) );
	mea->setElementType( domPlane::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "box" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemBox) );
	mea->setElementType( domBox::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "sphere" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemSphere) );
	mea->setElementType( domSphere::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "cylinder" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemCylinder) );
	mea->setElementType( domCylinder::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "tapered_cylinder" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemTapered_cylinder) );
	mea->setElementType( domTapered_cylinder::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "capsule" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemCapsule) );
	mea->setElementType( domCapsule::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "tapered_capsule" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemTapered_capsule) );
	mea->setElementType( domTapered_capsule::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	cm = new daeMetaChoice( _Meta, cm, 5, 0, -1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "translate" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemTranslate_array) );
	mea->setElementType( domTranslate::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "rotate" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemRotate_array) );
	mea->setElementType( domRotate::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3006, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 3006 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domInstance_rigid_body::domTechnique_common::domShape,_contentsOrder));

	
	
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
	_Meta->registerConstructor(domInstance_rigid_body::domTechnique_common::domShape::domHollow::create);

	_Meta->setIsInnerClass( true );
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


