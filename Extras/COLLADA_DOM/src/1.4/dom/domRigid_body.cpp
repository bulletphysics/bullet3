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
#include <dom/domRigid_body.h>

daeElementRef
domRigid_body::create(daeInt bytes)
{
	domRigid_bodyRef ref = new(bytes) domRigid_body;
	return ref;
}


daeMetaElement *
domRigid_body::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "rigid_body" );
	_Meta->setStaticPointerAddress(&domRigid_body::_Meta);
	_Meta->registerConstructor(domRigid_body::create);

	// Add elements: technique_common, technique, extra
    _Meta->appendElement(domRigid_body::domTechnique_common::registerElement(),daeOffsetOf(domRigid_body,elemTechnique_common));
    _Meta->appendArrayElement(domTechnique::registerElement(),daeOffsetOf(domRigid_body,elemTechnique_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domRigid_body,elemExtra_array));

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domRigid_body , attrSid ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domRigid_body , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domRigid_body));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domRigid_body::domTechnique_common::create(daeInt bytes)
{
	domRigid_body::domTechnique_commonRef ref = new(bytes) domRigid_body::domTechnique_common;
	return ref;
}


daeMetaElement *
domRigid_body::domTechnique_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "technique_common" );
	_Meta->setStaticPointerAddress(&domRigid_body::domTechnique_common::_Meta);
	_Meta->registerConstructor(domRigid_body::domTechnique_common::create);

	// Add elements: dynamic, mass, mass_frame, inertia, instance_physics_material, physics_material, shape
    _Meta->appendElement(domRigid_body::domTechnique_common::domDynamic::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common,elemDynamic));
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common,elemMass),"mass"); 
    _Meta->appendElement(domRigid_body::domTechnique_common::domMass_frame::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common,elemMass_frame));
    _Meta->appendElement(domTargetableFloat3::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common,elemInertia),"inertia"); 
    _Meta->appendElement(domInstance_physics_material::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common,elemInstance_physics_material));
    _Meta->appendElement(domPhysics_material::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common,elemPhysics_material));
    _Meta->appendArrayElement(domRigid_body::domTechnique_common::domShape::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common,elemShape_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domRigid_body::domTechnique_common,_contents));

	
	
	_Meta->setElementSize(sizeof(domRigid_body::domTechnique_common));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domRigid_body::domTechnique_common::domDynamic::create(daeInt bytes)
{
	domRigid_body::domTechnique_common::domDynamicRef ref = new(bytes) domRigid_body::domTechnique_common::domDynamic;
	return ref;
}


daeMetaElement *
domRigid_body::domTechnique_common::domDynamic::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "dynamic" );
	_Meta->setStaticPointerAddress(&domRigid_body::domTechnique_common::domDynamic::_Meta);
	_Meta->registerConstructor(domRigid_body::domTechnique_common::domDynamic::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domRigid_body::domTechnique_common::domDynamic , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domRigid_body::domTechnique_common::domDynamic , attrSid ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domRigid_body::domTechnique_common::domDynamic));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domRigid_body::domTechnique_common::domMass_frame::create(daeInt bytes)
{
	domRigid_body::domTechnique_common::domMass_frameRef ref = new(bytes) domRigid_body::domTechnique_common::domMass_frame;
	return ref;
}


daeMetaElement *
domRigid_body::domTechnique_common::domMass_frame::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "mass_frame" );
	_Meta->setStaticPointerAddress(&domRigid_body::domTechnique_common::domMass_frame::_Meta);
	_Meta->registerConstructor(domRigid_body::domTechnique_common::domMass_frame::create);

	// Add elements: translate, rotate
    _Meta->appendArrayElement(domTranslate::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common::domMass_frame,elemTranslate_array));
    _Meta->appendArrayElement(domRotate::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common::domMass_frame,elemRotate_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domRigid_body::domTechnique_common::domMass_frame,_contents));

	
	
	_Meta->setElementSize(sizeof(domRigid_body::domTechnique_common::domMass_frame));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domRigid_body::domTechnique_common::domShape::create(daeInt bytes)
{
	domRigid_body::domTechnique_common::domShapeRef ref = new(bytes) domRigid_body::domTechnique_common::domShape;
	return ref;
}


daeMetaElement *
domRigid_body::domTechnique_common::domShape::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "shape" );
	_Meta->setStaticPointerAddress(&domRigid_body::domTechnique_common::domShape::_Meta);
	_Meta->registerConstructor(domRigid_body::domTechnique_common::domShape::create);

	// Add elements: hollow, mass, density, instance_physics_material, physics_material, instance_geometry, plane, box, sphere, cylinder, tapered_cylinder, capsule, tapered_capsule, translate, rotate, extra
    _Meta->appendElement(domRigid_body::domTechnique_common::domShape::domHollow::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common::domShape,elemHollow));
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common::domShape,elemMass),"mass"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common::domShape,elemDensity),"density"); 
    _Meta->appendElement(domInstance_physics_material::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common::domShape,elemInstance_physics_material));
    _Meta->appendElement(domPhysics_material::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common::domShape,elemPhysics_material));
    _Meta->appendElement(domInstance_geometry::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common::domShape,elemInstance_geometry));
    _Meta->appendElement(domPlane::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common::domShape,elemPlane));
    _Meta->appendElement(domBox::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common::domShape,elemBox));
    _Meta->appendElement(domSphere::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common::domShape,elemSphere));
    _Meta->appendElement(domCylinder::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common::domShape,elemCylinder));
    _Meta->appendElement(domTapered_cylinder::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common::domShape,elemTapered_cylinder));
    _Meta->appendElement(domCapsule::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common::domShape,elemCapsule));
    _Meta->appendElement(domTapered_capsule::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common::domShape,elemTapered_capsule));
    _Meta->appendArrayElement(domTranslate::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common::domShape,elemTranslate_array));
    _Meta->appendArrayElement(domRotate::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common::domShape,elemRotate_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domRigid_body::domTechnique_common::domShape,elemExtra_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domRigid_body::domTechnique_common::domShape,_contents));

	
	
	_Meta->setElementSize(sizeof(domRigid_body::domTechnique_common::domShape));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domRigid_body::domTechnique_common::domShape::domHollow::create(daeInt bytes)
{
	domRigid_body::domTechnique_common::domShape::domHollowRef ref = new(bytes) domRigid_body::domTechnique_common::domShape::domHollow;
	return ref;
}


daeMetaElement *
domRigid_body::domTechnique_common::domShape::domHollow::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "hollow" );
	_Meta->setStaticPointerAddress(&domRigid_body::domTechnique_common::domShape::domHollow::_Meta);
	_Meta->registerConstructor(domRigid_body::domTechnique_common::domShape::domHollow::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domRigid_body::domTechnique_common::domShape::domHollow , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domRigid_body::domTechnique_common::domShape::domHollow , attrSid ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domRigid_body::domTechnique_common::domShape::domHollow));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domRigid_body::_Meta = NULL;
daeMetaElement * domRigid_body::domTechnique_common::_Meta = NULL;
daeMetaElement * domRigid_body::domTechnique_common::domDynamic::_Meta = NULL;
daeMetaElement * domRigid_body::domTechnique_common::domMass_frame::_Meta = NULL;
daeMetaElement * domRigid_body::domTechnique_common::domShape::_Meta = NULL;
daeMetaElement * domRigid_body::domTechnique_common::domShape::domHollow::_Meta = NULL;


