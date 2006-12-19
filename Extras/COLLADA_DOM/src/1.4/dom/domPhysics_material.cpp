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
#include <dom/domPhysics_material.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domPhysics_material::create(daeInt bytes)
{
	domPhysics_materialRef ref = new(bytes) domPhysics_material;
	return ref;
}


daeMetaElement *
domPhysics_material::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "physics_material" );
	_Meta->registerClass(domPhysics_material::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domPhysics_material,elemAsset) );
	mea->setElementType( domAsset::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 1, 1 );
	mea->setName( "technique_common" );
	mea->setOffset( daeOffsetOf(domPhysics_material,elemTechnique_common) );
	mea->setElementType( domPhysics_material::domTechnique_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 2, 0, -1 );
	mea->setName( "technique" );
	mea->setOffset( daeOffsetOf(domPhysics_material,elemTechnique_array) );
	mea->setElementType( domTechnique::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domPhysics_material,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 3 );
	_Meta->setCMRoot( cm );	

	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domPhysics_material , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domPhysics_material , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domPhysics_material));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domPhysics_material::domTechnique_common::create(daeInt bytes)
{
	domPhysics_material::domTechnique_commonRef ref = new(bytes) domPhysics_material::domTechnique_common;
	return ref;
}


daeMetaElement *
domPhysics_material::domTechnique_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "technique_common" );
	_Meta->registerClass(domPhysics_material::domTechnique_common::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "dynamic_friction" );
	mea->setOffset( daeOffsetOf(domPhysics_material::domTechnique_common,elemDynamic_friction) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "restitution" );
	mea->setOffset( daeOffsetOf(domPhysics_material::domTechnique_common,elemRestitution) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "static_friction" );
	mea->setOffset( daeOffsetOf(domPhysics_material::domTechnique_common,elemStatic_friction) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 2 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domPhysics_material::domTechnique_common));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domPhysics_material::_Meta = NULL;
daeMetaElement * domPhysics_material::domTechnique_common::_Meta = NULL;


