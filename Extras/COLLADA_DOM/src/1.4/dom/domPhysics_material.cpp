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
	_Meta->setStaticPointerAddress(&domPhysics_material::_Meta);
	_Meta->registerConstructor(domPhysics_material::create);

	// Add elements: asset, technique_common, technique, extra
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domPhysics_material,elemAsset));
    _Meta->appendElement(domPhysics_material::domTechnique_common::registerElement(),daeOffsetOf(domPhysics_material,elemTechnique_common));
    _Meta->appendArrayElement(domTechnique::registerElement(),daeOffsetOf(domPhysics_material,elemTechnique_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domPhysics_material,elemExtra_array));

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
	_Meta->setStaticPointerAddress(&domPhysics_material::domTechnique_common::_Meta);
	_Meta->registerConstructor(domPhysics_material::domTechnique_common::create);

	// Add elements: dynamic_friction, restitution, static_friction
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domPhysics_material::domTechnique_common,elemDynamic_friction),"dynamic_friction"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domPhysics_material::domTechnique_common,elemRestitution),"restitution"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domPhysics_material::domTechnique_common,elemStatic_friction),"static_friction"); 
	
	
	_Meta->setElementSize(sizeof(domPhysics_material::domTechnique_common));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domPhysics_material::_Meta = NULL;
daeMetaElement * domPhysics_material::domTechnique_common::_Meta = NULL;


