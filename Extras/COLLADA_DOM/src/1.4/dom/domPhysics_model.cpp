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
#include <dom/domPhysics_model.h>

daeElementRef
domPhysics_model::create(daeInt bytes)
{
	domPhysics_modelRef ref = new(bytes) domPhysics_model;
	return ref;
}


daeMetaElement *
domPhysics_model::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "physics_model" );
	_Meta->setStaticPointerAddress(&domPhysics_model::_Meta);
	_Meta->registerConstructor(domPhysics_model::create);

	// Add elements: asset, rigid_body, rigid_constraint, instance_physics_model, extra
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domPhysics_model,elemAsset));
    _Meta->appendArrayElement(domRigid_body::registerElement(),daeOffsetOf(domPhysics_model,elemRigid_body_array));
    _Meta->appendArrayElement(domRigid_constraint::registerElement(),daeOffsetOf(domPhysics_model,elemRigid_constraint_array));
    _Meta->appendArrayElement(domInstance_physics_model::registerElement(),daeOffsetOf(domPhysics_model,elemInstance_physics_model_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domPhysics_model,elemExtra_array));

	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domPhysics_model , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domPhysics_model , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domPhysics_model));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domPhysics_model::_Meta = NULL;


