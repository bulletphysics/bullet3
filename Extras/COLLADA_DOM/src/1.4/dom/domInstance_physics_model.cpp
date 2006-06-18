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
#include <dom/domInstance_physics_model.h>

daeElementRef
domInstance_physics_model::create(daeInt bytes)
{
	domInstance_physics_modelRef ref = new(bytes) domInstance_physics_model;
	ref->attrUrl.setContainer( (domInstance_physics_model*)ref );
	ref->attrParent.setContainer( (domInstance_physics_model*)ref );
	return ref;
}


daeMetaElement *
domInstance_physics_model::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "instance_physics_model" );
	_Meta->setStaticPointerAddress(&domInstance_physics_model::_Meta);
	_Meta->registerConstructor(domInstance_physics_model::create);

	// Add elements: instance_force_field, instance_rigid_body, instance_rigid_constraint, extra
    _Meta->appendArrayElement(domInstance_force_field::registerElement(),daeOffsetOf(domInstance_physics_model,elemInstance_force_field_array));
    _Meta->appendArrayElement(domInstance_rigid_body::registerElement(),daeOffsetOf(domInstance_physics_model,elemInstance_rigid_body_array));
    _Meta->appendArrayElement(domInstance_rigid_constraint::registerElement(),daeOffsetOf(domInstance_physics_model,elemInstance_rigid_constraint_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domInstance_physics_model,elemExtra_array));

	//	Add attribute: url
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "url" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domInstance_physics_model , attrUrl ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_physics_model , attrSid ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: parent
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "parent" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domInstance_physics_model , attrParent ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domInstance_physics_model));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domInstance_physics_model::_Meta = NULL;


