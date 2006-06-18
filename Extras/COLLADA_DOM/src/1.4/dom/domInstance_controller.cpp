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
#include <dom/domInstance_controller.h>

daeElementRef
domInstance_controller::create(daeInt bytes)
{
	domInstance_controllerRef ref = new(bytes) domInstance_controller;
	ref->attrUrl.setContainer( (domInstance_controller*)ref );
	return ref;
}


daeMetaElement *
domInstance_controller::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "instance_controller" );
	_Meta->setStaticPointerAddress(&domInstance_controller::_Meta);
	_Meta->registerConstructor(domInstance_controller::create);

	// Add elements: skeleton, bind_material, extra
    _Meta->appendArrayElement(domInstance_controller::domSkeleton::registerElement(),daeOffsetOf(domInstance_controller,elemSkeleton_array));
    _Meta->appendElement(domBind_material::registerElement(),daeOffsetOf(domInstance_controller,elemBind_material));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domInstance_controller,elemExtra_array));

	//	Add attribute: url
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "url" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domInstance_controller , attrUrl ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domInstance_controller));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domInstance_controller::domSkeleton::create(daeInt bytes)
{
	domInstance_controller::domSkeletonRef ref = new(bytes) domInstance_controller::domSkeleton;
	ref->_value.setContainer( (domInstance_controller::domSkeleton*)ref );
	return ref;
}


daeMetaElement *
domInstance_controller::domSkeleton::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "skeleton" );
	_Meta->setStaticPointerAddress(&domInstance_controller::domSkeleton::_Meta);
	_Meta->registerConstructor(domInstance_controller::domSkeleton::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domInstance_controller::domSkeleton , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domInstance_controller::domSkeleton));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domInstance_controller::_Meta = NULL;
daeMetaElement * domInstance_controller::domSkeleton::_Meta = NULL;


