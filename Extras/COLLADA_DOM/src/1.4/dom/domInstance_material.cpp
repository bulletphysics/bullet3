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
#include <dom/domInstance_material.h>

daeElementRef
domInstance_material::create(daeInt bytes)
{
	domInstance_materialRef ref = new(bytes) domInstance_material;
	ref->attrTarget.setContainer( (domInstance_material*)ref );
	return ref;
}


daeMetaElement *
domInstance_material::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "instance_material" );
	_Meta->setStaticPointerAddress(&domInstance_material::_Meta);
	_Meta->registerConstructor(domInstance_material::create);

	// Add elements: bind, extra
    _Meta->appendArrayElement(domInstance_material::domBind::registerElement(),daeOffsetOf(domInstance_material,elemBind_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domInstance_material,elemExtra_array));

	//	Add attribute: symbol
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "symbol" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_material , attrSymbol ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: target
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "target" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domInstance_material , attrTarget ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domInstance_material));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domInstance_material::domBind::create(daeInt bytes)
{
	domInstance_material::domBindRef ref = new(bytes) domInstance_material::domBind;
	return ref;
}


daeMetaElement *
domInstance_material::domBind::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bind" );
	_Meta->setStaticPointerAddress(&domInstance_material::domBind::_Meta);
	_Meta->registerConstructor(domInstance_material::domBind::create);


	//	Add attribute: semantic
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "semantic" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_material::domBind , attrSemantic ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: target
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "target" );
		ma->setType( daeAtomicType::get("xsToken"));
		ma->setOffset( daeOffsetOf( domInstance_material::domBind , attrTarget ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domInstance_material::domBind));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domInstance_material::_Meta = NULL;
daeMetaElement * domInstance_material::domBind::_Meta = NULL;


