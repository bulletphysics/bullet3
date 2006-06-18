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
#include <dom/domInstance_geometry.h>

daeElementRef
domInstance_geometry::create(daeInt bytes)
{
	domInstance_geometryRef ref = new(bytes) domInstance_geometry;
	ref->attrUrl.setContainer( (domInstance_geometry*)ref );
	return ref;
}


daeMetaElement *
domInstance_geometry::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "instance_geometry" );
	_Meta->setStaticPointerAddress(&domInstance_geometry::_Meta);
	_Meta->registerConstructor(domInstance_geometry::create);

	// Add elements: bind_material, extra
    _Meta->appendElement(domBind_material::registerElement(),daeOffsetOf(domInstance_geometry,elemBind_material));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domInstance_geometry,elemExtra_array));

	//	Add attribute: url
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "url" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domInstance_geometry , attrUrl ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domInstance_geometry));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domInstance_geometry::_Meta = NULL;


