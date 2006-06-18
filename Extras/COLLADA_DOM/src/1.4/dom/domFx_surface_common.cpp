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
#include <dom/domFx_surface_common.h>

daeElementRef
domFx_surface_common::create(daeInt bytes)
{
	domFx_surface_commonRef ref = new(bytes) domFx_surface_common;
	return ref;
}


daeMetaElement *
domFx_surface_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fx_surface_common" );
	_Meta->setStaticPointerAddress(&domFx_surface_common::_Meta);
	_Meta->registerConstructor(domFx_surface_common::create);

	// Add elements: init_from, format, size, viewport_ratio, mip_levels, mipmap_generate
    _Meta->appendArrayElement(domFx_surface_common::domInit_from::registerElement(),daeOffsetOf(domFx_surface_common,elemInit_from_array));
    _Meta->appendElement(domFx_surface_common::domFormat::registerElement(),daeOffsetOf(domFx_surface_common,elemFormat));
    _Meta->appendElement(domFx_surface_common::domSize::registerElement(),daeOffsetOf(domFx_surface_common,elemSize));
    _Meta->appendElement(domFx_surface_common::domViewport_ratio::registerElement(),daeOffsetOf(domFx_surface_common,elemViewport_ratio));
    _Meta->appendElement(domFx_surface_common::domMip_levels::registerElement(),daeOffsetOf(domFx_surface_common,elemMip_levels));
    _Meta->appendElement(domFx_surface_common::domMipmap_generate::registerElement(),daeOffsetOf(domFx_surface_common,elemMipmap_generate));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domFx_surface_common,_contents));


	//	Add attribute: type
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "type" );
		ma->setType( daeAtomicType::get("Fx_surface_type_enum"));
		ma->setOffset( daeOffsetOf( domFx_surface_common , attrType ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_common));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_common::domInit_from::create(daeInt bytes)
{
	domFx_surface_common::domInit_fromRef ref = new(bytes) domFx_surface_common::domInit_from;
	return ref;
}


daeMetaElement *
domFx_surface_common::domInit_from::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "init_from" );
	_Meta->setStaticPointerAddress(&domFx_surface_common::domInit_from::_Meta);
	_Meta->registerConstructor(domFx_surface_common::domInit_from::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsIDREFS"));
		ma->setOffset( daeOffsetOf( domFx_surface_common::domInit_from , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: mip
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "mip" );
		ma->setType( daeAtomicType::get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domFx_surface_common::domInit_from , attrMip ));
		ma->setContainer( _Meta );
		ma->setDefault( "0");
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: slice
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "slice" );
		ma->setType( daeAtomicType::get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domFx_surface_common::domInit_from , attrSlice ));
		ma->setContainer( _Meta );
		ma->setDefault( "0");
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: face
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "face" );
		ma->setType( daeAtomicType::get("Fx_surface_face_enum"));
		ma->setOffset( daeOffsetOf( domFx_surface_common::domInit_from , attrFace ));
		ma->setContainer( _Meta );
		ma->setDefault( "POSITIVE_X");
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_common::domInit_from));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_common::domFormat::create(daeInt bytes)
{
	domFx_surface_common::domFormatRef ref = new(bytes) domFx_surface_common::domFormat;
	return ref;
}


daeMetaElement *
domFx_surface_common::domFormat::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "format" );
	_Meta->setStaticPointerAddress(&domFx_surface_common::domFormat::_Meta);
	_Meta->registerConstructor(domFx_surface_common::domFormat::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsString"));
		ma->setOffset( daeOffsetOf( domFx_surface_common::domFormat , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_common::domFormat));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_common::domSize::create(daeInt bytes)
{
	domFx_surface_common::domSizeRef ref = new(bytes) domFx_surface_common::domSize;
	return ref;
}


daeMetaElement *
domFx_surface_common::domSize::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "size" );
	_Meta->setStaticPointerAddress(&domFx_surface_common::domSize::_Meta);
	_Meta->registerConstructor(domFx_surface_common::domSize::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Int3"));
		ma->setOffset( daeOffsetOf( domFx_surface_common::domSize , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_common::domSize));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_common::domViewport_ratio::create(daeInt bytes)
{
	domFx_surface_common::domViewport_ratioRef ref = new(bytes) domFx_surface_common::domViewport_ratio;
	return ref;
}


daeMetaElement *
domFx_surface_common::domViewport_ratio::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "viewport_ratio" );
	_Meta->setStaticPointerAddress(&domFx_surface_common::domViewport_ratio::_Meta);
	_Meta->registerConstructor(domFx_surface_common::domViewport_ratio::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float2"));
		ma->setOffset( daeOffsetOf( domFx_surface_common::domViewport_ratio , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_common::domViewport_ratio));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_common::domMip_levels::create(daeInt bytes)
{
	domFx_surface_common::domMip_levelsRef ref = new(bytes) domFx_surface_common::domMip_levels;
	return ref;
}


daeMetaElement *
domFx_surface_common::domMip_levels::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "mip_levels" );
	_Meta->setStaticPointerAddress(&domFx_surface_common::domMip_levels::_Meta);
	_Meta->registerConstructor(domFx_surface_common::domMip_levels::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domFx_surface_common::domMip_levels , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_common::domMip_levels));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_common::domMipmap_generate::create(daeInt bytes)
{
	domFx_surface_common::domMipmap_generateRef ref = new(bytes) domFx_surface_common::domMipmap_generate;
	return ref;
}


daeMetaElement *
domFx_surface_common::domMipmap_generate::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "mipmap_generate" );
	_Meta->setStaticPointerAddress(&domFx_surface_common::domMipmap_generate::_Meta);
	_Meta->registerConstructor(domFx_surface_common::domMipmap_generate::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domFx_surface_common::domMipmap_generate , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_common::domMipmap_generate));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domFx_surface_common::_Meta = NULL;
daeMetaElement * domFx_surface_common::domInit_from::_Meta = NULL;
daeMetaElement * domFx_surface_common::domFormat::_Meta = NULL;
daeMetaElement * domFx_surface_common::domSize::_Meta = NULL;
daeMetaElement * domFx_surface_common::domViewport_ratio::_Meta = NULL;
daeMetaElement * domFx_surface_common::domMip_levels::_Meta = NULL;
daeMetaElement * domFx_surface_common::domMipmap_generate::_Meta = NULL;


