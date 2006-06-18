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
#include <dom/domCommon_color_or_texture_type.h>

daeElementRef
domCommon_color_or_texture_type::create(daeInt bytes)
{
	domCommon_color_or_texture_typeRef ref = new(bytes) domCommon_color_or_texture_type;
	return ref;
}


daeMetaElement *
domCommon_color_or_texture_type::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "common_color_or_texture_type" );
	_Meta->setStaticPointerAddress(&domCommon_color_or_texture_type::_Meta);
	_Meta->registerConstructor(domCommon_color_or_texture_type::create);

	// Add elements: color, param, texture
    _Meta->appendElement(domCommon_color_or_texture_type::domColor::registerElement(),daeOffsetOf(domCommon_color_or_texture_type,elemColor));
    _Meta->appendElement(domCommon_color_or_texture_type::domParam::registerElement(),daeOffsetOf(domCommon_color_or_texture_type,elemParam));
    _Meta->appendElement(domCommon_color_or_texture_type::domTexture::registerElement(),daeOffsetOf(domCommon_color_or_texture_type,elemTexture));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domCommon_color_or_texture_type,_contents));

	
	
	_Meta->setElementSize(sizeof(domCommon_color_or_texture_type));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCommon_color_or_texture_type::domColor::create(daeInt bytes)
{
	domCommon_color_or_texture_type::domColorRef ref = new(bytes) domCommon_color_or_texture_type::domColor;
	return ref;
}


daeMetaElement *
domCommon_color_or_texture_type::domColor::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "color" );
	_Meta->setStaticPointerAddress(&domCommon_color_or_texture_type::domColor::_Meta);
	_Meta->registerConstructor(domCommon_color_or_texture_type::domColor::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_color_common"));
		ma->setOffset( daeOffsetOf( domCommon_color_or_texture_type::domColor , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domCommon_color_or_texture_type::domColor , attrSid ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCommon_color_or_texture_type::domColor));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCommon_color_or_texture_type::domParam::create(daeInt bytes)
{
	domCommon_color_or_texture_type::domParamRef ref = new(bytes) domCommon_color_or_texture_type::domParam;
	return ref;
}


daeMetaElement *
domCommon_color_or_texture_type::domParam::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "param" );
	_Meta->setStaticPointerAddress(&domCommon_color_or_texture_type::domParam::_Meta);
	_Meta->registerConstructor(domCommon_color_or_texture_type::domParam::create);


	//	Add attribute: ref
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domCommon_color_or_texture_type::domParam , attrRef ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCommon_color_or_texture_type::domParam));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCommon_color_or_texture_type::domTexture::create(daeInt bytes)
{
	domCommon_color_or_texture_type::domTextureRef ref = new(bytes) domCommon_color_or_texture_type::domTexture;
	return ref;
}


daeMetaElement *
domCommon_color_or_texture_type::domTexture::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "texture" );
	_Meta->setStaticPointerAddress(&domCommon_color_or_texture_type::domTexture::_Meta);
	_Meta->registerConstructor(domCommon_color_or_texture_type::domTexture::create);

	// Add elements: extra
    _Meta->appendElement(domExtra::registerElement(),daeOffsetOf(domCommon_color_or_texture_type::domTexture,elemExtra));

	//	Add attribute: texture
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "texture" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domCommon_color_or_texture_type::domTexture , attrTexture ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: texcoord
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "texcoord" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domCommon_color_or_texture_type::domTexture , attrTexcoord ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCommon_color_or_texture_type::domTexture));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domCommon_color_or_texture_type::_Meta = NULL;
daeMetaElement * domCommon_color_or_texture_type::domColor::_Meta = NULL;
daeMetaElement * domCommon_color_or_texture_type::domParam::_Meta = NULL;
daeMetaElement * domCommon_color_or_texture_type::domTexture::_Meta = NULL;


