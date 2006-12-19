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
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

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
	_Meta->registerClass(domCommon_color_or_texture_type::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "color" );
	mea->setOffset( daeOffsetOf(domCommon_color_or_texture_type,elemColor) );
	mea->setElementType( domCommon_color_or_texture_type::domColor::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "param" );
	mea->setOffset( daeOffsetOf(domCommon_color_or_texture_type,elemParam) );
	mea->setElementType( domCommon_color_or_texture_type::domParam::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "texture" );
	mea->setOffset( daeOffsetOf(domCommon_color_or_texture_type,elemTexture) );
	mea->setElementType( domCommon_color_or_texture_type::domTexture::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domCommon_color_or_texture_type,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domCommon_color_or_texture_type,_contentsOrder));

	
	
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
	_Meta->registerClass(domCommon_color_or_texture_type::domColor::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domCommon_color_or_texture_type::domParam::create, &_Meta);

	_Meta->setIsInnerClass( true );

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
	_Meta->registerClass(domCommon_color_or_texture_type::domTexture::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domCommon_color_or_texture_type::domTexture,elemExtra) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	

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


