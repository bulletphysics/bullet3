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
#include <dom/domFx_samplerCUBE_common.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFx_samplerCUBE_common::create(daeInt bytes)
{
	domFx_samplerCUBE_commonRef ref = new(bytes) domFx_samplerCUBE_common;
	return ref;
}


daeMetaElement *
domFx_samplerCUBE_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fx_samplerCUBE_common" );
	_Meta->registerClass(domFx_samplerCUBE_common::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "source" );
	mea->setOffset( daeOffsetOf(domFx_samplerCUBE_common,elemSource) );
	mea->setElementType( domFx_samplerCUBE_common::domSource::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "wrap_s" );
	mea->setOffset( daeOffsetOf(domFx_samplerCUBE_common,elemWrap_s) );
	mea->setElementType( domFx_samplerCUBE_common::domWrap_s::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "wrap_t" );
	mea->setOffset( daeOffsetOf(domFx_samplerCUBE_common,elemWrap_t) );
	mea->setElementType( domFx_samplerCUBE_common::domWrap_t::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 3, 0, 1 );
	mea->setName( "wrap_p" );
	mea->setOffset( daeOffsetOf(domFx_samplerCUBE_common,elemWrap_p) );
	mea->setElementType( domFx_samplerCUBE_common::domWrap_p::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 4, 0, 1 );
	mea->setName( "minfilter" );
	mea->setOffset( daeOffsetOf(domFx_samplerCUBE_common,elemMinfilter) );
	mea->setElementType( domFx_samplerCUBE_common::domMinfilter::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 5, 0, 1 );
	mea->setName( "magfilter" );
	mea->setOffset( daeOffsetOf(domFx_samplerCUBE_common,elemMagfilter) );
	mea->setElementType( domFx_samplerCUBE_common::domMagfilter::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 6, 0, 1 );
	mea->setName( "mipfilter" );
	mea->setOffset( daeOffsetOf(domFx_samplerCUBE_common,elemMipfilter) );
	mea->setElementType( domFx_samplerCUBE_common::domMipfilter::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 7, 0, 1 );
	mea->setName( "border_color" );
	mea->setOffset( daeOffsetOf(domFx_samplerCUBE_common,elemBorder_color) );
	mea->setElementType( domFx_samplerCUBE_common::domBorder_color::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 8, 0, 1 );
	mea->setName( "mipmap_maxlevel" );
	mea->setOffset( daeOffsetOf(domFx_samplerCUBE_common,elemMipmap_maxlevel) );
	mea->setElementType( domFx_samplerCUBE_common::domMipmap_maxlevel::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 9, 0, 1 );
	mea->setName( "mipmap_bias" );
	mea->setOffset( daeOffsetOf(domFx_samplerCUBE_common,elemMipmap_bias) );
	mea->setElementType( domFx_samplerCUBE_common::domMipmap_bias::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 10, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domFx_samplerCUBE_common,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 10 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domFx_samplerCUBE_common));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_samplerCUBE_common::domSource::create(daeInt bytes)
{
	domFx_samplerCUBE_common::domSourceRef ref = new(bytes) domFx_samplerCUBE_common::domSource;
	return ref;
}


daeMetaElement *
domFx_samplerCUBE_common::domSource::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "source" );
	_Meta->registerClass(domFx_samplerCUBE_common::domSource::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domFx_samplerCUBE_common::domSource , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_samplerCUBE_common::domSource));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_samplerCUBE_common::domWrap_s::create(daeInt bytes)
{
	domFx_samplerCUBE_common::domWrap_sRef ref = new(bytes) domFx_samplerCUBE_common::domWrap_s;
	return ref;
}


daeMetaElement *
domFx_samplerCUBE_common::domWrap_s::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "wrap_s" );
	_Meta->registerClass(domFx_samplerCUBE_common::domWrap_s::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_sampler_wrap_common"));
		ma->setOffset( daeOffsetOf( domFx_samplerCUBE_common::domWrap_s , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_samplerCUBE_common::domWrap_s));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_samplerCUBE_common::domWrap_t::create(daeInt bytes)
{
	domFx_samplerCUBE_common::domWrap_tRef ref = new(bytes) domFx_samplerCUBE_common::domWrap_t;
	return ref;
}


daeMetaElement *
domFx_samplerCUBE_common::domWrap_t::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "wrap_t" );
	_Meta->registerClass(domFx_samplerCUBE_common::domWrap_t::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_sampler_wrap_common"));
		ma->setOffset( daeOffsetOf( domFx_samplerCUBE_common::domWrap_t , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_samplerCUBE_common::domWrap_t));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_samplerCUBE_common::domWrap_p::create(daeInt bytes)
{
	domFx_samplerCUBE_common::domWrap_pRef ref = new(bytes) domFx_samplerCUBE_common::domWrap_p;
	return ref;
}


daeMetaElement *
domFx_samplerCUBE_common::domWrap_p::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "wrap_p" );
	_Meta->registerClass(domFx_samplerCUBE_common::domWrap_p::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_sampler_wrap_common"));
		ma->setOffset( daeOffsetOf( domFx_samplerCUBE_common::domWrap_p , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_samplerCUBE_common::domWrap_p));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_samplerCUBE_common::domMinfilter::create(daeInt bytes)
{
	domFx_samplerCUBE_common::domMinfilterRef ref = new(bytes) domFx_samplerCUBE_common::domMinfilter;
	return ref;
}


daeMetaElement *
domFx_samplerCUBE_common::domMinfilter::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "minfilter" );
	_Meta->registerClass(domFx_samplerCUBE_common::domMinfilter::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_sampler_filter_common"));
		ma->setOffset( daeOffsetOf( domFx_samplerCUBE_common::domMinfilter , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_samplerCUBE_common::domMinfilter));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_samplerCUBE_common::domMagfilter::create(daeInt bytes)
{
	domFx_samplerCUBE_common::domMagfilterRef ref = new(bytes) domFx_samplerCUBE_common::domMagfilter;
	return ref;
}


daeMetaElement *
domFx_samplerCUBE_common::domMagfilter::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "magfilter" );
	_Meta->registerClass(domFx_samplerCUBE_common::domMagfilter::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_sampler_filter_common"));
		ma->setOffset( daeOffsetOf( domFx_samplerCUBE_common::domMagfilter , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_samplerCUBE_common::domMagfilter));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_samplerCUBE_common::domMipfilter::create(daeInt bytes)
{
	domFx_samplerCUBE_common::domMipfilterRef ref = new(bytes) domFx_samplerCUBE_common::domMipfilter;
	return ref;
}


daeMetaElement *
domFx_samplerCUBE_common::domMipfilter::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "mipfilter" );
	_Meta->registerClass(domFx_samplerCUBE_common::domMipfilter::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_sampler_filter_common"));
		ma->setOffset( daeOffsetOf( domFx_samplerCUBE_common::domMipfilter , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_samplerCUBE_common::domMipfilter));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_samplerCUBE_common::domBorder_color::create(daeInt bytes)
{
	domFx_samplerCUBE_common::domBorder_colorRef ref = new(bytes) domFx_samplerCUBE_common::domBorder_color;
	return ref;
}


daeMetaElement *
domFx_samplerCUBE_common::domBorder_color::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "border_color" );
	_Meta->registerClass(domFx_samplerCUBE_common::domBorder_color::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_color_common"));
		ma->setOffset( daeOffsetOf( domFx_samplerCUBE_common::domBorder_color , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_samplerCUBE_common::domBorder_color));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_samplerCUBE_common::domMipmap_maxlevel::create(daeInt bytes)
{
	domFx_samplerCUBE_common::domMipmap_maxlevelRef ref = new(bytes) domFx_samplerCUBE_common::domMipmap_maxlevel;
	return ref;
}


daeMetaElement *
domFx_samplerCUBE_common::domMipmap_maxlevel::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "mipmap_maxlevel" );
	_Meta->registerClass(domFx_samplerCUBE_common::domMipmap_maxlevel::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsUnsignedByte"));
		ma->setOffset( daeOffsetOf( domFx_samplerCUBE_common::domMipmap_maxlevel , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_samplerCUBE_common::domMipmap_maxlevel));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_samplerCUBE_common::domMipmap_bias::create(daeInt bytes)
{
	domFx_samplerCUBE_common::domMipmap_biasRef ref = new(bytes) domFx_samplerCUBE_common::domMipmap_bias;
	return ref;
}


daeMetaElement *
domFx_samplerCUBE_common::domMipmap_bias::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "mipmap_bias" );
	_Meta->registerClass(domFx_samplerCUBE_common::domMipmap_bias::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsFloat"));
		ma->setOffset( daeOffsetOf( domFx_samplerCUBE_common::domMipmap_bias , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_samplerCUBE_common::domMipmap_bias));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domFx_samplerCUBE_common::_Meta = NULL;
daeMetaElement * domFx_samplerCUBE_common::domSource::_Meta = NULL;
daeMetaElement * domFx_samplerCUBE_common::domWrap_s::_Meta = NULL;
daeMetaElement * domFx_samplerCUBE_common::domWrap_t::_Meta = NULL;
daeMetaElement * domFx_samplerCUBE_common::domWrap_p::_Meta = NULL;
daeMetaElement * domFx_samplerCUBE_common::domMinfilter::_Meta = NULL;
daeMetaElement * domFx_samplerCUBE_common::domMagfilter::_Meta = NULL;
daeMetaElement * domFx_samplerCUBE_common::domMipfilter::_Meta = NULL;
daeMetaElement * domFx_samplerCUBE_common::domBorder_color::_Meta = NULL;
daeMetaElement * domFx_samplerCUBE_common::domMipmap_maxlevel::_Meta = NULL;
daeMetaElement * domFx_samplerCUBE_common::domMipmap_bias::_Meta = NULL;


