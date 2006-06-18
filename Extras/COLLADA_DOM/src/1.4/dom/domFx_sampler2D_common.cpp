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
#include <dom/domFx_sampler2D_common.h>

daeElementRef
domFx_sampler2D_common::create(daeInt bytes)
{
	domFx_sampler2D_commonRef ref = new(bytes) domFx_sampler2D_common;
	return ref;
}


daeMetaElement *
domFx_sampler2D_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fx_sampler2D_common" );
	_Meta->setStaticPointerAddress(&domFx_sampler2D_common::_Meta);
	_Meta->registerConstructor(domFx_sampler2D_common::create);

	// Add elements: source, wrap_s, wrap_t, minfilter, magfilter, mipfilter, border_color, mipmap_maxlevel, mipmap_bias
    _Meta->appendElement(domFx_sampler2D_common::domSource::registerElement(),daeOffsetOf(domFx_sampler2D_common,elemSource));
    _Meta->appendElement(domFx_sampler2D_common::domWrap_s::registerElement(),daeOffsetOf(domFx_sampler2D_common,elemWrap_s));
    _Meta->appendElement(domFx_sampler2D_common::domWrap_t::registerElement(),daeOffsetOf(domFx_sampler2D_common,elemWrap_t));
    _Meta->appendElement(domFx_sampler2D_common::domMinfilter::registerElement(),daeOffsetOf(domFx_sampler2D_common,elemMinfilter));
    _Meta->appendElement(domFx_sampler2D_common::domMagfilter::registerElement(),daeOffsetOf(domFx_sampler2D_common,elemMagfilter));
    _Meta->appendElement(domFx_sampler2D_common::domMipfilter::registerElement(),daeOffsetOf(domFx_sampler2D_common,elemMipfilter));
    _Meta->appendElement(domFx_sampler2D_common::domBorder_color::registerElement(),daeOffsetOf(domFx_sampler2D_common,elemBorder_color));
    _Meta->appendElement(domFx_sampler2D_common::domMipmap_maxlevel::registerElement(),daeOffsetOf(domFx_sampler2D_common,elemMipmap_maxlevel));
    _Meta->appendElement(domFx_sampler2D_common::domMipmap_bias::registerElement(),daeOffsetOf(domFx_sampler2D_common,elemMipmap_bias));
	
	
	_Meta->setElementSize(sizeof(domFx_sampler2D_common));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_sampler2D_common::domSource::create(daeInt bytes)
{
	domFx_sampler2D_common::domSourceRef ref = new(bytes) domFx_sampler2D_common::domSource;
	return ref;
}


daeMetaElement *
domFx_sampler2D_common::domSource::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "source" );
	_Meta->setStaticPointerAddress(&domFx_sampler2D_common::domSource::_Meta);
	_Meta->registerConstructor(domFx_sampler2D_common::domSource::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domFx_sampler2D_common::domSource , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_sampler2D_common::domSource));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_sampler2D_common::domWrap_s::create(daeInt bytes)
{
	domFx_sampler2D_common::domWrap_sRef ref = new(bytes) domFx_sampler2D_common::domWrap_s;
	return ref;
}


daeMetaElement *
domFx_sampler2D_common::domWrap_s::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "wrap_s" );
	_Meta->setStaticPointerAddress(&domFx_sampler2D_common::domWrap_s::_Meta);
	_Meta->registerConstructor(domFx_sampler2D_common::domWrap_s::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_sampler_wrap_common"));
		ma->setOffset( daeOffsetOf( domFx_sampler2D_common::domWrap_s , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_sampler2D_common::domWrap_s));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_sampler2D_common::domWrap_t::create(daeInt bytes)
{
	domFx_sampler2D_common::domWrap_tRef ref = new(bytes) domFx_sampler2D_common::domWrap_t;
	return ref;
}


daeMetaElement *
domFx_sampler2D_common::domWrap_t::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "wrap_t" );
	_Meta->setStaticPointerAddress(&domFx_sampler2D_common::domWrap_t::_Meta);
	_Meta->registerConstructor(domFx_sampler2D_common::domWrap_t::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_sampler_wrap_common"));
		ma->setOffset( daeOffsetOf( domFx_sampler2D_common::domWrap_t , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_sampler2D_common::domWrap_t));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_sampler2D_common::domMinfilter::create(daeInt bytes)
{
	domFx_sampler2D_common::domMinfilterRef ref = new(bytes) domFx_sampler2D_common::domMinfilter;
	return ref;
}


daeMetaElement *
domFx_sampler2D_common::domMinfilter::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "minfilter" );
	_Meta->setStaticPointerAddress(&domFx_sampler2D_common::domMinfilter::_Meta);
	_Meta->registerConstructor(domFx_sampler2D_common::domMinfilter::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_sampler_filter_common"));
		ma->setOffset( daeOffsetOf( domFx_sampler2D_common::domMinfilter , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_sampler2D_common::domMinfilter));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_sampler2D_common::domMagfilter::create(daeInt bytes)
{
	domFx_sampler2D_common::domMagfilterRef ref = new(bytes) domFx_sampler2D_common::domMagfilter;
	return ref;
}


daeMetaElement *
domFx_sampler2D_common::domMagfilter::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "magfilter" );
	_Meta->setStaticPointerAddress(&domFx_sampler2D_common::domMagfilter::_Meta);
	_Meta->registerConstructor(domFx_sampler2D_common::domMagfilter::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_sampler_filter_common"));
		ma->setOffset( daeOffsetOf( domFx_sampler2D_common::domMagfilter , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_sampler2D_common::domMagfilter));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_sampler2D_common::domMipfilter::create(daeInt bytes)
{
	domFx_sampler2D_common::domMipfilterRef ref = new(bytes) domFx_sampler2D_common::domMipfilter;
	return ref;
}


daeMetaElement *
domFx_sampler2D_common::domMipfilter::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "mipfilter" );
	_Meta->setStaticPointerAddress(&domFx_sampler2D_common::domMipfilter::_Meta);
	_Meta->registerConstructor(domFx_sampler2D_common::domMipfilter::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_sampler_filter_common"));
		ma->setOffset( daeOffsetOf( domFx_sampler2D_common::domMipfilter , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_sampler2D_common::domMipfilter));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_sampler2D_common::domBorder_color::create(daeInt bytes)
{
	domFx_sampler2D_common::domBorder_colorRef ref = new(bytes) domFx_sampler2D_common::domBorder_color;
	return ref;
}


daeMetaElement *
domFx_sampler2D_common::domBorder_color::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "border_color" );
	_Meta->setStaticPointerAddress(&domFx_sampler2D_common::domBorder_color::_Meta);
	_Meta->registerConstructor(domFx_sampler2D_common::domBorder_color::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_color_common"));
		ma->setOffset( daeOffsetOf( domFx_sampler2D_common::domBorder_color , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_sampler2D_common::domBorder_color));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_sampler2D_common::domMipmap_maxlevel::create(daeInt bytes)
{
	domFx_sampler2D_common::domMipmap_maxlevelRef ref = new(bytes) domFx_sampler2D_common::domMipmap_maxlevel;
	return ref;
}


daeMetaElement *
domFx_sampler2D_common::domMipmap_maxlevel::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "mipmap_maxlevel" );
	_Meta->setStaticPointerAddress(&domFx_sampler2D_common::domMipmap_maxlevel::_Meta);
	_Meta->registerConstructor(domFx_sampler2D_common::domMipmap_maxlevel::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsUnsignedByte"));
		ma->setOffset( daeOffsetOf( domFx_sampler2D_common::domMipmap_maxlevel , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_sampler2D_common::domMipmap_maxlevel));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_sampler2D_common::domMipmap_bias::create(daeInt bytes)
{
	domFx_sampler2D_common::domMipmap_biasRef ref = new(bytes) domFx_sampler2D_common::domMipmap_bias;
	return ref;
}


daeMetaElement *
domFx_sampler2D_common::domMipmap_bias::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "mipmap_bias" );
	_Meta->setStaticPointerAddress(&domFx_sampler2D_common::domMipmap_bias::_Meta);
	_Meta->registerConstructor(domFx_sampler2D_common::domMipmap_bias::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsFloat"));
		ma->setOffset( daeOffsetOf( domFx_sampler2D_common::domMipmap_bias , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_sampler2D_common::domMipmap_bias));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domFx_sampler2D_common::_Meta = NULL;
daeMetaElement * domFx_sampler2D_common::domSource::_Meta = NULL;
daeMetaElement * domFx_sampler2D_common::domWrap_s::_Meta = NULL;
daeMetaElement * domFx_sampler2D_common::domWrap_t::_Meta = NULL;
daeMetaElement * domFx_sampler2D_common::domMinfilter::_Meta = NULL;
daeMetaElement * domFx_sampler2D_common::domMagfilter::_Meta = NULL;
daeMetaElement * domFx_sampler2D_common::domMipfilter::_Meta = NULL;
daeMetaElement * domFx_sampler2D_common::domBorder_color::_Meta = NULL;
daeMetaElement * domFx_sampler2D_common::domMipmap_maxlevel::_Meta = NULL;
daeMetaElement * domFx_sampler2D_common::domMipmap_bias::_Meta = NULL;


