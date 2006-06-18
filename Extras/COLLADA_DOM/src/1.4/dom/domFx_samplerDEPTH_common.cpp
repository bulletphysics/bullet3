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
#include <dom/domFx_samplerDEPTH_common.h>

daeElementRef
domFx_samplerDEPTH_common::create(daeInt bytes)
{
	domFx_samplerDEPTH_commonRef ref = new(bytes) domFx_samplerDEPTH_common;
	return ref;
}


daeMetaElement *
domFx_samplerDEPTH_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fx_samplerDEPTH_common" );
	_Meta->setStaticPointerAddress(&domFx_samplerDEPTH_common::_Meta);
	_Meta->registerConstructor(domFx_samplerDEPTH_common::create);

	// Add elements: source, wrap_s, wrap_t, minfilter, magfilter
    _Meta->appendElement(domFx_samplerDEPTH_common::domSource::registerElement(),daeOffsetOf(domFx_samplerDEPTH_common,elemSource));
    _Meta->appendElement(domFx_samplerDEPTH_common::domWrap_s::registerElement(),daeOffsetOf(domFx_samplerDEPTH_common,elemWrap_s));
    _Meta->appendElement(domFx_samplerDEPTH_common::domWrap_t::registerElement(),daeOffsetOf(domFx_samplerDEPTH_common,elemWrap_t));
    _Meta->appendElement(domFx_samplerDEPTH_common::domMinfilter::registerElement(),daeOffsetOf(domFx_samplerDEPTH_common,elemMinfilter));
    _Meta->appendElement(domFx_samplerDEPTH_common::domMagfilter::registerElement(),daeOffsetOf(domFx_samplerDEPTH_common,elemMagfilter));
	
	
	_Meta->setElementSize(sizeof(domFx_samplerDEPTH_common));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_samplerDEPTH_common::domSource::create(daeInt bytes)
{
	domFx_samplerDEPTH_common::domSourceRef ref = new(bytes) domFx_samplerDEPTH_common::domSource;
	return ref;
}


daeMetaElement *
domFx_samplerDEPTH_common::domSource::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "source" );
	_Meta->setStaticPointerAddress(&domFx_samplerDEPTH_common::domSource::_Meta);
	_Meta->registerConstructor(domFx_samplerDEPTH_common::domSource::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domFx_samplerDEPTH_common::domSource , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_samplerDEPTH_common::domSource));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_samplerDEPTH_common::domWrap_s::create(daeInt bytes)
{
	domFx_samplerDEPTH_common::domWrap_sRef ref = new(bytes) domFx_samplerDEPTH_common::domWrap_s;
	return ref;
}


daeMetaElement *
domFx_samplerDEPTH_common::domWrap_s::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "wrap_s" );
	_Meta->setStaticPointerAddress(&domFx_samplerDEPTH_common::domWrap_s::_Meta);
	_Meta->registerConstructor(domFx_samplerDEPTH_common::domWrap_s::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_sampler_wrap_common"));
		ma->setOffset( daeOffsetOf( domFx_samplerDEPTH_common::domWrap_s , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_samplerDEPTH_common::domWrap_s));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_samplerDEPTH_common::domWrap_t::create(daeInt bytes)
{
	domFx_samplerDEPTH_common::domWrap_tRef ref = new(bytes) domFx_samplerDEPTH_common::domWrap_t;
	return ref;
}


daeMetaElement *
domFx_samplerDEPTH_common::domWrap_t::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "wrap_t" );
	_Meta->setStaticPointerAddress(&domFx_samplerDEPTH_common::domWrap_t::_Meta);
	_Meta->registerConstructor(domFx_samplerDEPTH_common::domWrap_t::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_sampler_wrap_common"));
		ma->setOffset( daeOffsetOf( domFx_samplerDEPTH_common::domWrap_t , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_samplerDEPTH_common::domWrap_t));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_samplerDEPTH_common::domMinfilter::create(daeInt bytes)
{
	domFx_samplerDEPTH_common::domMinfilterRef ref = new(bytes) domFx_samplerDEPTH_common::domMinfilter;
	return ref;
}


daeMetaElement *
domFx_samplerDEPTH_common::domMinfilter::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "minfilter" );
	_Meta->setStaticPointerAddress(&domFx_samplerDEPTH_common::domMinfilter::_Meta);
	_Meta->registerConstructor(domFx_samplerDEPTH_common::domMinfilter::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_sampler_filter_common"));
		ma->setOffset( daeOffsetOf( domFx_samplerDEPTH_common::domMinfilter , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_samplerDEPTH_common::domMinfilter));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_samplerDEPTH_common::domMagfilter::create(daeInt bytes)
{
	domFx_samplerDEPTH_common::domMagfilterRef ref = new(bytes) domFx_samplerDEPTH_common::domMagfilter;
	return ref;
}


daeMetaElement *
domFx_samplerDEPTH_common::domMagfilter::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "magfilter" );
	_Meta->setStaticPointerAddress(&domFx_samplerDEPTH_common::domMagfilter::_Meta);
	_Meta->registerConstructor(domFx_samplerDEPTH_common::domMagfilter::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_sampler_filter_common"));
		ma->setOffset( daeOffsetOf( domFx_samplerDEPTH_common::domMagfilter , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_samplerDEPTH_common::domMagfilter));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domFx_samplerDEPTH_common::_Meta = NULL;
daeMetaElement * domFx_samplerDEPTH_common::domSource::_Meta = NULL;
daeMetaElement * domFx_samplerDEPTH_common::domWrap_s::_Meta = NULL;
daeMetaElement * domFx_samplerDEPTH_common::domWrap_t::_Meta = NULL;
daeMetaElement * domFx_samplerDEPTH_common::domMinfilter::_Meta = NULL;
daeMetaElement * domFx_samplerDEPTH_common::domMagfilter::_Meta = NULL;


