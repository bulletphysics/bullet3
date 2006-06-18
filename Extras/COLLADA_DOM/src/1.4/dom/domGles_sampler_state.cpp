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
#include <dom/domGles_sampler_state.h>

daeElementRef
domGles_sampler_state::create(daeInt bytes)
{
	domGles_sampler_stateRef ref = new(bytes) domGles_sampler_state;
	return ref;
}


daeMetaElement *
domGles_sampler_state::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "gles_sampler_state" );
	_Meta->setStaticPointerAddress(&domGles_sampler_state::_Meta);
	_Meta->registerConstructor(domGles_sampler_state::create);

	// Add elements: wrap_s, wrap_t, minfilter, magfilter, mipfilter, mipmap_maxlevel, mipmap_bias, extra
    _Meta->appendElement(domGles_sampler_state::domWrap_s::registerElement(),daeOffsetOf(domGles_sampler_state,elemWrap_s));
    _Meta->appendElement(domGles_sampler_state::domWrap_t::registerElement(),daeOffsetOf(domGles_sampler_state,elemWrap_t));
    _Meta->appendElement(domGles_sampler_state::domMinfilter::registerElement(),daeOffsetOf(domGles_sampler_state,elemMinfilter));
    _Meta->appendElement(domGles_sampler_state::domMagfilter::registerElement(),daeOffsetOf(domGles_sampler_state,elemMagfilter));
    _Meta->appendElement(domGles_sampler_state::domMipfilter::registerElement(),daeOffsetOf(domGles_sampler_state,elemMipfilter));
    _Meta->appendElement(domGles_sampler_state::domMipmap_maxlevel::registerElement(),daeOffsetOf(domGles_sampler_state,elemMipmap_maxlevel));
    _Meta->appendElement(domGles_sampler_state::domMipmap_bias::registerElement(),daeOffsetOf(domGles_sampler_state,elemMipmap_bias));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domGles_sampler_state,elemExtra_array));

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_sampler_state , attrSid ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_sampler_state));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_sampler_state::domWrap_s::create(daeInt bytes)
{
	domGles_sampler_state::domWrap_sRef ref = new(bytes) domGles_sampler_state::domWrap_s;
	return ref;
}


daeMetaElement *
domGles_sampler_state::domWrap_s::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "wrap_s" );
	_Meta->setStaticPointerAddress(&domGles_sampler_state::domWrap_s::_Meta);
	_Meta->registerConstructor(domGles_sampler_state::domWrap_s::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Gles_sampler_wrap"));
		ma->setOffset( daeOffsetOf( domGles_sampler_state::domWrap_s , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_sampler_state::domWrap_s));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_sampler_state::domWrap_t::create(daeInt bytes)
{
	domGles_sampler_state::domWrap_tRef ref = new(bytes) domGles_sampler_state::domWrap_t;
	return ref;
}


daeMetaElement *
domGles_sampler_state::domWrap_t::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "wrap_t" );
	_Meta->setStaticPointerAddress(&domGles_sampler_state::domWrap_t::_Meta);
	_Meta->registerConstructor(domGles_sampler_state::domWrap_t::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Gles_sampler_wrap"));
		ma->setOffset( daeOffsetOf( domGles_sampler_state::domWrap_t , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_sampler_state::domWrap_t));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_sampler_state::domMinfilter::create(daeInt bytes)
{
	domGles_sampler_state::domMinfilterRef ref = new(bytes) domGles_sampler_state::domMinfilter;
	return ref;
}


daeMetaElement *
domGles_sampler_state::domMinfilter::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "minfilter" );
	_Meta->setStaticPointerAddress(&domGles_sampler_state::domMinfilter::_Meta);
	_Meta->registerConstructor(domGles_sampler_state::domMinfilter::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_sampler_filter_common"));
		ma->setOffset( daeOffsetOf( domGles_sampler_state::domMinfilter , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_sampler_state::domMinfilter));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_sampler_state::domMagfilter::create(daeInt bytes)
{
	domGles_sampler_state::domMagfilterRef ref = new(bytes) domGles_sampler_state::domMagfilter;
	return ref;
}


daeMetaElement *
domGles_sampler_state::domMagfilter::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "magfilter" );
	_Meta->setStaticPointerAddress(&domGles_sampler_state::domMagfilter::_Meta);
	_Meta->registerConstructor(domGles_sampler_state::domMagfilter::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_sampler_filter_common"));
		ma->setOffset( daeOffsetOf( domGles_sampler_state::domMagfilter , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_sampler_state::domMagfilter));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_sampler_state::domMipfilter::create(daeInt bytes)
{
	domGles_sampler_state::domMipfilterRef ref = new(bytes) domGles_sampler_state::domMipfilter;
	return ref;
}


daeMetaElement *
domGles_sampler_state::domMipfilter::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "mipfilter" );
	_Meta->setStaticPointerAddress(&domGles_sampler_state::domMipfilter::_Meta);
	_Meta->registerConstructor(domGles_sampler_state::domMipfilter::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_sampler_filter_common"));
		ma->setOffset( daeOffsetOf( domGles_sampler_state::domMipfilter , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_sampler_state::domMipfilter));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_sampler_state::domMipmap_maxlevel::create(daeInt bytes)
{
	domGles_sampler_state::domMipmap_maxlevelRef ref = new(bytes) domGles_sampler_state::domMipmap_maxlevel;
	return ref;
}


daeMetaElement *
domGles_sampler_state::domMipmap_maxlevel::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "mipmap_maxlevel" );
	_Meta->setStaticPointerAddress(&domGles_sampler_state::domMipmap_maxlevel::_Meta);
	_Meta->registerConstructor(domGles_sampler_state::domMipmap_maxlevel::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsUnsignedByte"));
		ma->setOffset( daeOffsetOf( domGles_sampler_state::domMipmap_maxlevel , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_sampler_state::domMipmap_maxlevel));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_sampler_state::domMipmap_bias::create(daeInt bytes)
{
	domGles_sampler_state::domMipmap_biasRef ref = new(bytes) domGles_sampler_state::domMipmap_bias;
	return ref;
}


daeMetaElement *
domGles_sampler_state::domMipmap_bias::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "mipmap_bias" );
	_Meta->setStaticPointerAddress(&domGles_sampler_state::domMipmap_bias::_Meta);
	_Meta->registerConstructor(domGles_sampler_state::domMipmap_bias::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsFloat"));
		ma->setOffset( daeOffsetOf( domGles_sampler_state::domMipmap_bias , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_sampler_state::domMipmap_bias));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domGles_sampler_state::_Meta = NULL;
daeMetaElement * domGles_sampler_state::domWrap_s::_Meta = NULL;
daeMetaElement * domGles_sampler_state::domWrap_t::_Meta = NULL;
daeMetaElement * domGles_sampler_state::domMinfilter::_Meta = NULL;
daeMetaElement * domGles_sampler_state::domMagfilter::_Meta = NULL;
daeMetaElement * domGles_sampler_state::domMipfilter::_Meta = NULL;
daeMetaElement * domGles_sampler_state::domMipmap_maxlevel::_Meta = NULL;
daeMetaElement * domGles_sampler_state::domMipmap_bias::_Meta = NULL;


