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
#include <dom/domGles_texture_unit.h>

daeElementRef
domGles_texture_unit::create(daeInt bytes)
{
	domGles_texture_unitRef ref = new(bytes) domGles_texture_unit;
	return ref;
}


daeMetaElement *
domGles_texture_unit::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "gles_texture_unit" );
	_Meta->setStaticPointerAddress(&domGles_texture_unit::_Meta);
	_Meta->registerConstructor(domGles_texture_unit::create);

	// Add elements: surface, sampler_state, texcoord
    _Meta->appendElement(domGles_texture_unit::domSurface::registerElement(),daeOffsetOf(domGles_texture_unit,elemSurface));
    _Meta->appendElement(domGles_texture_unit::domSampler_state::registerElement(),daeOffsetOf(domGles_texture_unit,elemSampler_state));
    _Meta->appendElement(domGles_texture_unit::domTexcoord::registerElement(),daeOffsetOf(domGles_texture_unit,elemTexcoord));

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_texture_unit , attrSid ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_texture_unit));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_texture_unit::domSurface::create(daeInt bytes)
{
	domGles_texture_unit::domSurfaceRef ref = new(bytes) domGles_texture_unit::domSurface;
	return ref;
}


daeMetaElement *
domGles_texture_unit::domSurface::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "surface" );
	_Meta->setStaticPointerAddress(&domGles_texture_unit::domSurface::_Meta);
	_Meta->registerConstructor(domGles_texture_unit::domSurface::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_texture_unit::domSurface , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_texture_unit::domSurface));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_texture_unit::domSampler_state::create(daeInt bytes)
{
	domGles_texture_unit::domSampler_stateRef ref = new(bytes) domGles_texture_unit::domSampler_state;
	return ref;
}


daeMetaElement *
domGles_texture_unit::domSampler_state::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "sampler_state" );
	_Meta->setStaticPointerAddress(&domGles_texture_unit::domSampler_state::_Meta);
	_Meta->registerConstructor(domGles_texture_unit::domSampler_state::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_texture_unit::domSampler_state , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_texture_unit::domSampler_state));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_texture_unit::domTexcoord::create(daeInt bytes)
{
	domGles_texture_unit::domTexcoordRef ref = new(bytes) domGles_texture_unit::domTexcoord;
	return ref;
}


daeMetaElement *
domGles_texture_unit::domTexcoord::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "texcoord" );
	_Meta->setStaticPointerAddress(&domGles_texture_unit::domTexcoord::_Meta);
	_Meta->registerConstructor(domGles_texture_unit::domTexcoord::create);


	//	Add attribute: semantic
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "semantic" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_texture_unit::domTexcoord , attrSemantic ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_texture_unit::domTexcoord));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domGles_texture_unit::_Meta = NULL;
daeMetaElement * domGles_texture_unit::domSurface::_Meta = NULL;
daeMetaElement * domGles_texture_unit::domSampler_state::_Meta = NULL;
daeMetaElement * domGles_texture_unit::domTexcoord::_Meta = NULL;


