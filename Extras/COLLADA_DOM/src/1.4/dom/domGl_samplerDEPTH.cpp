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
#include <dom/domGl_samplerDEPTH.h>

daeElementRef
domGl_samplerDEPTH::create(daeInt bytes)
{
	domGl_samplerDEPTHRef ref = new(bytes) domGl_samplerDEPTH;
	return ref;
}


daeMetaElement *
domGl_samplerDEPTH::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "gl_samplerDEPTH" );
	_Meta->setStaticPointerAddress(&domGl_samplerDEPTH::_Meta);
	_Meta->registerConstructor(domGl_samplerDEPTH::create);

	// Add elements: source, wrap_s, wrap_t, minfilter, magfilter
    _Meta->appendElement(domSource::registerElement(),daeOffsetOf(domGl_samplerDEPTH,elemSource));
    _Meta->appendElement(domWrap_s::registerElement(),daeOffsetOf(domGl_samplerDEPTH,elemWrap_s));
    _Meta->appendElement(domWrap_t::registerElement(),daeOffsetOf(domGl_samplerDEPTH,elemWrap_t));
    _Meta->appendElement(domMinfilter::registerElement(),daeOffsetOf(domGl_samplerDEPTH,elemMinfilter));
    _Meta->appendElement(domMagfilter::registerElement(),daeOffsetOf(domGl_samplerDEPTH,elemMagfilter));
	
	
	_Meta->setElementSize(sizeof(domGl_samplerDEPTH));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domGl_samplerDEPTH::_Meta = NULL;


