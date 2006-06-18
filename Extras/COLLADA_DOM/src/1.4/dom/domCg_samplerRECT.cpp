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
#include <dom/domCg_samplerRECT.h>

daeElementRef
domCg_samplerRECT::create(daeInt bytes)
{
	domCg_samplerRECTRef ref = new(bytes) domCg_samplerRECT;
	return ref;
}


daeMetaElement *
domCg_samplerRECT::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "cg_samplerRECT" );
	_Meta->setStaticPointerAddress(&domCg_samplerRECT::_Meta);
	_Meta->registerConstructor(domCg_samplerRECT::create);

	// Add elements: source, wrap_s, wrap_t, minfilter, magfilter, mipfilter, border_color, mipmap_maxlevel, mipmap_bias
    _Meta->appendElement(domSource::registerElement(),daeOffsetOf(domCg_samplerRECT,elemSource));
    _Meta->appendElement(domWrap_s::registerElement(),daeOffsetOf(domCg_samplerRECT,elemWrap_s));
    _Meta->appendElement(domWrap_t::registerElement(),daeOffsetOf(domCg_samplerRECT,elemWrap_t));
    _Meta->appendElement(domMinfilter::registerElement(),daeOffsetOf(domCg_samplerRECT,elemMinfilter));
    _Meta->appendElement(domMagfilter::registerElement(),daeOffsetOf(domCg_samplerRECT,elemMagfilter));
    _Meta->appendElement(domMipfilter::registerElement(),daeOffsetOf(domCg_samplerRECT,elemMipfilter));
    _Meta->appendElement(domBorder_color::registerElement(),daeOffsetOf(domCg_samplerRECT,elemBorder_color));
    _Meta->appendElement(domMipmap_maxlevel::registerElement(),daeOffsetOf(domCg_samplerRECT,elemMipmap_maxlevel));
    _Meta->appendElement(domMipmap_bias::registerElement(),daeOffsetOf(domCg_samplerRECT,elemMipmap_bias));
	
	
	_Meta->setElementSize(sizeof(domCg_samplerRECT));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domCg_samplerRECT::_Meta = NULL;


