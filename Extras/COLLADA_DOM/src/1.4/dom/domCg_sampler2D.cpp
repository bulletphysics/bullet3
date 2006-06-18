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
#include <dom/domCg_sampler2D.h>

daeElementRef
domCg_sampler2D::create(daeInt bytes)
{
	domCg_sampler2DRef ref = new(bytes) domCg_sampler2D;
	return ref;
}


daeMetaElement *
domCg_sampler2D::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "cg_sampler2D" );
	_Meta->setStaticPointerAddress(&domCg_sampler2D::_Meta);
	_Meta->registerConstructor(domCg_sampler2D::create);

	// Add elements: source, wrap_s, wrap_t, minfilter, magfilter, mipfilter, border_color, mipmap_maxlevel, mipmap_bias
    _Meta->appendElement(domSource::registerElement(),daeOffsetOf(domCg_sampler2D,elemSource));
    _Meta->appendElement(domWrap_s::registerElement(),daeOffsetOf(domCg_sampler2D,elemWrap_s));
    _Meta->appendElement(domWrap_t::registerElement(),daeOffsetOf(domCg_sampler2D,elemWrap_t));
    _Meta->appendElement(domMinfilter::registerElement(),daeOffsetOf(domCg_sampler2D,elemMinfilter));
    _Meta->appendElement(domMagfilter::registerElement(),daeOffsetOf(domCg_sampler2D,elemMagfilter));
    _Meta->appendElement(domMipfilter::registerElement(),daeOffsetOf(domCg_sampler2D,elemMipfilter));
    _Meta->appendElement(domBorder_color::registerElement(),daeOffsetOf(domCg_sampler2D,elemBorder_color));
    _Meta->appendElement(domMipmap_maxlevel::registerElement(),daeOffsetOf(domCg_sampler2D,elemMipmap_maxlevel));
    _Meta->appendElement(domMipmap_bias::registerElement(),daeOffsetOf(domCg_sampler2D,elemMipmap_bias));
	
	
	_Meta->setElementSize(sizeof(domCg_sampler2D));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domCg_sampler2D::_Meta = NULL;


