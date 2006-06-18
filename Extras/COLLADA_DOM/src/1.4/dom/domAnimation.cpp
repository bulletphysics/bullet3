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
#include <dom/domAnimation.h>

daeElementRef
domAnimation::create(daeInt bytes)
{
	domAnimationRef ref = new(bytes) domAnimation;
	return ref;
}


daeMetaElement *
domAnimation::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "animation" );
	_Meta->setStaticPointerAddress(&domAnimation::_Meta);
	_Meta->registerConstructor(domAnimation::create);

	// Add elements: asset, source, sampler, channel, animation, extra
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domAnimation,elemAsset));
    _Meta->appendArrayElement(domSource::registerElement(),daeOffsetOf(domAnimation,elemSource_array));
    _Meta->appendArrayElement(domSampler::registerElement(),daeOffsetOf(domAnimation,elemSampler_array));
    _Meta->appendArrayElement(domChannel::registerElement(),daeOffsetOf(domAnimation,elemChannel_array));
    _Meta->appendArrayElement(domAnimation::registerElement(),daeOffsetOf(domAnimation,elemAnimation_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domAnimation,elemExtra_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domAnimation,_contents));


	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domAnimation , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domAnimation , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domAnimation));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domAnimation::_Meta = NULL;


