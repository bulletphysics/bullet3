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
#include <dom/domSampler.h>

daeElementRef
domSampler::create(daeInt bytes)
{
	domSamplerRef ref = new(bytes) domSampler;
	return ref;
}


daeMetaElement *
domSampler::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "sampler" );
	_Meta->setStaticPointerAddress(&domSampler::_Meta);
	_Meta->registerConstructor(domSampler::create);

	// Add elements: input
    _Meta->appendArrayElement(domInputLocal::registerElement(),daeOffsetOf(domSampler,elemInput_array),"input"); 

	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domSampler , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domSampler));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domSampler::_Meta = NULL;


