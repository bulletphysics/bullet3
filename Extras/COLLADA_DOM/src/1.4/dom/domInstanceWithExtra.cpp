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
#include <dom/domInstanceWithExtra.h>

daeElementRef
domInstanceWithExtra::create(daeInt bytes)
{
	domInstanceWithExtraRef ref = new(bytes) domInstanceWithExtra;
	ref->attrUrl.setContainer( (domInstanceWithExtra*)ref );
	return ref;
}


daeMetaElement *
domInstanceWithExtra::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "InstanceWithExtra" );
	_Meta->setStaticPointerAddress(&domInstanceWithExtra::_Meta);
	_Meta->registerConstructor(domInstanceWithExtra::create);

	// Add elements: extra
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domInstanceWithExtra,elemExtra_array));

	//	Add attribute: url
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "url" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domInstanceWithExtra , attrUrl ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domInstanceWithExtra));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domInstanceWithExtra::_Meta = NULL;


