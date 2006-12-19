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
#include <dom/domCg_connect_param.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domCg_connect_param::create(daeInt bytes)
{
	domCg_connect_paramRef ref = new(bytes) domCg_connect_param;
	return ref;
}


daeMetaElement *
domCg_connect_param::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "cg_connect_param" );
	_Meta->registerClass(domCg_connect_param::create, &_Meta);


	//	Add attribute: ref
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( daeAtomicType::get("Cg_identifier"));
		ma->setOffset( daeOffsetOf( domCg_connect_param , attrRef ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_connect_param));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domCg_connect_param::_Meta = NULL;


