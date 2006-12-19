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
#include <dom/domGles_texture_constant_type.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGles_texture_constant_type::create(daeInt bytes)
{
	domGles_texture_constant_typeRef ref = new(bytes) domGles_texture_constant_type;
	return ref;
}


daeMetaElement *
domGles_texture_constant_type::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "gles_texture_constant_type" );
	_Meta->registerClass(domGles_texture_constant_type::create, &_Meta);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGles_texture_constant_type , attrValue ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_texture_constant_type , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_texture_constant_type));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domGles_texture_constant_type::_Meta = NULL;


