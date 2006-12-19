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
#include <dom/domFx_surface_init_from_common.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFx_surface_init_from_common::create(daeInt bytes)
{
	domFx_surface_init_from_commonRef ref = new(bytes) domFx_surface_init_from_common;
	return ref;
}


daeMetaElement *
domFx_surface_init_from_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fx_surface_init_from_common" );
	_Meta->registerClass(domFx_surface_init_from_common::create, &_Meta);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsIDREF"));
		ma->setOffset( daeOffsetOf( domFx_surface_init_from_common , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: mip
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "mip" );
		ma->setType( daeAtomicType::get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domFx_surface_init_from_common , attrMip ));
		ma->setContainer( _Meta );
		ma->setDefault( "0");
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: slice
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "slice" );
		ma->setType( daeAtomicType::get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domFx_surface_init_from_common , attrSlice ));
		ma->setContainer( _Meta );
		ma->setDefault( "0");
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: face
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "face" );
		ma->setType( daeAtomicType::get("Fx_surface_face_enum"));
		ma->setOffset( daeOffsetOf( domFx_surface_init_from_common , attrFace ));
		ma->setContainer( _Meta );
		ma->setDefault( "POSITIVE_X");
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_init_from_common));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domFx_surface_init_from_common::_Meta = NULL;


