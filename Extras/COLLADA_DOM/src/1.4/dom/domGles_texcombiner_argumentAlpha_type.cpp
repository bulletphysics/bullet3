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
#include <dom/domGles_texcombiner_argumentAlpha_type.h>

daeElementRef
domGles_texcombiner_argumentAlpha_type::create(daeInt bytes)
{
	domGles_texcombiner_argumentAlpha_typeRef ref = new(bytes) domGles_texcombiner_argumentAlpha_type;
	return ref;
}


daeMetaElement *
domGles_texcombiner_argumentAlpha_type::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "gles_texcombiner_argumentAlpha_type" );
	_Meta->setStaticPointerAddress(&domGles_texcombiner_argumentAlpha_type::_Meta);
	_Meta->registerConstructor(domGles_texcombiner_argumentAlpha_type::create);


	//	Add attribute: source
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "source" );
		ma->setType( daeAtomicType::get("Gles_texcombiner_source_enums"));
		ma->setOffset( daeOffsetOf( domGles_texcombiner_argumentAlpha_type , attrSource ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: operand
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "operand" );
		ma->setType( daeAtomicType::get("Gles_texcombiner_operandAlpha_enums"));
		ma->setOffset( daeOffsetOf( domGles_texcombiner_argumentAlpha_type , attrOperand ));
		ma->setContainer( _Meta );
		ma->setDefault( "SRC_ALPHA");
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: unit
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "unit" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_texcombiner_argumentAlpha_type , attrUnit ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_texcombiner_argumentAlpha_type));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domGles_texcombiner_argumentAlpha_type::_Meta = NULL;


