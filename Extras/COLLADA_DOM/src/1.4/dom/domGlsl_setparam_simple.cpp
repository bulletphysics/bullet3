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
#include <dom/domGlsl_setparam_simple.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGlsl_setparam_simple::create(daeInt bytes)
{
	domGlsl_setparam_simpleRef ref = new(bytes) domGlsl_setparam_simple;
	return ref;
}


daeMetaElement *
domGlsl_setparam_simple::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "glsl_setparam_simple" );
	_Meta->registerClass(domGlsl_setparam_simple::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 0, -1 );
	mea->setName( "annotate" );
	mea->setOffset( daeOffsetOf(domGlsl_setparam_simple,elemAnnotate_array) );
	mea->setElementType( domFx_annotate_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 1, 1 );
	mea->setName( "glsl_param_type" );
	mea->setOffset( daeOffsetOf(domGlsl_setparam_simple,elemGlsl_param_type) );
	mea->setElementType( domGlsl_param_type::registerElement() );
	cm->appendChild( new daeMetaGroup( mea, _Meta, cm, 1, 1, 1 ) );
	
	cm->setMaxOrdinal( 1 );
	_Meta->setCMRoot( cm );	

	//	Add attribute: ref
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( daeAtomicType::get("Glsl_identifier"));
		ma->setOffset( daeOffsetOf( domGlsl_setparam_simple , attrRef ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_setparam_simple));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domGlsl_setparam_simple::_Meta = NULL;


