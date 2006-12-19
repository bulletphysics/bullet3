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
#include <dom/domInstance_light.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domInstance_light::create(daeInt bytes)
{
	domInstance_lightRef ref = new(bytes) domInstance_light;
	return ref;
}


daeMetaElement *
domInstance_light::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "instance_light" );
	_Meta->registerClass(domInstance_light::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domInstance_light,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	

	//	Add attribute: url
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "url" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domInstance_light , attrUrl ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_light , attrSid ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_light , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domInstance_light));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domInstance_light::_Meta = NULL;


