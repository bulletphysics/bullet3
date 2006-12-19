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
#include <dom/domImage.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domImage::create(daeInt bytes)
{
	domImageRef ref = new(bytes) domImage;
	return ref;
}


daeMetaElement *
domImage::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "image" );
	_Meta->registerClass(domImage::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domImage,elemAsset) );
	mea->setElementType( domAsset::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaChoice( _Meta, cm, 1, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "data" );
	mea->setOffset( daeOffsetOf(domImage,elemData) );
	mea->setElementType( domImage::domData::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "init_from" );
	mea->setOffset( daeOffsetOf(domImage,elemInit_from) );
	mea->setElementType( domImage::domInit_from::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domImage,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 2 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domImage,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domImage,_contentsOrder));


	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domImage , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domImage , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: format
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "format" );
		ma->setType( daeAtomicType::get("xsToken"));
		ma->setOffset( daeOffsetOf( domImage , attrFormat ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: height
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "height" );
		ma->setType( daeAtomicType::get("Uint"));
		ma->setOffset( daeOffsetOf( domImage , attrHeight ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: width
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "width" );
		ma->setType( daeAtomicType::get("Uint"));
		ma->setOffset( daeOffsetOf( domImage , attrWidth ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: depth
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "depth" );
		ma->setType( daeAtomicType::get("Uint"));
		ma->setOffset( daeOffsetOf( domImage , attrDepth ));
		ma->setContainer( _Meta );
		ma->setDefault( "1");
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domImage));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domImage::domData::create(daeInt bytes)
{
	domImage::domDataRef ref = new(bytes) domImage::domData;
	return ref;
}


daeMetaElement *
domImage::domData::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "data" );
	_Meta->registerClass(domImage::domData::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("ListOfHexBinary"));
		ma->setOffset( daeOffsetOf( domImage::domData , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domImage::domData));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domImage::domInit_from::create(daeInt bytes)
{
	domImage::domInit_fromRef ref = new(bytes) domImage::domInit_from;
	ref->_value.setContainer( (domImage::domInit_from*)ref );
	return ref;
}


daeMetaElement *
domImage::domInit_from::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "init_from" );
	_Meta->registerClass(domImage::domInit_from::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domImage::domInit_from , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domImage::domInit_from));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domImage::_Meta = NULL;
daeMetaElement * domImage::domData::_Meta = NULL;
daeMetaElement * domImage::domInit_from::_Meta = NULL;


