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
	_Meta->setStaticPointerAddress(&domImage::_Meta);
	_Meta->registerConstructor(domImage::create);

	// Add elements: asset, data, init_from, extra
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domImage,elemAsset));
    _Meta->appendElement(domImage::domData::registerElement(),daeOffsetOf(domImage,elemData));
    _Meta->appendElement(domImage::domInit_from::registerElement(),daeOffsetOf(domImage,elemInit_from));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domImage,elemExtra_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domImage,_contents));


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
	_Meta->setStaticPointerAddress(&domImage::domData::_Meta);
	_Meta->registerConstructor(domImage::domData::create);

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
	_Meta->setStaticPointerAddress(&domImage::domInit_from::_Meta);
	_Meta->registerConstructor(domImage::domInit_from::create);

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


