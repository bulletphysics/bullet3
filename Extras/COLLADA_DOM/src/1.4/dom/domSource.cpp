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
#include <dom/domSource.h>

daeElementRef
domSource::create(daeInt bytes)
{
	domSourceRef ref = new(bytes) domSource;
	return ref;
}


daeMetaElement *
domSource::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "source" );
	_Meta->setStaticPointerAddress(&domSource::_Meta);
	_Meta->registerConstructor(domSource::create);

	// Add elements: asset, IDREF_array, Name_array, bool_array, float_array, int_array, technique_common, technique
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domSource,elemAsset));
    _Meta->appendElement(domIDREF_array::registerElement(),daeOffsetOf(domSource,elemIDREF_array));
    _Meta->appendElement(domName_array::registerElement(),daeOffsetOf(domSource,elemName_array));
    _Meta->appendElement(domBool_array::registerElement(),daeOffsetOf(domSource,elemBool_array));
    _Meta->appendElement(domFloat_array::registerElement(),daeOffsetOf(domSource,elemFloat_array));
    _Meta->appendElement(domInt_array::registerElement(),daeOffsetOf(domSource,elemInt_array));
    _Meta->appendElement(domSource::domTechnique_common::registerElement(),daeOffsetOf(domSource,elemTechnique_common));
    _Meta->appendArrayElement(domTechnique::registerElement(),daeOffsetOf(domSource,elemTechnique_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domSource,_contents));


	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domSource , attrId ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domSource , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domSource));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domSource::domTechnique_common::create(daeInt bytes)
{
	domSource::domTechnique_commonRef ref = new(bytes) domSource::domTechnique_common;
	return ref;
}


daeMetaElement *
domSource::domTechnique_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "technique_common" );
	_Meta->setStaticPointerAddress(&domSource::domTechnique_common::_Meta);
	_Meta->registerConstructor(domSource::domTechnique_common::create);

	// Add elements: accessor
    _Meta->appendElement(domAccessor::registerElement(),daeOffsetOf(domSource::domTechnique_common,elemAccessor));
	
	
	_Meta->setElementSize(sizeof(domSource::domTechnique_common));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domSource::_Meta = NULL;
daeMetaElement * domSource::domTechnique_common::_Meta = NULL;


