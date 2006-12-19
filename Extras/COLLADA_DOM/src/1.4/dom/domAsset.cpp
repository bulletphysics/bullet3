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
#include <dom/domAsset.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domAsset::create(daeInt bytes)
{
	domAssetRef ref = new(bytes) domAsset;
	return ref;
}


daeMetaElement *
domAsset::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "asset" );
	_Meta->registerClass(domAsset::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 0, -1 );
	mea->setName( "contributor" );
	mea->setOffset( daeOffsetOf(domAsset,elemContributor_array) );
	mea->setElementType( domAsset::domContributor::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 1, 1 );
	mea->setName( "created" );
	mea->setOffset( daeOffsetOf(domAsset,elemCreated) );
	mea->setElementType( domAsset::domCreated::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "keywords" );
	mea->setOffset( daeOffsetOf(domAsset,elemKeywords) );
	mea->setElementType( domAsset::domKeywords::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 3, 1, 1 );
	mea->setName( "modified" );
	mea->setOffset( daeOffsetOf(domAsset,elemModified) );
	mea->setElementType( domAsset::domModified::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 4, 0, 1 );
	mea->setName( "revision" );
	mea->setOffset( daeOffsetOf(domAsset,elemRevision) );
	mea->setElementType( domAsset::domRevision::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 5, 0, 1 );
	mea->setName( "subject" );
	mea->setOffset( daeOffsetOf(domAsset,elemSubject) );
	mea->setElementType( domAsset::domSubject::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 6, 0, 1 );
	mea->setName( "title" );
	mea->setOffset( daeOffsetOf(domAsset,elemTitle) );
	mea->setElementType( domAsset::domTitle::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 7, 0, 1 );
	mea->setName( "unit" );
	mea->setOffset( daeOffsetOf(domAsset,elemUnit) );
	mea->setElementType( domAsset::domUnit::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 8, 0, 1 );
	mea->setName( "up_axis" );
	mea->setOffset( daeOffsetOf(domAsset,elemUp_axis) );
	mea->setElementType( domAsset::domUp_axis::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 8 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domAsset));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domAsset::domContributor::create(daeInt bytes)
{
	domAsset::domContributorRef ref = new(bytes) domAsset::domContributor;
	return ref;
}


daeMetaElement *
domAsset::domContributor::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "contributor" );
	_Meta->registerClass(domAsset::domContributor::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "author" );
	mea->setOffset( daeOffsetOf(domAsset::domContributor,elemAuthor) );
	mea->setElementType( domAsset::domContributor::domAuthor::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "authoring_tool" );
	mea->setOffset( daeOffsetOf(domAsset::domContributor,elemAuthoring_tool) );
	mea->setElementType( domAsset::domContributor::domAuthoring_tool::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "comments" );
	mea->setOffset( daeOffsetOf(domAsset::domContributor,elemComments) );
	mea->setElementType( domAsset::domContributor::domComments::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 3, 0, 1 );
	mea->setName( "copyright" );
	mea->setOffset( daeOffsetOf(domAsset::domContributor,elemCopyright) );
	mea->setElementType( domAsset::domContributor::domCopyright::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 4, 0, 1 );
	mea->setName( "source_data" );
	mea->setOffset( daeOffsetOf(domAsset::domContributor,elemSource_data) );
	mea->setElementType( domAsset::domContributor::domSource_data::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 4 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domAsset::domContributor));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domAsset::domContributor::domAuthor::create(daeInt bytes)
{
	domAsset::domContributor::domAuthorRef ref = new(bytes) domAsset::domContributor::domAuthor;
	return ref;
}


daeMetaElement *
domAsset::domContributor::domAuthor::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "author" );
	_Meta->registerClass(domAsset::domContributor::domAuthor::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsString"));
		ma->setOffset( daeOffsetOf( domAsset::domContributor::domAuthor , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domAsset::domContributor::domAuthor));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domAsset::domContributor::domAuthoring_tool::create(daeInt bytes)
{
	domAsset::domContributor::domAuthoring_toolRef ref = new(bytes) domAsset::domContributor::domAuthoring_tool;
	return ref;
}


daeMetaElement *
domAsset::domContributor::domAuthoring_tool::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "authoring_tool" );
	_Meta->registerClass(domAsset::domContributor::domAuthoring_tool::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsString"));
		ma->setOffset( daeOffsetOf( domAsset::domContributor::domAuthoring_tool , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domAsset::domContributor::domAuthoring_tool));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domAsset::domContributor::domComments::create(daeInt bytes)
{
	domAsset::domContributor::domCommentsRef ref = new(bytes) domAsset::domContributor::domComments;
	return ref;
}


daeMetaElement *
domAsset::domContributor::domComments::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "comments" );
	_Meta->registerClass(domAsset::domContributor::domComments::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsString"));
		ma->setOffset( daeOffsetOf( domAsset::domContributor::domComments , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domAsset::domContributor::domComments));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domAsset::domContributor::domCopyright::create(daeInt bytes)
{
	domAsset::domContributor::domCopyrightRef ref = new(bytes) domAsset::domContributor::domCopyright;
	return ref;
}


daeMetaElement *
domAsset::domContributor::domCopyright::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "copyright" );
	_Meta->registerClass(domAsset::domContributor::domCopyright::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsString"));
		ma->setOffset( daeOffsetOf( domAsset::domContributor::domCopyright , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domAsset::domContributor::domCopyright));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domAsset::domContributor::domSource_data::create(daeInt bytes)
{
	domAsset::domContributor::domSource_dataRef ref = new(bytes) domAsset::domContributor::domSource_data;
	ref->_value.setContainer( (domAsset::domContributor::domSource_data*)ref );
	return ref;
}


daeMetaElement *
domAsset::domContributor::domSource_data::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "source_data" );
	_Meta->registerClass(domAsset::domContributor::domSource_data::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domAsset::domContributor::domSource_data , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domAsset::domContributor::domSource_data));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domAsset::domCreated::create(daeInt bytes)
{
	domAsset::domCreatedRef ref = new(bytes) domAsset::domCreated;
	return ref;
}


daeMetaElement *
domAsset::domCreated::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "created" );
	_Meta->registerClass(domAsset::domCreated::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsDateTime"));
		ma->setOffset( daeOffsetOf( domAsset::domCreated , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domAsset::domCreated));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domAsset::domKeywords::create(daeInt bytes)
{
	domAsset::domKeywordsRef ref = new(bytes) domAsset::domKeywords;
	return ref;
}


daeMetaElement *
domAsset::domKeywords::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "keywords" );
	_Meta->registerClass(domAsset::domKeywords::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsString"));
		ma->setOffset( daeOffsetOf( domAsset::domKeywords , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domAsset::domKeywords));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domAsset::domModified::create(daeInt bytes)
{
	domAsset::domModifiedRef ref = new(bytes) domAsset::domModified;
	return ref;
}


daeMetaElement *
domAsset::domModified::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "modified" );
	_Meta->registerClass(domAsset::domModified::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsDateTime"));
		ma->setOffset( daeOffsetOf( domAsset::domModified , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domAsset::domModified));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domAsset::domRevision::create(daeInt bytes)
{
	domAsset::domRevisionRef ref = new(bytes) domAsset::domRevision;
	return ref;
}


daeMetaElement *
domAsset::domRevision::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "revision" );
	_Meta->registerClass(domAsset::domRevision::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsString"));
		ma->setOffset( daeOffsetOf( domAsset::domRevision , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domAsset::domRevision));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domAsset::domSubject::create(daeInt bytes)
{
	domAsset::domSubjectRef ref = new(bytes) domAsset::domSubject;
	return ref;
}


daeMetaElement *
domAsset::domSubject::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "subject" );
	_Meta->registerClass(domAsset::domSubject::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsString"));
		ma->setOffset( daeOffsetOf( domAsset::domSubject , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domAsset::domSubject));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domAsset::domTitle::create(daeInt bytes)
{
	domAsset::domTitleRef ref = new(bytes) domAsset::domTitle;
	return ref;
}


daeMetaElement *
domAsset::domTitle::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "title" );
	_Meta->registerClass(domAsset::domTitle::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsString"));
		ma->setOffset( daeOffsetOf( domAsset::domTitle , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domAsset::domTitle));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domAsset::domUnit::create(daeInt bytes)
{
	domAsset::domUnitRef ref = new(bytes) domAsset::domUnit;
	return ref;
}


daeMetaElement *
domAsset::domUnit::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "unit" );
	_Meta->registerClass(domAsset::domUnit::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: meter
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "meter" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domAsset::domUnit , attrMeter ));
		ma->setContainer( _Meta );
		ma->setDefault( "1.0");
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNMTOKEN"));
		ma->setOffset( daeOffsetOf( domAsset::domUnit , attrName ));
		ma->setContainer( _Meta );
		ma->setDefault( "meter");
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domAsset::domUnit));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domAsset::domUp_axis::create(daeInt bytes)
{
	domAsset::domUp_axisRef ref = new(bytes) domAsset::domUp_axis;
	return ref;
}


daeMetaElement *
domAsset::domUp_axis::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "up_axis" );
	_Meta->registerClass(domAsset::domUp_axis::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("UpAxisType"));
		ma->setOffset( daeOffsetOf( domAsset::domUp_axis , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domAsset::domUp_axis));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domAsset::_Meta = NULL;
daeMetaElement * domAsset::domContributor::_Meta = NULL;
daeMetaElement * domAsset::domContributor::domAuthor::_Meta = NULL;
daeMetaElement * domAsset::domContributor::domAuthoring_tool::_Meta = NULL;
daeMetaElement * domAsset::domContributor::domComments::_Meta = NULL;
daeMetaElement * domAsset::domContributor::domCopyright::_Meta = NULL;
daeMetaElement * domAsset::domContributor::domSource_data::_Meta = NULL;
daeMetaElement * domAsset::domCreated::_Meta = NULL;
daeMetaElement * domAsset::domKeywords::_Meta = NULL;
daeMetaElement * domAsset::domModified::_Meta = NULL;
daeMetaElement * domAsset::domRevision::_Meta = NULL;
daeMetaElement * domAsset::domSubject::_Meta = NULL;
daeMetaElement * domAsset::domTitle::_Meta = NULL;
daeMetaElement * domAsset::domUnit::_Meta = NULL;
daeMetaElement * domAsset::domUp_axis::_Meta = NULL;


