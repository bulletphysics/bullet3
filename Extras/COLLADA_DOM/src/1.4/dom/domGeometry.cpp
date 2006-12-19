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
#include <dom/domGeometry.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGeometry::create(daeInt bytes)
{
	domGeometryRef ref = new(bytes) domGeometry;
	return ref;
}


daeMetaElement *
domGeometry::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "geometry" );
	_Meta->registerClass(domGeometry::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domGeometry,elemAsset) );
	mea->setElementType( domAsset::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaChoice( _Meta, cm, 1, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "convex_mesh" );
	mea->setOffset( daeOffsetOf(domGeometry,elemConvex_mesh) );
	mea->setElementType( domConvex_mesh::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "mesh" );
	mea->setOffset( daeOffsetOf(domGeometry,elemMesh) );
	mea->setElementType( domMesh::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "spline" );
	mea->setOffset( daeOffsetOf(domGeometry,elemSpline) );
	mea->setElementType( domSpline::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domGeometry,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 2 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domGeometry,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domGeometry,_contentsOrder));


	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domGeometry , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGeometry , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGeometry));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domGeometry::_Meta = NULL;


