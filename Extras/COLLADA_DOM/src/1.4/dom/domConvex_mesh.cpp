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
#include <dom/domConvex_mesh.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domConvex_mesh::create(daeInt bytes)
{
	domConvex_meshRef ref = new(bytes) domConvex_mesh;
	ref->attrConvex_hull_of.setContainer( (domConvex_mesh*)ref );
	return ref;
}


daeMetaElement *
domConvex_mesh::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "convex_mesh" );
	_Meta->registerClass(domConvex_mesh::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 0, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, -1 );
	mea->setName( "source" );
	mea->setOffset( daeOffsetOf(domConvex_mesh,elemSource_array) );
	mea->setElementType( domSource::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 1, 1 );
	mea->setName( "vertices" );
	mea->setOffset( daeOffsetOf(domConvex_mesh,elemVertices) );
	mea->setElementType( domVertices::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaChoice( _Meta, cm, 2, 0, -1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "lines" );
	mea->setOffset( daeOffsetOf(domConvex_mesh,elemLines_array) );
	mea->setElementType( domLines::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "linestrips" );
	mea->setOffset( daeOffsetOf(domConvex_mesh,elemLinestrips_array) );
	mea->setElementType( domLinestrips::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "polygons" );
	mea->setOffset( daeOffsetOf(domConvex_mesh,elemPolygons_array) );
	mea->setElementType( domPolygons::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "polylist" );
	mea->setOffset( daeOffsetOf(domConvex_mesh,elemPolylist_array) );
	mea->setElementType( domPolylist::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "triangles" );
	mea->setOffset( daeOffsetOf(domConvex_mesh,elemTriangles_array) );
	mea->setElementType( domTriangles::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "trifans" );
	mea->setOffset( daeOffsetOf(domConvex_mesh,elemTrifans_array) );
	mea->setElementType( domTrifans::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "tristrips" );
	mea->setOffset( daeOffsetOf(domConvex_mesh,elemTristrips_array) );
	mea->setElementType( domTristrips::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3003, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domConvex_mesh,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 3003 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domConvex_mesh,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domConvex_mesh,_contentsOrder));


	//	Add attribute: convex_hull_of
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "convex_hull_of" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domConvex_mesh , attrConvex_hull_of ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domConvex_mesh));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domConvex_mesh::_Meta = NULL;


