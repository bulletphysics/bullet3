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
	_Meta->setStaticPointerAddress(&domConvex_mesh::_Meta);
	_Meta->registerConstructor(domConvex_mesh::create);

	// Add elements: source, vertices, lines, linestrips, polygons, polylist, triangles, trifans, tristrips, extra
    _Meta->appendArrayElement(domSource::registerElement(),daeOffsetOf(domConvex_mesh,elemSource_array));
    _Meta->appendElement(domVertices::registerElement(),daeOffsetOf(domConvex_mesh,elemVertices));
    _Meta->appendArrayElement(domLines::registerElement(),daeOffsetOf(domConvex_mesh,elemLines_array));
    _Meta->appendArrayElement(domLinestrips::registerElement(),daeOffsetOf(domConvex_mesh,elemLinestrips_array));
    _Meta->appendArrayElement(domPolygons::registerElement(),daeOffsetOf(domConvex_mesh,elemPolygons_array));
    _Meta->appendArrayElement(domPolylist::registerElement(),daeOffsetOf(domConvex_mesh,elemPolylist_array));
    _Meta->appendArrayElement(domTriangles::registerElement(),daeOffsetOf(domConvex_mesh,elemTriangles_array));
    _Meta->appendArrayElement(domTrifans::registerElement(),daeOffsetOf(domConvex_mesh,elemTrifans_array));
    _Meta->appendArrayElement(domTristrips::registerElement(),daeOffsetOf(domConvex_mesh,elemTristrips_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domConvex_mesh,elemExtra_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domConvex_mesh,_contents));


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


