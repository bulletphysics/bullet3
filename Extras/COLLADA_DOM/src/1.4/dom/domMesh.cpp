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
#include <dom/domMesh.h>

daeElementRef
domMesh::create(daeInt bytes)
{
	domMeshRef ref = new(bytes) domMesh;
	return ref;
}


daeMetaElement *
domMesh::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "mesh" );
	_Meta->setStaticPointerAddress(&domMesh::_Meta);
	_Meta->registerConstructor(domMesh::create);

	// Add elements: source, vertices, lines, linestrips, polygons, polylist, triangles, trifans, tristrips, extra
    _Meta->appendArrayElement(domSource::registerElement(),daeOffsetOf(domMesh,elemSource_array));
    _Meta->appendElement(domVertices::registerElement(),daeOffsetOf(domMesh,elemVertices));
    _Meta->appendArrayElement(domLines::registerElement(),daeOffsetOf(domMesh,elemLines_array));
    _Meta->appendArrayElement(domLinestrips::registerElement(),daeOffsetOf(domMesh,elemLinestrips_array));
    _Meta->appendArrayElement(domPolygons::registerElement(),daeOffsetOf(domMesh,elemPolygons_array));
    _Meta->appendArrayElement(domPolylist::registerElement(),daeOffsetOf(domMesh,elemPolylist_array));
    _Meta->appendArrayElement(domTriangles::registerElement(),daeOffsetOf(domMesh,elemTriangles_array));
    _Meta->appendArrayElement(domTrifans::registerElement(),daeOffsetOf(domMesh,elemTrifans_array));
    _Meta->appendArrayElement(domTristrips::registerElement(),daeOffsetOf(domMesh,elemTristrips_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domMesh,elemExtra_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domMesh,_contents));

	
	
	_Meta->setElementSize(sizeof(domMesh));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domMesh::_Meta = NULL;


