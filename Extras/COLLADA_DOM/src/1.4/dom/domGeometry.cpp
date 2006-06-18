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
	_Meta->setStaticPointerAddress(&domGeometry::_Meta);
	_Meta->registerConstructor(domGeometry::create);

	// Add elements: asset, convex_mesh, mesh, spline, extra
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domGeometry,elemAsset));
    _Meta->appendElement(domConvex_mesh::registerElement(),daeOffsetOf(domGeometry,elemConvex_mesh));
    _Meta->appendElement(domMesh::registerElement(),daeOffsetOf(domGeometry,elemMesh));
    _Meta->appendElement(domSpline::registerElement(),daeOffsetOf(domGeometry,elemSpline));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domGeometry,elemExtra_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domGeometry,_contents));


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


