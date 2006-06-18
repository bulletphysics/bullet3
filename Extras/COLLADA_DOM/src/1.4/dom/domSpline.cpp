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
#include <dom/domSpline.h>

daeElementRef
domSpline::create(daeInt bytes)
{
	domSplineRef ref = new(bytes) domSpline;
	return ref;
}


daeMetaElement *
domSpline::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "spline" );
	_Meta->setStaticPointerAddress(&domSpline::_Meta);
	_Meta->registerConstructor(domSpline::create);

	// Add elements: source, control_vertices, extra
    _Meta->appendArrayElement(domSource::registerElement(),daeOffsetOf(domSpline,elemSource_array));
    _Meta->appendElement(domSpline::domControl_vertices::registerElement(),daeOffsetOf(domSpline,elemControl_vertices));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domSpline,elemExtra_array));

	//	Add attribute: closed
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "closed" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domSpline , attrClosed ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domSpline));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domSpline::domControl_vertices::create(daeInt bytes)
{
	domSpline::domControl_verticesRef ref = new(bytes) domSpline::domControl_vertices;
	return ref;
}


daeMetaElement *
domSpline::domControl_vertices::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "control_vertices" );
	_Meta->setStaticPointerAddress(&domSpline::domControl_vertices::_Meta);
	_Meta->registerConstructor(domSpline::domControl_vertices::create);

	// Add elements: input, extra
    _Meta->appendArrayElement(domInputLocal::registerElement(),daeOffsetOf(domSpline::domControl_vertices,elemInput_array),"input"); 
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domSpline::domControl_vertices,elemExtra_array));
	
	
	_Meta->setElementSize(sizeof(domSpline::domControl_vertices));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domSpline::_Meta = NULL;
daeMetaElement * domSpline::domControl_vertices::_Meta = NULL;


