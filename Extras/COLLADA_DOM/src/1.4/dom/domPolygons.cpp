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
#include <dom/domPolygons.h>

daeElementRef
domPolygons::create(daeInt bytes)
{
	domPolygonsRef ref = new(bytes) domPolygons;
	return ref;
}


daeMetaElement *
domPolygons::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "polygons" );
	_Meta->setStaticPointerAddress(&domPolygons::_Meta);
	_Meta->registerConstructor(domPolygons::create);

	// Add elements: input, p, ph, extra
    _Meta->appendArrayElement(domInputLocalOffset::registerElement(),daeOffsetOf(domPolygons,elemInput_array),"input"); 
    _Meta->appendArrayElement(domP::registerElement(),daeOffsetOf(domPolygons,elemP_array));
    _Meta->appendArrayElement(domPolygons::domPh::registerElement(),daeOffsetOf(domPolygons,elemPh_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domPolygons,elemExtra_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domPolygons,_contents));


	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domPolygons , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: count
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "count" );
		ma->setType( daeAtomicType::get("Uint"));
		ma->setOffset( daeOffsetOf( domPolygons , attrCount ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: material
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "material" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domPolygons , attrMaterial ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domPolygons));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domPolygons::domPh::create(daeInt bytes)
{
	domPolygons::domPhRef ref = new(bytes) domPolygons::domPh;
	return ref;
}


daeMetaElement *
domPolygons::domPh::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "ph" );
	_Meta->setStaticPointerAddress(&domPolygons::domPh::_Meta);
	_Meta->registerConstructor(domPolygons::domPh::create);

	// Add elements: p, h
    _Meta->appendElement(domP::registerElement(),daeOffsetOf(domPolygons::domPh,elemP));
    _Meta->appendArrayElement(domPolygons::domPh::domH::registerElement(),daeOffsetOf(domPolygons::domPh,elemH_array));
	
	
	_Meta->setElementSize(sizeof(domPolygons::domPh));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domPolygons::domPh::domH::create(daeInt bytes)
{
	domPolygons::domPh::domHRef ref = new(bytes) domPolygons::domPh::domH;
	return ref;
}


daeMetaElement *
domPolygons::domPh::domH::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "h" );
	_Meta->setStaticPointerAddress(&domPolygons::domPh::domH::_Meta);
	_Meta->registerConstructor(domPolygons::domPh::domH::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("ListOfUInts"));
		ma->setOffset( daeOffsetOf( domPolygons::domPh::domH , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domPolygons::domPh::domH));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domPolygons::_Meta = NULL;
daeMetaElement * domPolygons::domPh::_Meta = NULL;
daeMetaElement * domPolygons::domPh::domH::_Meta = NULL;


