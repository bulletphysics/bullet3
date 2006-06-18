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
#include <dom/domSkin.h>

daeElementRef
domSkin::create(daeInt bytes)
{
	domSkinRef ref = new(bytes) domSkin;
	ref->attrSource.setContainer( (domSkin*)ref );
	return ref;
}


daeMetaElement *
domSkin::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "skin" );
	_Meta->setStaticPointerAddress(&domSkin::_Meta);
	_Meta->registerConstructor(domSkin::create);

	// Add elements: bind_shape_matrix, source, joints, vertex_weights, extra
    _Meta->appendElement(domSkin::domBind_shape_matrix::registerElement(),daeOffsetOf(domSkin,elemBind_shape_matrix));
    _Meta->appendArrayElement(domSource::registerElement(),daeOffsetOf(domSkin,elemSource_array));
    _Meta->appendElement(domSkin::domJoints::registerElement(),daeOffsetOf(domSkin,elemJoints));
    _Meta->appendElement(domSkin::domVertex_weights::registerElement(),daeOffsetOf(domSkin,elemVertex_weights));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domSkin,elemExtra_array));

	//	Add attribute: source
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "source" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domSkin , attrSource ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domSkin));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domSkin::domBind_shape_matrix::create(daeInt bytes)
{
	domSkin::domBind_shape_matrixRef ref = new(bytes) domSkin::domBind_shape_matrix;
	return ref;
}


daeMetaElement *
domSkin::domBind_shape_matrix::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bind_shape_matrix" );
	_Meta->setStaticPointerAddress(&domSkin::domBind_shape_matrix::_Meta);
	_Meta->registerConstructor(domSkin::domBind_shape_matrix::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float4x4"));
		ma->setOffset( daeOffsetOf( domSkin::domBind_shape_matrix , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domSkin::domBind_shape_matrix));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domSkin::domJoints::create(daeInt bytes)
{
	domSkin::domJointsRef ref = new(bytes) domSkin::domJoints;
	return ref;
}


daeMetaElement *
domSkin::domJoints::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "joints" );
	_Meta->setStaticPointerAddress(&domSkin::domJoints::_Meta);
	_Meta->registerConstructor(domSkin::domJoints::create);

	// Add elements: input, extra
    _Meta->appendArrayElement(domInputLocal::registerElement(),daeOffsetOf(domSkin::domJoints,elemInput_array),"input"); 
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domSkin::domJoints,elemExtra_array));
	
	
	_Meta->setElementSize(sizeof(domSkin::domJoints));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domSkin::domVertex_weights::create(daeInt bytes)
{
	domSkin::domVertex_weightsRef ref = new(bytes) domSkin::domVertex_weights;
	return ref;
}


daeMetaElement *
domSkin::domVertex_weights::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "vertex_weights" );
	_Meta->setStaticPointerAddress(&domSkin::domVertex_weights::_Meta);
	_Meta->registerConstructor(domSkin::domVertex_weights::create);

	// Add elements: input, vcount, v, extra
    _Meta->appendArrayElement(domInputLocalOffset::registerElement(),daeOffsetOf(domSkin::domVertex_weights,elemInput_array),"input"); 
    _Meta->appendElement(domSkin::domVertex_weights::domVcount::registerElement(),daeOffsetOf(domSkin::domVertex_weights,elemVcount));
    _Meta->appendElement(domSkin::domVertex_weights::domV::registerElement(),daeOffsetOf(domSkin::domVertex_weights,elemV));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domSkin::domVertex_weights,elemExtra_array));

	//	Add attribute: count
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "count" );
		ma->setType( daeAtomicType::get("Uint"));
		ma->setOffset( daeOffsetOf( domSkin::domVertex_weights , attrCount ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domSkin::domVertex_weights));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domSkin::domVertex_weights::domVcount::create(daeInt bytes)
{
	domSkin::domVertex_weights::domVcountRef ref = new(bytes) domSkin::domVertex_weights::domVcount;
	return ref;
}


daeMetaElement *
domSkin::domVertex_weights::domVcount::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "vcount" );
	_Meta->setStaticPointerAddress(&domSkin::domVertex_weights::domVcount::_Meta);
	_Meta->registerConstructor(domSkin::domVertex_weights::domVcount::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("ListOfUInts"));
		ma->setOffset( daeOffsetOf( domSkin::domVertex_weights::domVcount , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domSkin::domVertex_weights::domVcount));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domSkin::domVertex_weights::domV::create(daeInt bytes)
{
	domSkin::domVertex_weights::domVRef ref = new(bytes) domSkin::domVertex_weights::domV;
	return ref;
}


daeMetaElement *
domSkin::domVertex_weights::domV::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "v" );
	_Meta->setStaticPointerAddress(&domSkin::domVertex_weights::domV::_Meta);
	_Meta->registerConstructor(domSkin::domVertex_weights::domV::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("ListOfInts"));
		ma->setOffset( daeOffsetOf( domSkin::domVertex_weights::domV , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domSkin::domVertex_weights::domV));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domSkin::_Meta = NULL;
daeMetaElement * domSkin::domBind_shape_matrix::_Meta = NULL;
daeMetaElement * domSkin::domJoints::_Meta = NULL;
daeMetaElement * domSkin::domVertex_weights::_Meta = NULL;
daeMetaElement * domSkin::domVertex_weights::domVcount::_Meta = NULL;
daeMetaElement * domSkin::domVertex_weights::domV::_Meta = NULL;


