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
#include <dom/domBox.h>

daeElementRef
domBox::create(daeInt bytes)
{
	domBoxRef ref = new(bytes) domBox;
	return ref;
}


daeMetaElement *
domBox::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "box" );
	_Meta->setStaticPointerAddress(&domBox::_Meta);
	_Meta->registerConstructor(domBox::create);

	// Add elements: half_extents, extra
    _Meta->appendElement(domBox::domHalf_extents::registerElement(),daeOffsetOf(domBox,elemHalf_extents));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domBox,elemExtra_array));
	
	
	_Meta->setElementSize(sizeof(domBox));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domBox::domHalf_extents::create(daeInt bytes)
{
	domBox::domHalf_extentsRef ref = new(bytes) domBox::domHalf_extents;
	return ref;
}


daeMetaElement *
domBox::domHalf_extents::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "half_extents" );
	_Meta->setStaticPointerAddress(&domBox::domHalf_extents::_Meta);
	_Meta->registerConstructor(domBox::domHalf_extents::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float3"));
		ma->setOffset( daeOffsetOf( domBox::domHalf_extents , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domBox::domHalf_extents));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domBox::_Meta = NULL;
daeMetaElement * domBox::domHalf_extents::_Meta = NULL;


