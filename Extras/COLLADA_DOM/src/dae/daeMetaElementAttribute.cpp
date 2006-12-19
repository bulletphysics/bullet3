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

#include <dae/daeMetaElementAttribute.h>
#include <dae/daeMetaElement.h>

daeMetaElementAttribute::daeMetaElementAttribute( daeMetaElement *container, daeMetaCMPolicy *parent, daeUInt ordinal,
												 daeInt minO, daeInt maxO) : daeMetaCMPolicy( container, parent, ordinal, minO, maxO )
{
	_elementType = NULL;
}

daeMetaElementAttribute::~daeMetaElementAttribute()
{}

daeMetaElementArrayAttribute::daeMetaElementArrayAttribute( daeMetaElement *container, daeMetaCMPolicy *parent, daeUInt ordinal,
												 daeInt minO, daeInt maxO) : daeMetaElementAttribute( container, parent, ordinal, minO, maxO )
{
}

daeMetaElementArrayAttribute::~daeMetaElementArrayAttribute()
{}


void daeMetaElementAttribute::set(daeElement* e, daeString s)
{
	//_type->stringToMemory((char*)s, getWritableMemory(e));
	daeElementRef *ref = (daeElementRef*)(getWritableMemory(e));
	if ((*ref) == NULL) {
		(*ref) = _elementType->create();
	}
	(*ref)->getMeta()->getValueAttribute()->set((*ref), s);
}

void daeMetaElementAttribute::copy(daeElement* to, daeElement *from) {
	daeElement *cpy = (*(daeElementRef*)(getWritableMemory(from)))->clone();
	(*(daeElementRef*)(getWritableMemory(to))) = cpy;
}

void daeMetaElementArrayAttribute::copy(daeElement* to, daeElement *from) {
	(void)to;
	(void)from;
}

void
daeMetaElementAttribute::setDocument( daeElement * parent, daeDocument* c )
{
	daeElementRef* er = (daeElementRef*)getWritableMemory( parent );
	if ( ((daeElement*)(*er)) != NULL ) {
		(*er)->setDocument( c );
	}
}

void
daeMetaElementArrayAttribute::setDocument( daeElement * parent, daeDocument* c )
{
	daeElementRefArray* era = (daeElementRefArray*)getWritableMemory( parent );
	for ( unsigned int i = 0; i < era->getCount(); i++ ) {
		era->get(i)->setDocument( c );
	}
}

daeInt
daeMetaElementAttribute::getCount(daeElement* e)
{
	if (e == NULL)
		return 0;
	return ((*((daeElementRef*)getWritableMemory(e))) != NULL);
}

daeMemoryRef
daeMetaElementAttribute::get(daeElement *e, daeInt index)
{
	(void)index; 
	return getWritableMemory(e);
}

daeInt
daeMetaElementArrayAttribute::getCount(daeElement *e)
{
	if (e == NULL)
		return 0;
	daeElementRefArray* era = (daeElementRefArray*)getWritableMemory(e);
	if (era == NULL)
		return 0;
	return (daeInt)era->getCount();
}

daeMemoryRef
daeMetaElementArrayAttribute::get(daeElement* e, daeInt index)
{
	if (e == NULL)
		return NULL;
	daeElementRefArray* era = (daeElementRefArray*)getWritableMemory(e);
	if (era == NULL || index >= (daeInt)era->getCount() )
		return NULL;
	return (daeMemoryRef)&(era->get(index));
}

daeElement *
daeMetaElementAttribute::placeElement(daeElement* parent, daeElement* child, daeUInt &ordinal, daeInt offset, daeElement* before, daeElement *after )
{
	(void)offset;
	(void)before;
	(void)after;
	if ((parent == NULL)||(child == NULL))
		return NULL;
	if ( child->getMeta() != _elementType || ( child->getElementName() != NULL && strcmp( child->getElementName(), _name ) != 0 )  ) {
		return NULL;
	}
	if (child->getParentElement() == parent) {
		//I Don't know why this gets called when the child already has this as parent.
		return child;
	}
	daeElementRef* er = (daeElementRef*)getWritableMemory(parent);
	
	daeElement::removeFromParent( child );
	child->setParentElement( parent );

	*er = child;
	ordinal = _ordinalOffset;

	return child;
}

daeElement *
daeMetaElementArrayAttribute::placeElement(daeElement* parent, daeElement* child, daeUInt &ordinal, daeInt offset, daeElement* before, daeElement *after )
{
	if ((parent == NULL)||(child == NULL))
		return NULL;
	if ( child->getMeta() != _elementType || ( child->getElementName() != NULL && strcmp( child->getElementName(), _name ) != 0 )  ) {
		return NULL;
	}
	daeElement *p = child->getParentElement();
	daeElementRefArray* era = (daeElementRefArray*)getWritableMemory(parent);
	if ( _maxOccurs != -1 && (daeInt)era->getCount()-offset >= _maxOccurs ) {
		return NULL;
	}
	removeElement( p, child );
	child->setParentElement( parent );

	if ( before != NULL && before->getMeta() == _elementType ) {
		size_t idx(0);
		if ( era->find( before, idx ) == DAE_OK ) {
			era->insertAt( idx, child );
		}
	}
	else if ( after != NULL && after->getMeta() == _elementType ) {
		size_t idx(0);
		if ( era->find( after, idx ) == DAE_OK ) {
			era->insertAt( idx+1, child );
		}
	}
	else {
		era->append(child);
	}
	ordinal = _ordinalOffset;

	return child;
}

// These are the opposite of the placeElement functions above
daeBool
daeMetaElementAttribute::removeElement(daeElement* parent, daeElement* child)
{
	(void)child; // silence unused variable warning

	if ((parent == NULL)||(child == NULL ))
		return false;

	daeElementRef* er = (daeElementRef*)getWritableMemory(parent);
	if ( *er != child )  {
		return false;
	}
	*er = NULL;
	return true;
}

daeBool
daeMetaElementArrayAttribute::removeElement(daeElement* parent,
										   daeElement* child)
{
	if ((parent == NULL)||(child == NULL))
		return false ;
	
	daeElementRefArray* era = (daeElementRefArray*)getWritableMemory(parent);
/*	if ( (daeInt)era->getCount() <= _minOccurs ) {
		return false;
	}*/
	daeInt error = era->remove(child);
	if ( error != DAE_OK ) {
		return false;
	}
	return true;
}

daeMetaElement *daeMetaElementAttribute::findChild( daeString elementName ) {
	if ( strcmp( elementName, _name ) == 0 ) {
		return _elementType;
	}
	return NULL;
}

void daeMetaElementAttribute::getChildren( daeElement *parent, daeElementRefArray &array ) {
	daeElementRef* er = (daeElementRef*)getWritableMemory(parent);
	if ( *er != NULL ) {
		array.append( *er );
	}
}

void daeMetaElementArrayAttribute::getChildren( daeElement *parent, daeElementRefArray &array ) {
	daeElementRefArray* era = (daeElementRefArray*)getWritableMemory(parent);
	size_t cnt = era->getCount();
	for ( size_t x = 0; x < cnt; x++ ) {
		array.append( era->get(x) );
	}
}
