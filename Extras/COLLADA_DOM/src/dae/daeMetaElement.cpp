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

#include <dae/daeMetaElement.h>
#include <dae/daeElement.h>
#include <dae/daeDocument.h>
#include <dae/domAny.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaElementAttribute.h>

static daeMetaElementRefArray *mera = NULL;
static daeTArray< daeMetaElement** > *mes = NULL;

daeElementRef
daeMetaElement::create() 
{
#if defined(_DEBUG) && defined(WIN32)
	if (_createFunc == NULL)
		return NULL;
#endif
	daeElementRef ret =  (*_createFunc)(_elementSize);
	ret->setup(this);
		
	return ret;
}

daeElementRef
daeMetaElement::create(daeString s)
{
	daeMetaElement* me = NULL;
	if ( strcmp( s, _name ) == 0 ) {
		//looking for this meta
		me = this;
	}
	else {
		me = _contentModel->findChild(s);
	}
	if (me != NULL) {
		daeElementRef ret = me->create();
		if ( strcmp(s, me->getName() ) != 0 ) {
			ret->setElementName(s);
		}
		return ret;
	}
	if ( getAllowsAny() ) {
		daeElementRef ret = domAny::registerElement()->create();
		ret->setElementName(s);
		return ret;
	}
	return NULL;
}

/*
daeMetaElement*
daeMetaElement::findChild(daeString s)
{
	if (s != NULL) {
		if ( strcmp( _name, s) == 0 ) {
			return this;
		}
		int n = (int)_metaElements.getCount();
		int i;
		for(i=0;i<n;i++) {
			daeMetaElement* me = _metaElements[i]->_elementType;
		
			if ((me == NULL) || ((daeString)(me->_name) == NULL))
				continue;
		
			if (strcmp(me->_name,s)==0)
				return me;

			if (strcmp(_metaElements[i]->getName(),s)==0)
				return me;
		}
		//!!!ACL Added for testing complex types and groups
		for( i =0; i < (int)_otherChildren.getCount(); i++ ) {
			if ( strcmp( _otherChildren[i], s) == 0 ) {
				daeMetaElementAttribute *mea = _otherChildrenContainer[i];
				daeMetaElement *me = mea->getElementType();
				return me->findChild(s);
			}
		}
	}
	return NULL;
}*/

daeMetaElement::daeMetaElement()
{
	_name = "noname";
	_createFunc = NULL;
	_needsResolve = false;
	_elementSize = sizeof(daeElement);
	_metaValue = NULL;
	_metaContents = NULL;
    _metaContentsOrder = NULL; // sthomas
	_metaIntegration = NULL;
	_metaID = NULL;
	_isTrackableForQueries = true;
	_usesStringContents = false;
	_isTransparent = false;
	_isAbstract = false;
	_allowsAny = false;
	_innerClass = false;
	_metas().append(this);

	_contentModel = NULL;
}

daeMetaElement::~daeMetaElement()
{
	if (_metaContents)
		delete _metaContents;
    if (_contentModel) // sthomas
        delete _contentModel;
    if (_metaContentsOrder) // sthomas
        delete _metaContentsOrder;
}

void daeMetaElement::setCMRoot( daeMetaCMPolicy *cm )
{
    if (_contentModel) 
        delete _contentModel;
    _contentModel = cm;
}

void
daeMetaElement::addContents(daeInt offset)
{
	daeMetaElementArrayAttribute* meaa = new daeMetaElementArrayAttribute( this, NULL, 0, 1, -1 );
	meaa->setType(daeAtomicType::get("element"));
	meaa->setName("contents");
	meaa->setOffset(offset);
	meaa->setContainer( this);
	meaa->setElementType( daeElement::getMeta() );
	_metaContents = meaa;
}
void
daeMetaElement::addContentsOrder(daeInt offset)
{
	daeMetaArrayAttribute* meaa = new daeMetaArrayAttribute();
	meaa->setType(daeAtomicType::get("uint"));
	meaa->setName("contentsOrder");
	meaa->setOffset(offset);
	meaa->setContainer( this);

    if (_metaContentsOrder)
        delete _metaContentsOrder;

	_metaContentsOrder = meaa;
}


/*void
daeMetaElement::appendArrayElement(daeMetaElement* element, daeInt offset, daeString name)
{
	daeMetaElementArrayAttribute* meaa = new daeMetaElementArrayAttribute;
	meaa->setType(daeAtomicType::get("element"));
	if ( name ) {
		meaa->setName(name);
	}
	else {
		meaa->setName(element->getName());
	}
	meaa->setOffset(offset);
	meaa->setContainer(this);
	meaa->setElementType( element);
	_metaElements.append(meaa);
}
void
daeMetaElement::appendElement(daeMetaElement* element, daeInt offset, daeString name)
{
	daeMetaElementAttribute* meaa = new daeMetaElementAttribute;
	meaa->setType(daeAtomicType::get("element"));
	if ( name ) {
		meaa->setName(name);
	}
	else {
		meaa->setName(element->getName());
	}
	meaa->setOffset( offset);
	meaa->setContainer( this );
	meaa->setElementType( element );
	_metaElements.append(meaa);
}*/

void
daeMetaElement::appendAttribute(daeMetaAttribute* attr)
{
	if (attr == NULL)
		return;

	if (strcmp(attr->getName(),"_value") == 0) {
		_usesStringContents = attr->getType()->getUsesStrings();

		_metaValue = attr;
	}
	else
		_metaAttributes.append(attr);

	if ((attr->getType() != NULL) &&
		((strcmp(attr->getType()->getTypeString(),"resolver")==0)||
		 (strcmp(attr->getType()->getTypeString(),"idref_resolver")==0))) {
		_resolvers.append(attr);
		_needsResolve = true;
	}
	
	if ((attr->getName() != NULL) &&
		(strcmp(attr->getName(),"id") == 0)) {
		_metaID = attr;
		_isTrackableForQueries = true;
	}
}

void
daeMetaElement::validate()
{
	if (_createFunc == NULL)
		_createFunc = (daeElementConstructFunctionPtr) DAECreateElement;
	if (_elementSize == 0)
	{
		daeInt place=0;
		unsigned int i;
		for(i=0;i<_metaAttributes.getCount();i++) {
			place += _metaAttributes[i]->getSize();
			int align = _metaAttributes[i]->getAlignment();
			place += align;
			place &= (~(align-1));
		}
		_elementSize = place;
	}
}
	
daeMetaAttribute*
daeMetaElement::getMetaAttribute(daeString s)
{
	int cnt = (int)_metaAttributes.getCount();
	int i;
	for(i=0;i<cnt;i++)
		if (strcmp(_metaAttributes[i]->getName(),s) == 0)
			return _metaAttributes[i];
	return NULL;
}


void daeMetaElement::releaseMetas()
{
	_metas().clear();
	size_t count = _classMetaPointers().getCount();
	for ( size_t i = 0; i < count; i++ )
	{
		*(_classMetaPointers()[i]) = NULL;
	}
	_classMetaPointers().clear();
	if (mera)
	{
		delete mera;
		mera = NULL;
	}
	if (mes)
	{
		delete mes;
		mes = NULL;
	}
}

daeBool daeMetaElement::place(daeElement *parent, daeElement *child, daeUInt *ordinal )
{
	if (child->getMeta()->getIsAbstract() || parent->getMeta() != this ) {
		return false;
	}
	daeUInt ord;
	daeElement *retVal = _contentModel->placeElement( parent, child, ord );
	if ( retVal != NULL ) {
		//update document pointer
		child->setDocument( parent->getDocument() );
		if ( parent->getDocument() ) {
			parent->getDocument()->insertElement( retVal );
			parent->getDocument()->setModified(true);
		}
		//add to _contents array
		if (_metaContents != NULL) {
			daeElementRefArray* contents =
				(daeElementRefArray*)_metaContents->getWritableMemory(parent);
			daeUIntArray* contentsOrder =
				(daeUIntArray*)_metaContentsOrder->getWritableMemory(parent);
			daeBool needsAppend = true;
			size_t cnt = contentsOrder->getCount();
			for ( size_t x = 0; x < cnt; x++ ) {
				if ( contentsOrder->get(x) > ord ) {
					contents->insertAt( x, retVal );
					contentsOrder->insertAt( x, ord );
					needsAppend = false;
					break;
				}
			}
			if ( needsAppend ) {
				contents->append(retVal);
				contentsOrder->append( ord );
			}
		}
		if ( ordinal != NULL ) {
			*ordinal = ord;
		}
	}
	return retVal!=NULL;
}

daeBool daeMetaElement::placeAt( daeInt index, daeElement *parent, daeElement *child )
{
	if (child->getMeta()->getIsAbstract() || parent->getMeta() != this || index < 0 ) {
		return false;
	}
	daeUInt ord;
	daeElement *retVal = _contentModel->placeElement( parent, child, ord );
	if ( retVal != NULL ) {
		//add to _contents array
		if (_metaContents != NULL) {
			daeElementRefArray* contents =
				(daeElementRefArray*)_metaContents->getWritableMemory(parent);
			daeUIntArray* contentsOrder =
				(daeUIntArray*)_metaContentsOrder->getWritableMemory(parent);
			daeBool validLoc;
			if ( index > 0 ) {
				validLoc = contentsOrder->get(index) >= ord && contentsOrder->get(index) <= ord;
			}
			else {
				if ( contentsOrder->getCount() == 0 ) {
					validLoc = true;
				}
				else {
					validLoc = contentsOrder->get(index) >= ord;
				}
			}
			if ( validLoc ) {
				contents->insertAt( index, retVal );
				contentsOrder->insertAt( index, ord );
			}
			else {
				_contentModel->removeElement( parent, retVal );
				retVal = NULL;
			}
		}
	}
	if ( retVal != NULL ) {
		//update document pointer
		child->setDocument( parent->getDocument() );
		if ( parent->getDocument() ) {
			parent->getDocument()->insertElement( retVal );
			parent->getDocument()->setModified(true);
		}
	}
	return retVal!=NULL;
}

daeBool daeMetaElement::placeBefore( daeElement *marker, daeElement *parent, daeElement *child, daeUInt *ordinal )
{
	if (child->getMeta()->getIsAbstract() || parent->getMeta() != this ) {
		return false;
	}
	daeUInt ord;
	daeElement *retVal = _contentModel->placeElement( parent, child, ord, 0, marker, NULL );
	if ( retVal != NULL ) {
		//add to _contents array
		if (_metaContents != NULL) {
			daeElementRefArray* contents =
				(daeElementRefArray*)_metaContents->getWritableMemory(parent);
			daeUIntArray* contentsOrder =
				(daeUIntArray*)_metaContentsOrder->getWritableMemory(parent);
			size_t index(0);
			daeBool validLoc = false;
			if ( contents->find( marker, index ) == DAE_OK ) {
				if ( index > 0 ) {
					daeUInt gt = contentsOrder->get(index-1);
					daeUInt lt = contentsOrder->get(index);
					validLoc = gt <= ord && lt >= ord;
				}
				else {
					validLoc = contentsOrder->get(index) >= ord;
				}
			}
			if ( validLoc ) {
				contents->insertAt( index, retVal );
				contentsOrder->insertAt( index, ord );
				if ( ordinal != NULL ) {
					*ordinal = ord;
				}
			}
			else {
				_contentModel->removeElement( parent, retVal );
				retVal = NULL;
			}
		}
	}
	if ( retVal != NULL ) {
		//update document pointer
		child->setDocument( parent->getDocument() );
		if ( parent->getDocument() ) {
			parent->getDocument()->insertElement( retVal );
			parent->getDocument()->setModified(true);
		}
	}
	return retVal!=NULL;
}

daeBool daeMetaElement::placeAfter( daeElement *marker, daeElement *parent, daeElement *child, daeUInt *ordinal )
{
	if (child->getMeta()->getIsAbstract() || parent->getMeta() != this ) {
		return false;
	}
	daeUInt ord;
	daeElement *retVal = _contentModel->placeElement( parent, child, ord, 0, marker, NULL );
	if ( retVal != NULL ) {
		//add to _contents array
		if (_metaContents != NULL) {
			daeElementRefArray* contents =
				(daeElementRefArray*)_metaContents->getWritableMemory(parent);
			daeUIntArray* contentsOrder =
				(daeUIntArray*)_metaContentsOrder->getWritableMemory(parent);
			size_t index(0);
			daeBool validLoc = false;
			if ( contents->find( marker, index ) == DAE_OK ) {
				if ( index < contentsOrder->getCount()-1 ) {
					validLoc = contentsOrder->get(index) <= ord && contentsOrder->get(index+1) >= ord;
				}
				else {
					validLoc = contentsOrder->get(index) <= ord;
				}
			}
			if ( validLoc ) {
				contents->insertAt( index+1, retVal );
				contentsOrder->insertAt( index+1, ord );
				if ( ordinal != NULL ) {
					*ordinal = ord;
				}
			}
			else {
				_contentModel->removeElement( parent, retVal );
				retVal = NULL;
			}
		}
	}
	if ( retVal != NULL ) {
		//update document pointer
		child->setDocument( parent->getDocument() );
		if ( parent->getDocument() ) {
			parent->getDocument()->insertElement( retVal );
			parent->getDocument()->setModified(true);
		}
	}
	return retVal!=NULL;
}

daeBool daeMetaElement::remove(daeElement *parent, daeElement *child)
{
	if ( parent->getMeta() != this ) {
		return false;
	}
	//prevent child from being deleted
	daeElementRef el( child );
	if ( _contentModel->removeElement( parent, child ) ) {
		if ( _metaContents != NULL) 
		{
			daeElementRefArray* contents = (daeElementRefArray*)_metaContents->getWritableMemory(parent);
			daeUIntArray* contentsOrder = (daeUIntArray*)_metaContentsOrder->getWritableMemory(parent);
			size_t idx(0);
			if ( contents->remove(child, &idx) == DAE_OK ) {
				contentsOrder->removeIndex( idx );
			}
		}
		if ( child->getDocument() ) {
			child->getDocument()->removeElement( child );
			child->getDocument()->setModified(true);
		}
		return true;
	}
	return false;
}

void daeMetaElement::getChildren( daeElement* parent, daeElementRefArray &array )
{
	if ( parent->getMeta() != this ) {
		return;
	}
	if ( _metaContents != NULL ) {
		daeElementRefArray* contents = (daeElementRefArray*)_metaContents->getWritableMemory(parent);
		for ( size_t x = 0; x < contents->getCount(); x++ ) {
			array.append( contents->get(x) );
		}
	}
	else if ( _contentModel != NULL ) {
		_contentModel->getChildren( parent, array );
	}
}

daeMetaElementRefArray &daeMetaElement::_metas()
{
	if (!mera)
	{
		mera = new daeMetaElementRefArray();
	}
	return *mera;
}

daeTArray< daeMetaElement** > &daeMetaElement::_classMetaPointers()
{
	if (!mes)
	{
		mes = new daeTArray< daeMetaElement** >();
	}
	return *mes;
}

