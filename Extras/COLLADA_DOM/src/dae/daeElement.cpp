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

#include <dae/daeElement.h>
#include <dae/daeArray.h>
#include <dae/daeMetaAttribute.h>
#include <dae/daeMetaElement.h>
#include <dae/daeDatabase.h>
#include <dae/daeErrorHandler.h>

#include <dae/daeIntegrationObject.h>
#include <dae/daeURI.h>

void
daeElement::release() const
{
	if (--_refCount <= 0)
		delete this;
}
daeElementRef DAECreateElement(int nbytes)
{
	return new(nbytes) daeElement;
}

static daeElementRefArray resolveArray;
//static char StaticIndentBuf[] = "";

daeIntegrationObject*
daeElement::getIntObject( IntegrationState from_state, IntegrationState to_state )
{
	if ( !_intObject ) {
		return NULL;
	}
	if ( from_state >= int_created ) {
		if ( _intObject->_from_state < int_created ) {
			daeErrorHandler::get()->handleWarning("Warning: getIntObject tries to get object that is not created (from)");
			return NULL;
		}
		if ( from_state >= int_converted ) {
			_intObject->fromCOLLADAChecked();
			if ( from_state == int_finished ) {
				_intObject->fromCOLLADAPostProcessChecked();
			}
		}
	}
	if ( to_state >= int_created ) {
		if ( _intObject->_to_state < int_created ) {
			daeErrorHandler::get()->handleWarning("Warning: getIntObject tries to get object that is not created (to)");
			return NULL;
		}
		if ( to_state >= int_converted ) {
			_intObject->toCOLLADAChecked();
			if ( to_state == int_finished ) {
				_intObject->toCOLLADAPostProcessChecked();
			}
		}
	}
	return _intObject;
}

daeElementRef
daeElement::createElement(daeString className)
{
	daeElementRef elem = _meta->create(className);
	// Bug #225 work around
//	if ( elem != NULL)
//		elem->ref(); // change premature delete into memory leak.
	return elem;
}

daeElement*
daeElement::createAndPlace(daeString className)
{
	daeElementRef elem = _meta->create(className);
	daeBool place = false;
	if (elem != NULL)
		place = placeElement(elem);
	if (place)
		return elem;
	return NULL;
}

daeElement*
daeElement::createAndPlaceAt(daeInt index, daeString className)
{
	daeElementRef elem = _meta->create(className);
	daeBool place = false;
	if (elem != NULL)
		place = placeElementAt(index, elem);
	if (place)
		return elem;
	return NULL;
}

daeBool
daeElement::placeElement(daeElement* e)
{
	if (e == NULL || e == this)
		return false;
	if (e->getMeta()->getIsAbstract()) {
		return false;
	}
	daeMetaElementAttributeArray &meas = _meta->getMetaElements();
	if (_meta->getAllowsAny()) {
		//remove element from praent
		if ( e->_parent ) {
			e->_parent->removeChildElement( e );
		}
		e->_parent = this;
		//add element to contents if one exists
		if (_meta->getContents() != NULL) {
			daeElementRefArray* contents =
				(daeElementRefArray*)_meta->getContents()->getWritableMemory(this);
			contents->append(e);
		}	
		//update document pointer
		e->setDocument( _document );
		if ( _document ) {
			_document->insertElement( e );
			_document->setModified(true);
		}
		return true;	
	}
	int cnt = (int)meas.getCount();
	int i;
	daeString nm = e->getElementName();
	if ( !nm ) {
		nm = e->getTypeName();
	}
	for(i=0;i<cnt;i++) {
		if (strcmp(meas[i]->getName(), nm ) == 0 || strcmp(meas[i]->_elementType->getName(), nm ) == 0) {
			//add element to meta
			meas[i]->placeElement(this,e);
			//remove element from praent
			if ( e->_parent ) {
				e->_parent->removeChildElement( e );
			}
			e->_parent = this;
			//add element to contents if one exists
			if (_meta->getContents() != NULL) {
				daeElementRefArray* contents =
					(daeElementRefArray*)_meta->getContents()->getWritableMemory(this);
				contents->append(e);
			}	
			//update document pointer
			e->setDocument( _document );
			if ( _document ) {
				_document->insertElement( e );
				_document->setModified(true);
			}
			return true;
		}
	}
	for ( unsigned int c = 0; c < _meta->getPossibleChildrenCount(); c++ ) {
		if (strcmp(_meta->getPossibleChildName(c), e->_meta->getName()) == 0 ||
			strcmp(_meta->getPossibleChildType(c), e->_meta->getName()) == 0 ) {
			
			daeMetaElementAttribute *cont = _meta->getPossibleChildContainer(c);
			int elCnt = cont->getCount(this);
			daeMemoryRef mem = cont->get(this, elCnt );
			daeElementRef el;
			if ( mem != 0 ) {
				el = *(daeElementRef*)mem;
			}
			if ( el == NULL ) {
				cont->placeElement(this, cont->_elementType->create() );
				el = *(daeElementRef*)cont->get(this, elCnt );
				el->_parent = this;
			}
			if ( el->placeElement( e ) ) {
				if (_meta->getContents() != NULL) {
					daeElementRefArray* contents =
						(daeElementRefArray*)_meta->getContents()->getWritableMemory(this);
					contents->append(el);
				}
				return true;
			}
			return false;
		}
	}
	return false;
}

daeBool daeElement::placeElementAt(daeInt index, daeElement* e) {
	if (e == NULL || e == this)
		return false;
	if (e->getMeta()->getIsAbstract()) {
		return false;
	}
		
	daeMetaElementAttributeArray &meas = _meta->getMetaElements();
	if (_meta->getAllowsAny()) {
		//remove element from praent
		if ( e->_parent ) {
			e->_parent->removeChildElement( e );
		}
		e->_parent = this;
		//add element to contents if one exists
		if (_meta->getContents() != NULL) {
			daeElementRefArray* contents =
				(daeElementRefArray*)_meta->getContents()->getWritableMemory(this);
			contents->insertAt(index, e);
		}	
		//update document pointer
		e->setDocument( _document );
		if ( _document ) {
			_document->insertElement( e );
			_document->setModified(true);
		}
		return true;	
	}
	int cnt = (int)meas.getCount();
	int i;
	daeString nm = e->getElementName();
	if ( !nm ) {
		nm = e->getTypeName();
	}
	for(i=0;i<cnt;i++) {
		if (strcmp(meas[i]->getName(), nm ) == 0 || strcmp(meas[i]->_elementType->getName(), nm ) == 0) {
			//add element to meta
			meas[i]->placeElement(this,e);
			//remove element from praent
			if ( e->_parent ) {
				e->_parent->removeChildElement( e );
			}
			e->_parent = this;
			//add element to contents if one exists
			if (_meta->getContents() != NULL) {
				daeElementRefArray* contents =
					(daeElementRefArray*)_meta->getContents()->getWritableMemory(this);
				contents->insertAt( index, e );
			}	
			//update document pointer
			e->setDocument( _document );
			if ( _document ) {
				_document->insertElement( e );
				_document->setModified(true);
			}
			return true;
		}
	}
	for ( unsigned int c = 0; c < _meta->getPossibleChildrenCount(); c++ ) {
		if (strcmp(_meta->getPossibleChildName(c), e->_meta->getName()) == 0 ||
			strcmp(_meta->getPossibleChildType(c), e->_meta->getName()) == 0 ) {
			
			daeMetaElementAttribute *cont = _meta->getPossibleChildContainer(c);
			int elCnt = cont->getCount(this);
			daeMemoryRef mem = cont->get(this, elCnt );
			daeElementRef el;
			if ( mem != 0 ) {
				el = *(daeElementRef*)mem;
			}
			if ( el == NULL ) {
				cont->placeElement(this, cont->_elementType->create() );
				el = *(daeElementRef*)cont->get(this, elCnt );
				el->_parent = this;
			}
			if ( el->placeElement( e ) ) {
				if (_meta->getContents() != NULL) {
					daeElementRefArray* contents =
						(daeElementRefArray*)_meta->getContents()->getWritableMemory(this);
					contents->insertAt(index, el);
				}
				return true;
			}
			return false;
		}
	}
	return false;
}

daeBool daeElement::placeElementBefore( daeElement *marker, daeElement *element ) {
	if (marker == NULL || element == NULL || marker->getXMLParentElement() != this ) {
		return false;
	}
	if ( _meta->getContents() != NULL ) {
		size_t idx;
		daeElementRefArray* contents =
						(daeElementRefArray*)_meta->getContents()->getWritableMemory(this);
		if ( contents->find( marker, idx ) != DAE_OK ) {
			return false;
		}
		return placeElementAt( (daeInt)idx, element );
	}
	if ( strcmp( marker->getTypeName(), element->getTypeName() ) == 0 ) {
		//same type
		daeMetaElementAttribute *mea = _meta->getChildMetaElementAttribute( element->getTypeName() );
		daeElementRefArray* era = (daeElementRefArray*)mea->getWritableMemory(this);
		size_t idx;
		if ( era->find( marker, idx ) != DAE_OK ) {
			return false;
		}
		era->insertAt( idx, element );
		return true;
	}
	return placeElement( element );
}

daeBool daeElement::placeElementAfter( daeElement *marker, daeElement *element ) {
	if (marker == NULL || element == NULL || marker->getXMLParentElement() != this ) {
		return false;
	}
	if ( _meta->getContents() != NULL ) {
		size_t idx;
		daeElementRefArray* contents =
						(daeElementRefArray*)_meta->getContents()->getWritableMemory(this);
		if ( contents->find( marker, idx ) != DAE_OK ) {
			return false;
		}
		return placeElementAt( (daeInt)idx+1, element );
	}
	if ( strcmp( marker->getTypeName(), element->getTypeName() ) == 0 ) {
		daeMetaElementAttribute *mea = _meta->getChildMetaElementAttribute( element->getTypeName() );
		daeElementRefArray* era = (daeElementRefArray*)mea->getWritableMemory(this);
		size_t idx;
		if ( era->find( marker, idx ) != DAE_OK ) {
			return false;
		}
		era->insertAt( idx+1, element );
		return true;
	}
	return placeElement( element );
}

daeInt daeElement::findLastIndexOf( daeString elementName ) {
	if ( _meta->getContents() != NULL ) {
		daeElementRefArray* contents =
						(daeElementRefArray*)_meta->getContents()->getWritableMemory(this);
		for ( int i = (int)contents->getCount()-1; i >= 0; --i ) {
			daeString nm = contents->get(i)->getElementName();
			if ( nm == NULL ) {
				nm = contents->get(i)->getTypeName();
			}
			if ( strcmp( nm, elementName ) == 0 ) {
				return i;
			}
		}
		return -1;
	}
	daeInt idx = 0;
	size_t cnt = _meta->getMetaElements().getCount();
	for ( unsigned int i = 0; i < cnt; ++i ) {
		idx += (daeInt)((daeElementRefArray*)(_meta->getMetaElements().get(i)->getWritableMemory(this)))->getCount();
		if ( strcmp( _meta->getMetaElements().get(i)->getName(), elementName ) == 0 ) {
			return idx;
		}
	}
	return -1;
}

daeBool 
daeElement::removeChildElement(daeElement* element)
{
	// error traps
	if(element==NULL)
		return false;
	if(element->_parent != this)
		return false;

	// Clear the element's parent pointer, if the element has references outside
	// 'this' it won't go away, so we want to make sure it's parent is valid

	element->_parent = NULL;

	// Remove the element from our contents array (if we have one)

	if (_meta->getContents() != NULL) 
	{
		daeElementRefArray* contents = (daeElementRefArray*)_meta->getContents()->getWritableMemory(this);
		contents->remove(element);
	}

	// Remove the element from wherever else it appears in 'this'

	daeMetaElementAttributeArray &meas = _meta->getMetaElements();
	int cnt = (int)meas.getCount();
	int i;
	// Look for a meta element with a matching name
	for(i=0;i<cnt;i++) 
	{
		if (strcmp(meas[i]->getName(), element->_meta->getName()) == 0) 
		{
			if ( _document ) {
				_document->removeElement( element );
				_document->setModified(true);
			}
			// Remove it, if the element's ref count goes to zero it might be destructed,
			// so don't touch "element" again after this point.
			meas[i]->removeElement(this,element);
			return true;
		}
	}
	return false;
}

// !!!ACL Added to fix mantis issue 0000416
void daeElement::setDocument( daeDocument *c ) {
	if( _document == c ) {
		return;
	}
	_document = c;

	daeMetaElementAttributeArray &meas = _meta->getMetaElements();
	int cnt = (int)meas.getCount();
	for( int i = 0; i < cnt; i++) {
		meas[i]->setDocument( this, c );
	}

}

daeBool
daeElement::setAttribute(daeString attrName, daeString attrValue)
{
	if (_meta == NULL)
		return false;
	
	daeMetaAttributeRefArray& metaAttrs = _meta->getMetaAttributes();
	int n = (int)metaAttrs.getCount();
	int i;
	for(i=0;i<n;i++) {
		//fflush(stdout);
		if ((metaAttrs[i]->getName() != NULL) &&
			(strcmp(metaAttrs[i]->getName(),attrName)==0)) {
#if 0		//debug stuff
			printf("%s(%s)->setAttr(%s,%s)\n",
				   (const char*)_meta->getName(),
				   metaAttrs[i]->getType()->getTypeString(),
				   attrName,
				   attrValue);
#endif
			if (metaAttrs[i]->getType() != NULL)
			{
				metaAttrs[i]->set(this,attrValue);
			}
			return true;
		}
	}
	return false;
}

void
daeElement::appendResolveElement(daeElement* elem)
{
	resolveArray.append(elem);
}
void
daeElement::resolveAll()
{
	int cnt;
	while(resolveArray.getCount()) {
		cnt = (int)resolveArray.getCount();
		daeElementRef elem = resolveArray[cnt-1];
		resolveArray.removeIndex(cnt-1);
		elem->resolve();
	}
	/*size_t cnt = resolveArray.getCount();
	for ( size_t i =0; i < cnt; i++ ) {
		resolveArray[i]->resolve();
	}
	resolveArray.clear();*/
}

void
daeElement::resolve()
{
	if (_meta == NULL)
		return;
	
	daeMetaAttributeRefArray& maa = _meta->getMetaAttributes();
	int n = (int)maa.getCount();
	int i;
	for(i=0;i<n;i++)
		maa[i]->resolve(this);
}

void
daeElement::setup(daeMetaElement* meta)
{
	if (_meta)
		return;
	_meta = meta;
	if (meta->needsResolve())
		appendResolveElement((daeElement*)this);
	daeMetaElement* intlibMeta = meta->getMetaIntegration();
	if (intlibMeta != NULL)
	{
		 daeElementRef intObj = intlibMeta->create();
		 intObj->ref(); //inc the ref count
		_intObject = (daeIntegrationObject*)(daeElement*)intObj;
	}
	daeMetaAttributeRefArray& attrs = meta->getMetaAttributes();
	int macnt = (int)attrs.getCount();
	
	int i;
	for(i=0;i<macnt;i++)
		if (attrs[i]->getDefault() != NULL)
			attrs[i]->set(this, attrs[i]->getDefault());

#if 0	
	// Setup resolvers to know their containers and thus their file context
	daeMetaAttributePtrArray& resolvers = meta->getMetaResolvers();
	int racnt = resolvers.getCount();
	for(i=0;i<racnt;i++)
		((daeURI*)(resolvers[i]->getWritableMemory(this)))->_container =
			this;
#endif	
}

daeElement::daeElement():
		_refCount(0),
		_intObject(0),
		_parent(NULL),
		_document(NULL),
		_meta(NULL),
		_elementName(NULL)
{}

daeElement::~daeElement()
{
	if (_intObject)
		_intObject->release();

	if (_elementName) {
		delete[] _elementName;
		_elementName = NULL;
	}
}

//function used until we clarify what's a type and what's a name for an element
daeString daeElement::getTypeName() const
{
	return _meta->getName();
}
daeString daeElement::getElementName() const
{
	return _elementName;
}
void daeElement::setElementName( daeString nm ) {
	if ( nm == NULL ) {
		if ( _elementName ) delete[] _elementName;
		_elementName = NULL;
		return;
	}
	if ( !_elementName ) _elementName = new daeChar[128];
	strcpy( (char*)_elementName, nm );
}

daeString daeElement::getID() const
{
	if ((_meta == NULL) || (!_meta->getIDAttribute()))
		return NULL;
	else
		return *(daeStringRef*)_meta->getIDAttribute()->getWritableMemory(const_cast<daeElement*>(this));
}

void daeElement::getChildren( daeElementRefArray &array ) {
	if (_meta->getContents() != NULL) {
		daeMetaElementArrayAttribute *contents = _meta->getContents();
		for ( int i = 0; i < contents->getCount( this ); i++ ) {
			array.append( *(daeElementRef*)contents->get( this, i ) );
		}
	}
	else
	{
		daeMetaElementAttributeArray &meas = _meta->getMetaElements();
		for(unsigned int i = 0; i < meas.getCount(); i++) {
			for ( int c = 0; c < meas[i]->getCount( this ); c++ ) {
				array.append( *(daeElementRef*)meas[i]->get( this, c ) );
			}
		}			
	}
}

daeSmartRef<daeElement> daeElement::clone(daeString idSuffix, daeString nameSuffix) {
	//use the meta object system to create a new instance of this element
	daeElementRef ret = _meta->create();
	ret->setElementName( getElementName() );
	//use meta system to copy attributes
	daeMetaAttributeRefArray &attrs = _meta->getMetaAttributes();
	for ( unsigned int i = 0; i < attrs.getCount(); i++ ) {
		//memcpy( attrs[i]->getWritableMemory( ret ), attrs[i]->getWritableMemory( this ), attrs[i]->getSize() );
		attrs[i]->copy( ret, this );
	}
	if ( _meta->getValueAttribute() != NULL ) {
		daeMetaAttribute *val = _meta->getValueAttribute();
		//memcpy( val->getWritableMemory( ret ), val->getWritableMemory( this ), val->getSize() );
		val->copy( ret, this );
	}
	//use meta system to child elements
	if ( _meta->getContents() != NULL )  {
		daeMetaElementArrayAttribute *contents = _meta->getContents();
		for ( int i = 0; i < contents->getCount( this ); i++ ) {
			ret->placeElement( (*(daeElementRef*)contents->get( this, i ))->clone(idSuffix, nameSuffix) );
		}
	}
	else {
		daeMetaElementAttributeArray &meas = _meta->getMetaElements();
		for(unsigned int i = 0; i < meas.getCount(); i++) {
			for ( int c = 0; c < meas[i]->getCount( this ); c++ ) {
				ret->placeElement( (*(daeElementRef*)meas[i]->get( this, c ))->clone(idSuffix, nameSuffix) );
			}
		}	
	}
	//ret->ref(); //called because the cast to daeElement* releases a reference causing premature deletion
	//mangle the id
	daeMetaAttribute *id = _meta->getIDAttribute();
	if ( idSuffix != NULL && id != NULL ) {
		daeChar str[2048];
		id->getType()->memoryToString( id->getWritableMemory( ret ), str, 2048 );
		if ( strcmp( str, "" ) ) {
			strcat( str, idSuffix );
		}
		id->getType()->stringToMemory( str, id->getWritableMemory( ret ) );
	}
	//mangle the name
	daeMetaAttribute *nm = _meta->getMetaAttribute("name");
	if ( nameSuffix != NULL && nm != NULL ) {
		daeChar str[2048];
		nm->getType()->memoryToString( nm->getWritableMemory( ret ), str, 2048 );
		if ( strcmp( str, "" ) ) {
			strcat( str, nameSuffix );
		}
		nm->getType()->stringToMemory( str, nm->getWritableMemory( ret ) );
	}
	//ret->_intObject = _intObject;
	return ret;
}

daeURI *daeElement::getDocumentURI() const {
	if ( _document == NULL ) {
		return NULL;
	}
	return _document->getDocumentURI();
}
