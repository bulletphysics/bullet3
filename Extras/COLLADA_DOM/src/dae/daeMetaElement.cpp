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
#include <dae/domAny.h>

daeMetaElementRefArray daeMetaElement::_metas;

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
}

daeElementRef
daeMetaElement::create(daeString s)
{
	daeMetaElement* me = findChild(s);
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

daeMetaElement *
daeMetaElement::getChildMetaElement(daeString s)
{
	int n = (int)_metaElements.getCount();
	int i;
	for(i=0;i<n;i++) {
		if (strcmp(_metaElements[i]->_elementType->_name,s)==0)
			return _metaElements[i]->_elementType;
	}
	return NULL;
}

daeMetaElementAttribute *
daeMetaElement::getChildMetaElementAttribute(daeString s)
{
	int n = (int)_metaElements.getCount();
	int i;
	for(i=0;i<n;i++) {
		if (strcmp(_metaElements[i]->_elementType->_name,s)==0)
			return _metaElements[i];
	}
	return NULL;
}

#define defMAEA(class,maename) \
{ \
defMetaAttributeElement* maea = new daeMetaAttributeArrayElement; \
maea->

#define defME(class, name) \
	daeMetaElement* parent = active; \
	daeMetaElement* active = new daeMetaElement; \
	active->_name = "##name##"; \
	active->_elementSize = sizeof( class ); \
	if (parent != NULL) \
	    parent->appendElement(active, daeOffsetOf( parent, name ));


#define defMA(class,matype,maname) \
{ \
daeMetaAttribute* ma = new daeMetaAttribute; \
ma->_name = "##maname##";\
ma->_type = daeAtomicType::get("##matype##");\
ma->_offset = daeOffsetOf( class , _##maname );\
ma->_container = active; \
active->appendAttribute(ma); \
}

daeMetaElement* daeMetaElement::_Schema = NULL;
 
void
daeMetaElement::initializeSchemaMeta()
{
}

daeMetaElement::daeMetaElement()
{
	_name = "noname";
	_createFunc = NULL;
	_minOccurs = 1;
	_maxOccurs = 1;
	_ref = "none";
	_isSequence = false;
	_isChoice = false;
	_needsResolve = false;
	_elementSize = sizeof(daeElement);
	_metaValue = NULL;
	_metaContents = NULL;
	_metaIntegration = NULL;
	_metaID = NULL;
	_parent = NULL;
	_staticPointerAddress = NULL;
	_isTrackableForQueries = true;
	_usesStringContents = false;
	_isTransparent = false;
	_isAbstract = false;
	_allowsAny = false;
	_metas.append(this);
}

daeMetaElement::~daeMetaElement()
{
	if (_metaContents)
		delete _metaContents;
	if (_staticPointerAddress != NULL)
		*_staticPointerAddress = NULL;
}

void
daeMetaElement::addContents(daeInt offset)
{
	daeMetaElementArrayAttribute* meaa = new daeMetaElementArrayAttribute;
	meaa->setType(daeAtomicType::get("element"));
	meaa->setName("contents");
	meaa->setOffset(offset);
	meaa->setContainer( this);
	meaa->setElementType( daeElement::getMeta() );
	_metaContents = meaa;
}


void
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
	element->_parent = this;
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
	element->_parent = this;
}

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
		_createFunc = DAECreateElement;
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
	_metas.clear();
}

void daeMetaElement::appendPossibleChild( daeString name, daeMetaElementAttribute* cont, daeString type ) {
	_otherChildren.append( name );
	_otherChildrenContainer.append( cont );
	if ( type ) _otherChildrenTypes.append( type );
	else _otherChildrenTypes.append( "" );
}

