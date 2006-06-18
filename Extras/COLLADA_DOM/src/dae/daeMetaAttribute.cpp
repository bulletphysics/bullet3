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

#include <dae/daeMetaAttribute.h>
#include <dae/daeMetaElement.h>

daeStringRefArrayArray		daeMetaAttribute::_NameBindings;
daeMetaAttributeRefArray	daeMetaAttribute::_FactoryTemplates;

void
daeMetaAttribute::set(daeElement* e, daeString s)
{
	if( _type->getTypeEnum() == daeAtomicType::FloatType || _type->getTypeEnum() == daeAtomicType::DoubleType ) {
		if ( strcmp(s, "NaN") == 0 ) {
			fprintf(stderr, "NaN encountered while setting %s attribute in %s element.\n", (daeString)_name, (daeString)_container->getName() );
			fflush(stderr);
		}
		else if ( strcmp(s, "INF") == 0 ) {
			fprintf(stderr, "INF encountered while setting %s attribute in %s element.\n", (daeString)_name, (daeString)_container->getName() );
			fflush(stderr);
		}
	}
	_type->stringToMemory((char*)s, getWritableMemory(e));
	_isValid=true;
}

void daeMetaElementAttribute::set(daeElement* e, daeString s)
{
	//_type->stringToMemory((char*)s, getWritableMemory(e));
	daeElementRef *ref = (daeElementRef*)(getWritableMemory(e));
	if ((*ref) == NULL) {
		(*ref) = _elementType->create();
	}
	(*ref)->getMeta()->getValueAttribute()->set((*ref), s);
	_isValid=true;
}

void daeMetaAttribute::copy(daeElement* to, daeElement *from) {
	daeChar str[4096];
	_type->memoryToString( getWritableMemory(from), str, 2048 );
	_type->stringToMemory( str, getWritableMemory( to ) );
	//memcpy( getWritableMemory( to ), getWritableMemory( from ), getSize() );
	_isValid=true;
}

void
daeMetaArrayAttribute::set(daeElement* e, daeString s)
{
	if( _type->getTypeEnum() == daeAtomicType::FloatType || _type->getTypeEnum() == daeAtomicType::DoubleType ) {
		if ( strcmp(s, "NaN") == 0 ) {
			fprintf(stderr, "NaN encountered while setting %s attribute in %s element.\n", (daeString)_name, (daeString)_container->getName() );
			fflush(stderr);
		}
		else if ( strcmp(s, "INF") == 0 ) {
			fprintf(stderr, "INF encountered while setting %s attribute in %s element.\n", (daeString)_name, (daeString)_container->getName() );
			fflush(stderr);
		}
	}
	daeArray* array = (daeArray*)getWritableMemory(e);
	daeInt typeSize = _type->getSize();
	daeInt cnt = (daeInt)array->getCount();
	array->setRawCount(++cnt);
	_type->stringToMemory((char*)s, array->getRawData()+(cnt-1)*typeSize);
	_isValid=true;
}

void daeMetaArrayAttribute::copy(daeElement* to, daeElement *from) {
	daeArray* toArray = (daeArray*)getWritableMemory(to);
	daeArray* fromArray = (daeArray*)getWritableMemory(from);
	daeInt typeSize = _type->getSize();
	daeInt cnt = (daeInt)fromArray->getCount();
	toArray->setRawCount( cnt );
	//memcpy( toArray->getRawData(), fromArray->getRawData(), cnt * typeSize );
	daeChar *toData = toArray->getRawData();
	daeChar *fromData = fromArray->getRawData();
	daeChar str[4096];
	for ( int i = 0; i < cnt; i++ ) {
		_type->memoryToString( fromData+i*typeSize, str, 2048 );
		_type->stringToMemory( str, toData+i*typeSize );
	}
}


void daeMetaElementAttribute::copy(daeElement* to, daeElement *from) {
	daeElement *cpy = (*(daeElementRef*)(getWritableMemory(from)))->clone();
	(*(daeElementRef*)(getWritableMemory(to))) = cpy;
}
void daeMetaElementArrayAttribute::copy(daeElement* to, daeElement *from) {
	(void)to;
	(void)from;
}

daeMetaElementArrayAttribute::daeMetaElementArrayAttribute()
{
}

void
daeMetaElementAttribute::placeElement(daeElement* parent, daeElement* child)
{
	if (parent == NULL)
		return;
	
	daeElementRef* er = (daeElementRef*)getWritableMemory(parent);
	*er = child;
}

void
daeMetaElementArrayAttribute::placeElement(daeElement* parent,
										   daeElement* child)
{
	if ((parent == NULL)||(child == NULL))
		return;
	
	daeElementRefArray* era = (daeElementRefArray*)getWritableMemory(parent);
	era->append(child);
}
// !!!GAC added for testing 7/8/05
// These are the opposite of the placeElement functions above
void
daeMetaElementAttribute::removeElement(daeElement* parent, daeElement* child)
{
  (void)child; // silence unused variable warning

	if (parent == NULL)
		return;
	
	daeElementRef* er = (daeElementRef*)getWritableMemory(parent);
	*er = NULL;
}

void
daeMetaElementArrayAttribute::removeElement(daeElement* parent,
										   daeElement* child)
{
	if ((parent == NULL)||(child == NULL))
		return;
	
	daeElementRefArray* era = (daeElementRefArray*)getWritableMemory(parent);
	era->remove(child);
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

void
daeMetaAttribute::InitializeKnownTypes()
{
	daeInt index;
	index = (daeInt)_FactoryTemplates.append(new daeMetaAttribute);
	_NameBindings[index].append("int");
	_NameBindings[index].append("float");
	_NameBindings[index].append("string");
	_NameBindings[index].append("enum");
	
	index = (daeInt)_FactoryTemplates.append(new daeMetaArrayAttribute);
	_NameBindings[index].append("ListOfFloats");
	_NameBindings[index].append("ListOfInts");
	_NameBindings[index].append("ListOfTokens");

	index = (daeInt)_FactoryTemplates.append(new daeMetaElementAttribute);
	_NameBindings[index].append("xs:element");
	
	index = (daeInt)_FactoryTemplates.append(new daeMetaElementArrayAttribute);
	_NameBindings[index].append("element");
	
	//index = (daeInt)_FactoryTemplates.append(new daeMetaEnumAttribute);
	//_NameBindings[index].append("__enum");
}

daeMetaAttributeRef
daeMetaAttribute::Factory(daeStringRef xmlTypeName)
{
	unsigned int i;
	for(i=0;i<_FactoryTemplates.getCount();i++) {
		
		daeStringRefArray& nameBindings = _NameBindings[i];
		int count = (int)nameBindings.getCount();
		int j;
		for(j=0;j<count;j++)
			if (!strcmp(nameBindings[j],xmlTypeName))
				break;
		if (j!=count)
			return _FactoryTemplates[i]->clone();
	}
	
	return NULL;
}

daeMetaAttributeRef
daeMetaAttribute::clone()
{
	return new daeMetaAttribute;
}

daeMetaAttributeRef
daeMetaArrayAttribute::clone()
{
	return new daeMetaArrayAttribute;
}
//daeMetaAttributeRef
//daeMetaEnumAttribute::clone()
//{
//	return new daeMetaEnumAttribute;
//}

daeMetaElementAttribute::daeMetaElementAttribute()
{
	_minOccurs = 1;
	_maxOccurs = 1;
	_isInChoice = false;
	_isInSequence = false;
	//_ref = "noref";
	_previousInSequence = NULL;
	_elementType = NULL;
}

daeMetaAttributeRef
daeMetaElementAttribute::clone()
{
	return new daeMetaElementAttribute;
}

daeMetaAttributeRef
daeMetaElementArrayAttribute::clone()
{
	return new daeMetaElementArrayAttribute;
}

//daeMetaEnumAttribute::daeMetaEnumAttribute()
//{
//}

daeMetaAttribute::daeMetaAttribute()
{
	_name = "noname";
	_offset = -1;
	_type = NULL;
	_container = NULL;
	_default = NULL;
	_isValid = false;
	_isRequired = false;
}

void
daeMetaAttribute::resolve(daeElementRef element)
{
	if (_type != NULL)
		_type->resolve(element, this);
}

daeInt
daeMetaAttribute::getSize()
{
	return _type->getSize();
}
daeInt
daeMetaAttribute::getAlignment()
{
	return _type->getAlignment();
}

//!!!ACL 10-18
daeInt
daeMetaAttribute::getCount(daeElement* e)
{
	if (e == NULL)
		return 0;
	return (getWritableMemory(e) != NULL);
}

daeMemoryRef
daeMetaAttribute::get(daeElement *e, daeInt index)
{
	(void)index; 
	return getWritableMemory(e);
}

daeInt
daeMetaArrayAttribute::getCount(daeElement *e)
{
	if (e == NULL)
		return 0;
	daeArray* era = (daeArray*)getWritableMemory(e);
	if (era == NULL)
		return 0;
	return (daeInt)era->getCount();
}

daeMemoryRef
daeMetaArrayAttribute::get(daeElement* e, daeInt index)
{
	if (e == NULL)
		return NULL;
	daeArray* era = (daeArray*)getWritableMemory(e);
	if (era == NULL || index >= (daeInt)era->getCount() )
		return NULL;
	return era->getRawData()+(index*era->getElementSize());
}
//******************************************************
