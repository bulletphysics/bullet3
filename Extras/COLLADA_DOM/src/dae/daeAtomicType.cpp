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

#include <dae/daeAtomicType.h>
#include <dae/daeElement.h>
#include <dae/daeURI.h>
#include <dae/daeIDRef.h>
#include <dae/daeMetaElement.h>
#include <dae/daeDatabase.h>

daeAtomicTypeArray* daeAtomicType::_Types = NULL;
daeBool daeAtomicType::_TypesInitialized = false;

void
daeAtomicType::initializeKnownTypes()
{
	_Types = new daeAtomicTypeArray;
	initializeKnownBaseTypes();
	//mandatory to set here, because the array types are querying the atomic types
	_TypesInitialized = true;
}

void 
daeAtomicType::uninitializeKnownTypes()
{
	_TypesInitialized = false;
	unsigned int i;
	for (i=0;i<_Types->getCount();i++)
	{
		daeAtomicType* type = _Types->get(i);
		delete type;
	}
	delete _Types;
}

void
daeAtomicType::initializeKnownBaseTypes()
{
	_Types->append(new daeUIntType);
	_Types->append(new daeIntType);
	_Types->append(new daeLongType);
	_Types->append(new daeShortType);
	_Types->append(new daeUIntType);
	_Types->append(new daeULongType);
	_Types->append(new daeFloatType);
	_Types->append(new daeDoubleType);
	_Types->append(new daeStringRefType);
	_Types->append(new daeElementRefType);
	_Types->append(new daeEnumType);
	_Types->append(new daeRawRefType);
	_Types->append(new daeResolverType);
	_Types->append(new daeIDResolverType);
	_Types->append(new daeBoolType);
	_Types->append(new daeTokenType);
}

daeAtomicType*
daeAtomicType::get(daeStringRef typeString)
{
	if (!_TypesInitialized)
		daeAtomicType::initializeKnownTypes();

	int tCount = (int)_Types->getCount();
	int i;
	for(i=0; i<tCount; i++) {
		daeAtomicType* type = _Types->get(i);
		daeStringRefArray& nameBindings = type->getNameBindings();
		int count = (int)nameBindings.getCount();
		int j;
		for(j=0;j<count;j++)
			if (!strcmp(nameBindings[j],typeString))
				break;
		if (j!=count)
			return type;
	}
	return NULL;
}
daeAtomicType::daeAtomicType()
{
	_size = -1;
	_alignment = -1;
	_typeEnum = -1;
	_typeString = "notype";
	_printFormat = "badtype";
	_scanFormat = "";
	_maxStringLength = -1;
}

daeAtomicType*
daeAtomicType::get(daeEnum typeEnum)
{
	if (!_TypesInitialized)
		daeAtomicType::initializeKnownTypes();

	int tCount = (int)_Types->getCount();
	int i;
	for(i=0; i<tCount; i++) {
		daeAtomicType* type = _Types->get(i);
		if (type->getTypeEnum() == typeEnum)
			return type;
	}
	return NULL;
}

daeBool
daeAtomicType::stringToMemory(daeChar *src, daeChar* dstMemory)
{
	sscanf(src, _scanFormat, dstMemory);
	return true;
}

daeBool
daeAtomicType::memoryToString(daeChar* src, daeChar* dst, daeInt dstSize)
{
	// just to remove the warnings 
	(void)src;
	
	if (dstSize > 32)
		sprintf(dst,"unknown type string conversion\n");
	return true;
}

daeInt
daeAtomicType::append(daeAtomicType* t) {
	if (!_TypesInitialized)
		daeAtomicType::initializeKnownTypes();
	return (daeInt)_Types->append(t);
}
	
const daeAtomicType*
daeAtomicType::getByIndex(daeInt index) {
	return _Types->get(index);
}
	
daeInt
daeAtomicType::getCount() {
	return (daeInt)_Types->getCount();
}

daeEnumType::daeEnumType()
{
	_size = sizeof(daeEnum);
	_alignment = sizeof(daeEnum);
	_typeEnum = EnumType;
	_nameBindings.append("enum");
	_printFormat = "%s";//"%%.%ds";
	_scanFormat = "%s";
	_strings = NULL;
	_values = NULL;
	_typeString = "enum";
}

daeEnumType::~daeEnumType() {
	if ( _strings ) {
		delete _strings;
		_strings = NULL;
	}
	if ( _values ) {
		delete _values;
		_values = NULL;
	}
}

daeBoolType::daeBoolType()
{
	_size = sizeof(daeBool);
	_alignment = sizeof(daeBool);
	_typeEnum = BoolType;
	_printFormat = "%d";
	_scanFormat = "%d";
	_typeString = "bool";
	_maxStringLength = (daeInt)strlen("false")+1;
	_nameBindings.append("bool");
	//_nameBindings.append("xsBool");
	_nameBindings.append("xsBoolean");
}

daeIntType::daeIntType()
{
	_size = sizeof(daeInt);
	_alignment = sizeof(daeInt);
	_typeEnum = IntType;
	_maxStringLength = 16;
	_nameBindings.append("int");
	_nameBindings.append("xsInteger");
	_nameBindings.append("xsHexBinary");
	_nameBindings.append("xsIntegerArray");
	_nameBindings.append("xsHexBinaryArray");
	_nameBindings.append("xsByte");
	_nameBindings.append("xsInt");
	_printFormat = "%d";
	_scanFormat = "%d";
	_typeString = "int";
}
daeLongType::daeLongType()
{
	_size = sizeof(daeLong);
	_alignment = sizeof(daeLong);
	_typeEnum = LongType;
	_maxStringLength = 32;
	_nameBindings.append("xsLong");
	_nameBindings.append("xsLongArray");
	_printFormat = "%ld";
	_scanFormat = "%ld";
	_typeString = "long";
}
daeShortType::daeShortType()
{
	_maxStringLength = 8;
	_size = sizeof(daeShort);
	_alignment = sizeof(daeShort);
	_typeEnum = ShortType;
	_nameBindings.append("short");
	_nameBindings.append("xsShort");
	_printFormat = "%hd";
	_scanFormat = "%hd";
	_typeString = "short";
}
daeUIntType::daeUIntType()
{
	_maxStringLength = 16;
	_size = sizeof(daeUInt);
	_alignment = sizeof(daeUInt);
	_typeEnum = UIntType;
	_nameBindings.append("uint");
	_nameBindings.append("xsNonNegativeInteger");
	_nameBindings.append("xsUnsignedByte");
	_nameBindings.append("xsUnsignedInt");
	_nameBindings.append("xsPositiveInteger");
	_printFormat = "%u";
	_scanFormat = "%u";
	_typeString = "uint";
}
daeULongType::daeULongType()
{
	_size = sizeof(daeULong);
	_alignment = sizeof(daeULong);
	_typeEnum = ULongType;
	_maxStringLength = 32;
	_nameBindings.append("ulong");
	_nameBindings.append("xsUnsignedLong");
	_printFormat = "%lu";
	_scanFormat = "%lu";
	_typeString = "ulong";
}
daeFloatType::daeFloatType()
{
	_maxStringLength = 64;
	_size = sizeof(daeFloat);
	_alignment = sizeof(daeFloat);
	_typeEnum = FloatType;
	_nameBindings.append("float");
	_nameBindings.append("xsFloat");
	_printFormat = "%g";
	_scanFormat = "%g";
	_typeString = "float";
}
daeDoubleType::daeDoubleType()
{
	_size = sizeof(daeDouble);
	_alignment = sizeof(daeDouble);
	_typeEnum = DoubleType;
	_nameBindings.append("double");
	_nameBindings.append("xsDouble");
	_nameBindings.append("xsDecimal");
	_printFormat = "%lg";
	_scanFormat = "%lg";
	_typeString = "double";
	_maxStringLength = 64;
}

daeStringRefType::daeStringRefType()
{
	_size = sizeof(daeStringRef);
	_alignment = sizeof(daeStringRef);
	_typeEnum = StringRefType;
	_nameBindings.append("string");
	_nameBindings.append("xsString");
	_nameBindings.append("xsDateTime");	
	_printFormat = "%s";
	_scanFormat = "%s";
	_typeString = "string";
}

daeTokenType::daeTokenType()
{
	_size = sizeof(daeStringRef);
	_alignment = sizeof(daeStringRef);
	_typeEnum = TokenType;
	_nameBindings.append("token");
	_nameBindings.append("xsID");
	_nameBindings.append("xsNCName");
	_nameBindings.append("xsNMTOKEN");
	_nameBindings.append("xsName");
	_nameBindings.append("xsToken");
	_nameBindings.append("xsNameArray");
	_nameBindings.append("xsTokenArray");
	_nameBindings.append("xsNCNameArray");
	_printFormat = "%s";
	_scanFormat = "%s";
	_typeString = "token";
}

daeElementRefType::daeElementRefType()
{
	_size = sizeof(daeElementRef);
	_alignment = sizeof(daeElementRef);
	_typeEnum = ElementRefType;
	_nameBindings.append("element");
	_nameBindings.append("Element");
	_nameBindings.append("TrackedElement");
	_printFormat = "%p";
	_scanFormat = "%p";
	_typeString = "element";
	_maxStringLength = 64;
}

daeRawRefType::daeRawRefType()
{
	_size = sizeof(daeRawRef);
	_alignment = sizeof(daeRawRef);
	_typeEnum = RawRefType;
	_nameBindings.append("raw");
	_printFormat = "%p";
	_scanFormat = "%p";
	_typeString = "raw";
	_maxStringLength = 64;
}

daeResolverType::daeResolverType()
{
	_size = sizeof(daeURI);
	_alignment = sizeof(daeURI);
	_typeEnum = ResolverType;
	_nameBindings.append("resolver");
	_nameBindings.append("xsAnyURI");
	_printFormat = "%s";
	_scanFormat = "%s";
	_typeString = "resolver";
}
daeIDResolverType::daeIDResolverType()
{
	_size = sizeof(daeIDRef);
	_alignment = sizeof(daeIDRef);
	_typeEnum = IDResolverType;
	_nameBindings.append("xsIDREF");
	_nameBindings.append("xsIDREFS");
	_printFormat = "%s";
	_scanFormat = "%s";
	_typeString = "idref_resolver";
}
daeBool 
	daeIntType::memoryToString(daeChar* src, daeChar* dst, daeInt dstSize) 
{ 
	if (_maxStringLength > dstSize) return false;
	sprintf(dst,_printFormat,*((daeInt*)src));
	return true;
}
daeBool 
	daeLongType::memoryToString(daeChar* src, daeChar* dst, daeInt dstSize) 
{ 
	if (_maxStringLength > dstSize) return false;
	sprintf(dst,_printFormat,*((daeLong*)src));
	return true;
}
daeBool 
	daeShortType::memoryToString(daeChar* src, daeChar* dst, daeInt dstSize) 
{ 
	if (_maxStringLength > dstSize) return false;
	sprintf(dst,_printFormat,*((daeShort*)src));
	return true;
}
daeBool 
	daeUIntType::memoryToString(daeChar* src, daeChar* dst, daeInt dstSize) 
{ 
	if (_maxStringLength > dstSize) return false;
	sprintf(dst,_printFormat,*((daeUInt*)src));
	return true;
}
daeBool 
	daeULongType::memoryToString(daeChar* src, daeChar* dst, daeInt dstSize) 
{ 
	if (_maxStringLength > dstSize) return false;
	sprintf(dst,_printFormat,*((daeULong*)src));
	return true;
}
daeBool 
	daeFloatType::memoryToString(daeChar* src, daeChar* dst, daeInt dstSize) 
{ 
	if (_maxStringLength > dstSize) return false;
	sprintf(dst,_printFormat,*((daeFloat*)src));
	return true;
}

daeBool 
	daeDoubleType::memoryToString(daeChar* src, daeChar* dst, daeInt dstSize) 
{ 
	if (_maxStringLength > dstSize) return false;
	sprintf(dst,_printFormat,*((daeDouble*)src));
	return true;
}
daeBool 
	daeRawRefType::memoryToString(daeChar* src, daeChar* dst, daeInt dstSize)
{
	if (_maxStringLength > dstSize) return false;
	sprintf(dst,"%p",(void *)(*((daeRawRef*)src)));
	return true;
}

daeBool
daeStringRefType::getUsesStrings()
{
	return true;
}
daeBool
daeTokenType::getUsesStrings()
{
	return false;
}
daeBool
daeStringRefType::memoryToString(daeChar* src, daeChar* dst, daeInt dstSize)
{
	daeString s = *((daeStringRef *)src);
	if (!s || strlen(s) == 0)
		dst[0] = '\0';
	else {
		char tmp[64];
		sprintf(tmp,"%%.%ds",dstSize-1);
		sprintf(dst,tmp,(const char*)s);
		
		if ((daeInt)(strlen(s)+1) > dstSize)
			return false;
	}
	return true;
}

daeBool
daeResolverType::memoryToString(daeChar* src, daeChar* dst, daeInt dstSize)
{
#if 1
	// Get the URI we are trying to write
	daeURI *thisURI = ((daeURI *)src);
	daeString s;

	// !!!GAC We may want to re-resolve the URI before writing, if so call thisURI->resolveURI() here
	// !!!GAC if you're willing to trust that everything is properly resolved, this isn't needed
	
	// Was this URI successfully resolved ?  (if element or collection is null, we can't write the URI correctly)
	if(thisURI->getState() != daeURI::uri_success || !(thisURI->getElement()) || !(thisURI->getContainer()))
	{
		// This URI was never successfully resolved, so write out it's original value
		s = thisURI->getOriginalURI();
	}
	else
	{
		// This URI was successfully resolved, we need to determine if it is referencing this document (the one being written)
		// or some other document so we know what URI to write out.
		// !!!GAC this approach should be safe, if the collection pointer of our document matches the collection pointer 
		// !!!GAC of the element our URI is pointing at, we are pointing at our own doc.
		if(thisURI->getElement()->getCollection() == thisURI->getContainer()->getDocument())
		{
			// we will send back the original URI if we're pointing at ourselves
			s = thisURI->getOriginalURI();
		}
		else
		{
			// !!!GAC change this to test outputting of relative URIs, NOT FULLY TESTED!!!
#if 1
			// we will send back the full resolved URI
			s = thisURI->getURI();
#else
			// Makes the URI relative to the document being written, EXPERIMENTAL, not fully tested!!!
			thisURI->makeRelativeTo(thisURI->getDocument()->getCollection()->getDocumentURI());
			s = thisURI->getOriginalURI();
#endif
		}
	}
	// Copy at most dstSize-1 characters, null terminate and return error if the string was too long
	daeChar *d;
	int i;
	for(d = dst, i = 1;	*s != 0 && i<dstSize; s++, d++, i++)
	{
		// If the URI contains spaces, substitute %20
		if(*s == ' ')
		{
			if( (i+2)<dstSize)
			{
				*(d++)='%';
				*(d++)='2';
				*d = '0';
			}
			else
			{
				// not enough space to escape the string so null terminate and return error
				*d = 0;
				return(false);
			}
		}
		else
		{			
			*d = *s;
		}
	}
	*d = 0;
	if(*s == 0)
		return(true);
	else
		return(false);
#else
	// !!!GAC This is the old code which doesn't work for cross document references
	//	((daeURI*)src)->resolveURI();

	// Get the URI String as set, not the composited one from the base
	// as per SCEA request
	daeString s = ((daeURI *)src)->getOriginalURI();
	char tmp[64];
	sprintf(tmp,"%%.%ds",dstSize-1);
	sprintf(dst,tmp,s);
	
	if ((daeInt)(strlen(s)+1) > dstSize)
		return false;
	return true;
#endif
}
daeBool
daeIDResolverType::memoryToString(daeChar* src, daeChar* dst, daeInt dstSize)
{
	((daeIDRef*)src)->resolveID();

	daeString s = ((daeIDRef *)src)->getID();
	char tmp[64];
	sprintf(tmp,"%%.%ds",dstSize-1);
	sprintf(dst,tmp,s);
	
	if ((daeInt)(strlen(s)+1) > dstSize)
		return false;
	return true;
}

void
daeAtomicType::resolve(daeElementRef element, daeMetaAttributeRef ma)
{
	// just to remove the warnings 
	(void)element;
	(void)ma; 
}

void
daeResolverType::resolve(daeElementRef element, daeMetaAttributeRef ma)
{
	daeURI* resolver = (daeURI*)ma->getWritableMemory(element);
	resolver->setContainer(element);
	resolver->resolveElement();
}

daeBool
daeResolverType::stringToMemory(daeChar* src, daeChar* dstMemory)
{
	((daeURI*)dstMemory)->setURI(src);
	return true;
}
void
daeIDResolverType::resolve(daeElementRef element, daeMetaAttributeRef ma)
{
	daeIDRef* resolver = (daeIDRef*)ma->getWritableMemory(element);
	resolver->setContainer( element );
	resolver->resolveElement();
}

daeBool
daeIDResolverType::stringToMemory(daeChar* src, daeChar* dstMemory)
{
	((daeIDRef*)dstMemory)->setID(src);
	return true;
}

daeBool
daeStringRefType::stringToMemory(daeChar* srcChars, daeChar* dstMemory)
{
	*((daeStringRef*)dstMemory) = srcChars;
	return true;
}

daeBool
daeEnumType::stringToMemory(daeChar* src, daeChar* dst )
{
	size_t index; 
	if ( _strings->find(src,index) == DAE_ERR_QUERY_NO_MATCH ) return false;
	daeEnum val = _values->get( index );
	*((daeEnum*)dst) = val;

	return true;
}

daeBool
daeEnumType::memoryToString(daeChar* src, daeChar* dst, daeInt dstSize)
{	
	daeStringRef s = "unknown";
	if (_strings != NULL) {
		size_t index;
		if (_values->find(*((daeEnum*)src), index) == DAE_OK)
			s = _strings->get(index);
	}
	sprintf(dst,_printFormat,(const char*)s);
	(void)dstSize;
	return true;
}
daeBool
daeBoolType::stringToMemory(daeChar* srcChars, daeChar* dstMemory)
{
	if (strncmp(srcChars,"true",4)==0)
		*((daeBool*)dstMemory) = true;
	else
		*((daeBool*)dstMemory) = false;
	return true;
}

daeBool
daeBoolType::memoryToString(daeChar* src, daeChar* dst, daeInt dstSize)
{
	if (*((daeBool*)src)) {
		if (dstSize < 5)
			return false;
		else
			sprintf(dst,"true");
	}
	else {
		if (dstSize < 6)
			return false;
		else
			sprintf(dst,"false");
	}
	return true;
}
//!!!ACL added for 1.4 complex types and groups

//unImplemented
daeBool
daeElementRefType::memoryToString(daeChar* src, daeChar* dst, daeInt dstSize)
{
	/*if (*((daeBool*)src)) {
		if (dstSize < 5)
			return false;
		else
			sprintf(dst,"true");
	}
	else {
		if (dstSize < 6)
			return false;
		else
			sprintf(dst,"false");
	}*/
	(void)src;
	(void)dst;
	(void)dstSize;
	return false;
}

