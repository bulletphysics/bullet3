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

#include <dae/daeSIDResolver.h>
#include <dae/daeIDRef.h>
#include <dae/daeAtomicType.h>
#include <dae/daeMetaAttribute.h>
#include <dae/daeMetaElement.h>
#include <dae/daeURI.h>

daeSIDResolver::daeSIDResolver( daeElement *container, daeString target, daeString profile )
{
	element = NULL;
	doubleArray = NULL;
	doublePtr = NULL;

	this->container = container;
	if ( target != NULL ) {
		this->target = new char[ strlen( target ) +1 ];
		strcpy( (char*)this->target, target );
		state = target_loaded;
	}
	else {
		this->target = NULL;
		state = target_empty;
	}
	if ( profile != NULL ) {
		this->profile = new char[ strlen( profile ) +1 ];
		strcpy( (char*)this->profile, profile );
	}
	else {
		this->profile = NULL;
	}
}

daeSIDResolver::~daeSIDResolver()
{
	if ( target != NULL ) {
		delete[] target;
		target = NULL;
	}
	if ( profile != NULL ) {
		delete[] profile;
		profile = NULL;
	}
}

void daeSIDResolver::setTarget( daeString t )
{
	if ( target != NULL ) {
		delete[] target;
	}
	if ( t != NULL ) {
		target = new char[ strlen( t ) +1 ];
		strcpy( (char*)target, t );
		state = target_loaded;
	}
	else {
		target = NULL;
		state = target_empty;
	}
	element = NULL;
	doubleArray = NULL;
	doublePtr = NULL;
}


void daeSIDResolver::setProfile( daeString p )
{
	if ( profile != NULL ) {
		delete[] target;
	}
	if ( p != NULL ) {
		profile = new char[ strlen( p ) +1 ];
		strcpy( (char*)profile, p );
	}
	else {
		profile = NULL;
	}
	element = NULL;
	doubleArray = NULL;
	doublePtr = NULL;
}

void daeSIDResolver::setContainer(daeElement* element)
{
	if ( element != container ) {
		container = element;
		element = NULL;
		doubleArray = NULL;
		doublePtr = NULL;
		if ( target != NULL ) {
			state = target_loaded;
		}
		else {
			state = state = target_empty;
		}
	}
}

daeElementRef daeSIDResolver::getElement()
{
	if ( state == target_loaded ) {
		resolve();
	}
	return element;
}

daeDoubleArray *daeSIDResolver::getDoubleArray()
{
	if ( state == target_loaded ) {
		resolve();
	}
	return doubleArray;
}

daeDouble *daeSIDResolver::getDouble()
{
	if ( state == target_loaded ) {
		resolve();
	}
	return doublePtr;
}

void daeSIDResolver::resolve()
{
	char * str = (char *)target;
	char * pos = strchr( str, '/');
	char * id;
	if ( pos == NULL ) {
		pos = strchr( str, '.' );
	}
	if ( pos == NULL ) {
		pos = strchr( str, '(' );
	}
	if ( pos != NULL ) {
		id = new char[ pos - str + 1 ];
		strncpy( id, str, pos - str );
		id[ pos - str ] = 0;
		str = pos;
	}
	else {
		id = new char[ strlen( str ) + 1 ]; 
		strcpy( id, str );
		str = str + strlen( str );
	}
	if ( strcmp( id, "." ) == 0 ) {
		element = container;
		state = sid_success_element;
	}
	else {
		daeIDRef idref( id );
		idref.setContainer( container );
		idref.resolveElement();
		if ( idref.getState() != daeIDRef::id_success ) {
			state = sid_failed_not_found;
			delete[] id;
			element = NULL;
			return;
		}
		element = idref.getElement();
		state = sid_success_element;
	}

	char * next = NULL;
	while ( *str != '.' && *str != '(' && *str != 0 ) {
		if ( *str == '/' ) {
			str++;
		}
		if ( next != NULL ) {
			delete[] next;
			next = NULL;
		}
		pos = strchr( str, '/');
		if ( pos == NULL ) {
			pos = strchr( str, '.' );
		}
		if ( pos == NULL ) {
			pos = strchr( str, '(' );
		}
		if ( pos != NULL ) {
			next = new char[ pos - str + 1 ];
			strncpy( next, str, pos - str );
			next[ pos - str ] = 0;
			str = pos;
		}
		else {
			next = new char[ strlen( str ) + 1 ]; 
			strcpy( next, str );
			str = str + strlen( str );
		}
		//find the child element with SID of next
		daeElement *el = findSID( element, next );
		element = el;
		if ( element == NULL ) {
			//failed
			state = sid_failed_not_found;
			if ( id != NULL ) {
				delete[] id;
			}
			if ( next != NULL ) {
				delete[] next;
				next = NULL;
			}
			return;
		}
	}
	//check for the double array
	if ( strcmp( element->getTypeName(), "source" ) == 0 ) {
		daeElementRefArray children;
		element->getChildren( children );
		size_t cnt = children.getCount();

		for ( size_t x = 0; x < cnt; x++ ) {
			if ( strcmp( children[x]->getTypeName(), "float_array" ) == 0 ) {
				doubleArray = (daeDoubleArray*)children[x]->getMeta()->getValueAttribute()->getWritableMemory( children[x] );
				state = sid_success_array;
				break;
			}
		}
	}
	else 
	{
		daeMetaAttribute *ma = element->getMeta()->getValueAttribute();
		if ( ma != NULL ) {
			if ( ma->isArrayAttribute() && ma->getType()->getTypeEnum() == daeAtomicType::DoubleType ) {
				doubleArray = (daeDoubleArray*)ma->getWritableMemory( element );
				state = sid_success_array;
			}
		}
	}

	if( state == sid_success_array ) {
		//found the double array
		if ( *str == '.' ) {
			//do the double lookup stuff based on COMMON profile offset
			str++;
			if ( strcmp( str, "ANGLE" ) == 0 ) {
				doublePtr = &(doubleArray->get(3));
				state = sid_success_double;
			}
			else if ( strlen( str ) == 1 ) {
				switch ( *str ) {
					case 'X':
					case 'R':
					case 'U':
					case 'S':
						doublePtr = &(doubleArray->get(0));
						state = sid_success_double;
						break;
					case 'Y':
					case 'G':
					case 'V':
					case 'T':
						doublePtr = &(doubleArray->get(1));
						state = sid_success_double;
						break;
					case 'Z':
					case 'B':
					case 'P':
						doublePtr = &(doubleArray->get(2));
						state = sid_success_double;
						break;
					case 'W':
					case 'A':
					case 'Q':
						doublePtr = &(doubleArray->get(3));
						state = sid_success_double;
						break;
				};
			}
		}
		else if ( *str == '(' ) {
			//do the double lookup stuff based on the offset given
			str++;
			pos = strchr( str, '(' );
			daeInt i = atoi( str );
			if ( pos != NULL && doubleArray->getCount() == 16 ) {
				//we are doing a matrix lookup
				pos++;
				daeInt j = atoi( pos );
				doublePtr = &(doubleArray->get( i*4 + j ));
				state = sid_success_double;
			}
			else {
				//vector lookup
				if ( (daeInt)doubleArray->getCount() > i ) {
					doublePtr = &(doubleArray->get(i));
					state = sid_success_double;
				}
			}
		}
	}

	if ( id != NULL ) {
		delete[] id;
	}
	if ( next != NULL ) {
		delete[] next;
	}
}

daeElement *daeSIDResolver::findSID( daeElement *el, daeString sid ) {
	//first check yourself
	daeString *s = (daeString*)el->getAttributeValue( "sid" );
	if ( s != NULL && *s != NULL && strcmp( *s, sid ) == 0 ) {
		//found it
		return el;
	}
	//and if you are a instance_* then check what you point to
	daeString nm = el->getElementName();
	if ( nm == NULL ) {
		nm = el->getTypeName();
	}
	if ( strncmp( nm, "instance_", 9 ) == 0 ) {
		daeURI *uri = (daeURI*)el->getAttributeValue("url");
		if ( uri != NULL && uri->getElement() != NULL ) {
			daeElement *e = findSID( uri->getElement(), sid );
			if ( e != NULL ) {
				//found it
				return e;
			}
		}
	}
	
	daeElementRefArray children;
	el->getChildren( children );
	size_t cnt = children.getCount();
	for ( size_t x = 0; x < cnt; x++ ) {
		//examine the children
		//char s[56]; 
		//daeAtomicType::get( "token" )->memoryToString( children[x]->getAttributeValue( "sid" ), s, 56 );
		daeString *s = (daeString*)children[x]->getAttributeValue( "sid" );
		if ( s != NULL && *s != NULL && strcmp( *s, sid ) == 0 ) {
			//found it
			return children[x];
		}
	}
	for ( size_t x = 0; x < cnt; x++ ) {
		//if not found look for it in each child
		if ( profile != NULL && strcmp( children[x]->getTypeName(), "technique_COMMON" ) == 0 ) {
			//not looking for common profile
			continue;
		}
		else if ( strcmp( children[x]->getTypeName(), "technique" ) == 0 && children[x]->hasAttribute( "profile" ) ) {
			if ( profile == NULL || strcmp( profile, children[x]->getAttributeValue( "profile" ) ) != 0 ) {
				//not looking for this technique profile
				continue;
			}		
		}
		daeElement *e = findSID( children[x], sid );
		if ( e != NULL ) {
			//found it
			return e;
		}
	}
	return NULL;
}

