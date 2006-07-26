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

#include <dae/daeMetaChoice.h>

daeMetaChoice::daeMetaChoice( daeMetaElement *container, daeMetaCMPolicy *parent, daeUInt ordinal,
												 daeInt minO, daeInt maxO) : daeMetaCMPolicy( container, parent, ordinal, minO, maxO )
{}

daeMetaChoice::~daeMetaChoice()
{}

daeElement *daeMetaChoice::placeElement( daeElement *parent, daeElement *child, daeUInt &ordinal, daeInt offset, daeElement* before, daeElement *after ) {
	(void)offset;
	if ( _maxOccurs == -1 ) {
		//Needed to prevent infinate loops. If unbounded check to see if you have the child before just trying to place
		daeString nm = child->getElementName();
		if ( nm == NULL ) {
			nm = child->getTypeName();
		}
		if ( findChild( nm ) == NULL ) {
			return NULL;
		}
	}

	daeElement *retVal = NULL;
	for ( daeInt i = 0; ( i < _maxOccurs || _maxOccurs == -1 ); i++ ) {
		for ( size_t x = 0; x < _children.getCount(); x++ ) {
			if ( _children[x]->placeElement( parent, child, ordinal, i, before, after ) != NULL ) {
				retVal = child;
				ordinal = ordinal  + _ordinalOffset;
				break;
			}
		}
		if ( retVal != NULL ) break;
	}
	/*if ( retVal && _maxOccurs != -1 ) {
		//check if the place was valid - only if we aren't unbounded. unbounded is always valid
		daeInt cnt = 0;
		daeElementRefArray array;
		size_t arrayCnt = 0; //saves us from having to clear the array every child
		for ( size_t x = 0; x < _children.getCount(); x++ ) {
			_children[x]->getChildren( parent, array );
			if ( array.getCount() != arrayCnt ) {
				//this part of the content model has children.
				cnt++;
				if ( cnt > _maxOccurs ) {
					//picked too many choices - remove element and return false
					removeElement( parent, child );
					return false;
				}
				arrayCnt = array.getCount();
			}
		}

	}*/
	return retVal;
}

daeBool daeMetaChoice::removeElement( daeElement *parent, daeElement *child ) {
	for ( size_t x = 0; x < _children.getCount(); x++ ) {
		if ( _children[x]->removeElement( parent, child ) ) {
			return true;
		}
	}
	return false;
}

daeMetaElement * daeMetaChoice::findChild( daeString elementName ) {
	daeMetaElement *me = NULL;
	for ( size_t x = 0; x < _children.getCount(); x++ ) {
		me = _children[x]->findChild( elementName );
		if ( me != NULL ) {
			return me;
		}
	}
	return NULL;
}

void daeMetaChoice::getChildren( daeElement *parent, daeElementRefArray &array ) {
	(void)parent;
	(void)array;
	//this is taken care of by the _contents in metaElement
}

