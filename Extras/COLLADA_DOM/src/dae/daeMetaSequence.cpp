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

#include <dae/daeMetaSequence.h>

daeMetaSequence::daeMetaSequence( daeMetaElement *container, daeMetaCMPolicy *parent, daeUInt ordinal,
									daeInt minO, daeInt maxO) : 
									daeMetaCMPolicy( container, parent, ordinal, minO, maxO )
{}

daeMetaSequence::~daeMetaSequence()
{}

daeElement *daeMetaSequence::placeElement( daeElement *parent, daeElement *child, daeUInt &ordinal, daeInt offset, daeElement* before, daeElement *after ) {
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

	for ( daeInt i = 0; ( i < _maxOccurs || _maxOccurs == -1 ); i++ ) {
		for ( size_t x = 0; x < _children.getCount(); x++ ) {
			if ( _children[x]->placeElement( parent, child, ordinal, i, before, after ) != NULL ) {
				ordinal = ordinal + (i * ( _maxOrdinal + 1 )) + _ordinalOffset;
				return child;
			}
		}
	}
	return NULL;
}

daeBool daeMetaSequence::removeElement( daeElement *parent, daeElement *child ) {
	for ( size_t x = 0; x < _children.getCount(); x++ ) {
		if ( _children[x]->removeElement( parent, child ) ) {
			return true;
		}
	}
	return false;
}

daeMetaElement * daeMetaSequence::findChild( daeString elementName ) {
	daeMetaElement *me = NULL;
	for ( size_t x = 0; x < _children.getCount(); x++ ) {
		me = _children[x]->findChild( elementName );
		if ( me != NULL ) {
			return me;
		}
	}
	return NULL;
}

void daeMetaSequence::getChildren( daeElement *parent, daeElementRefArray &array ) {
	for ( size_t x = 0; x < _children.getCount(); x++ ) {
		_children[x]->getChildren( parent, array );
	}
}
