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

#include <dae/daeMetaAny.h>
#include <dae/domAny.h>
#include <dae/daeMetaElementAttribute.h>

daeMetaAny::daeMetaAny( daeMetaElement *container, daeMetaCMPolicy *parent, daeUInt ordinal,
												 daeInt minO, daeInt maxO) : daeMetaCMPolicy( container, parent, ordinal, minO, maxO )
{}

daeMetaAny::~daeMetaAny()
{}

daeElement *daeMetaAny::placeElement( daeElement *parent, daeElement *child, daeUInt &ordinal, daeInt offset, daeElement* before, daeElement *after ) {
	//remove element from praent
	(void)offset;
	(void)before;
	(void)after;
	daeElement::removeFromParent( child );
	child->setParentElement( parent );
	//*************************************************************************
	ordinal = 0;
	return child;
}

daeBool daeMetaAny::removeElement( daeElement *parent, daeElement *child ) {
	(void)parent;
	(void)child;
	return true;
}

daeMetaElement * daeMetaAny::findChild( daeString elementName ) {
	if ( elementName != NULL ) {
		const daeMetaElementRefArray &metas = daeMetaElement::getAllMetas();
		size_t cnt = metas.getCount();
		for ( size_t x = 0; x < cnt; x++ ) {
			if ( !metas[x]->getIsInnerClass() && strcmp( elementName, metas[x]->getName() ) == 0 ) {
				return metas[x];
			}
		}
	}
	return domAny::registerElement();
}

void daeMetaAny::getChildren( daeElement *parent, daeElementRefArray &array ) {
	(void)parent;
	(void)array;
	//this is taken care of by the _contents in metaElement
}

