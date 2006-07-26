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

#include <dae/daeDom.h>
#include <dae/domAny.h>
#include <dae/daeMetaAttribute.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>
#include <dae/daeErrorHandler.h>

daeElementRef
domAny::create(daeInt bytes)
{
	domAnyRef ref = new(bytes) domAny;
	return ref;
}


daeMetaElement *
domAny::registerElement()
{
    //if ( _Meta != NULL ) return _Meta;
    daeMetaElement *_Meta = new daeMetaElement;
    _Meta->setName( "any" );
	//_Meta->setStaticPointerAddress(&domAny::_Meta);
	_Meta->registerConstructor(domAny::create);
	daeMetaCMPolicy *cm = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	cm = new daeMetaAny( _Meta, cm, 0, 0, -1 );

	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	
	_Meta->setAllowsAny( true );
	
	_Meta->addContents(daeOffsetOf(domAny,_contents));
	_Meta->addContentsOrder(daeOffsetOf(domAny,_contentsOrder));
	
	//VALUE
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsString"));
		ma->setOffset( daeOffsetOf( domAny , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}

	_Meta->setElementSize(sizeof(domAny));
	_Meta->validate();

	return _Meta;
}

//daeMetaElement * domAny::_Meta = NULL;

daeBool domAny::setAttribute(daeString attrName, daeString attrValue) {
	if (_meta == NULL)
		return false;
	
	//if the attribute already exists set it.
	daeMetaAttributeRefArray& metaAttrs = _meta->getMetaAttributes();
	int n = (int)metaAttrs.getCount();
	int i;
	for(i=0;i<n;i++) {
		fflush(stdout);
		if ((metaAttrs[i]->getName() != NULL) && (strcmp(metaAttrs[i]->getName(),attrName)==0)) {
			if (metaAttrs[i]->getType() != NULL) {
				metaAttrs[i]->set(this,attrValue);
				_validAttributeArray[i] = true;
			}
			return true;
		}
	}
	//else register it and then set it.
	if ( n >= MAX_ATTRIBUTES ) {
		daeErrorHandler::get()->handleWarning( "domAny::setAttribute() - too many attributes on this domAny.  The maximum number of attributes allowed is MAX_ATTRIBUTES" );
		return false;
	}
	daeMetaAttribute *ma = new daeMetaAttribute;
	ma->setName( attrName );
	ma->setType( daeAtomicType::get("xsString"));
	ma->setOffset( (daeInt)daeOffsetOf( domAny , attrs[n] ));
	ma->setContainer( _meta );
	_meta->appendAttribute(ma);
	_validAttributeArray.append( true );
	if (metaAttrs[i]->getType() != NULL) {
		metaAttrs[i]->set(this,attrValue);
		return true;
	}
	
	return false;
}


