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
#include <dom/domMorph.h>

daeElementRef
domMorph::create(daeInt bytes)
{
	domMorphRef ref = new(bytes) domMorph;
	ref->attrSource.setContainer( (domMorph*)ref );
	return ref;
}


daeMetaElement *
domMorph::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "morph" );
	_Meta->setStaticPointerAddress(&domMorph::_Meta);
	_Meta->registerConstructor(domMorph::create);

	// Add elements: source, targets, extra
    _Meta->appendArrayElement(domSource::registerElement(),daeOffsetOf(domMorph,elemSource_array));
    _Meta->appendElement(domMorph::domTargets::registerElement(),daeOffsetOf(domMorph,elemTargets));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domMorph,elemExtra_array));

	//	Add attribute: method
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "method" );
		ma->setType( daeAtomicType::get("MorphMethodType"));
		ma->setOffset( daeOffsetOf( domMorph , attrMethod ));
		ma->setContainer( _Meta );
		ma->setDefault( "NORMALIZED");
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: source
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "source" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domMorph , attrSource ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domMorph));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domMorph::domTargets::create(daeInt bytes)
{
	domMorph::domTargetsRef ref = new(bytes) domMorph::domTargets;
	return ref;
}


daeMetaElement *
domMorph::domTargets::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "targets" );
	_Meta->setStaticPointerAddress(&domMorph::domTargets::_Meta);
	_Meta->registerConstructor(domMorph::domTargets::create);

	// Add elements: input, extra
    _Meta->appendArrayElement(domInputLocal::registerElement(),daeOffsetOf(domMorph::domTargets,elemInput_array),"input"); 
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domMorph::domTargets,elemExtra_array));
	
	
	_Meta->setElementSize(sizeof(domMorph::domTargets));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domMorph::_Meta = NULL;
daeMetaElement * domMorph::domTargets::_Meta = NULL;


