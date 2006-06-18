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
#include <dom/domFx_profile_abstract.h>

daeElementRef
domFx_profile_abstract::create(daeInt bytes)
{
	domFx_profile_abstractRef ref = new(bytes) domFx_profile_abstract;
	return ref;
}


daeMetaElement *
domFx_profile_abstract::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fx_profile_abstract" );
	_Meta->setStaticPointerAddress(&domFx_profile_abstract::_Meta);
	_Meta->registerConstructor(domFx_profile_abstract::create);

	_Meta->setIsAbstract( true );
	
	
	_Meta->setElementSize(sizeof(domFx_profile_abstract));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domFx_profile_abstract::_Meta = NULL;


