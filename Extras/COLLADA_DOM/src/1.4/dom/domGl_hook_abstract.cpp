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
#include <dom/domGl_hook_abstract.h>

daeElementRef
domGl_hook_abstract::create(daeInt bytes)
{
	domGl_hook_abstractRef ref = new(bytes) domGl_hook_abstract;
	return ref;
}


daeMetaElement *
domGl_hook_abstract::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "gl_hook_abstract" );
	_Meta->setStaticPointerAddress(&domGl_hook_abstract::_Meta);
	_Meta->registerConstructor(domGl_hook_abstract::create);

	_Meta->setIsAbstract( true );
	
	
	_Meta->setElementSize(sizeof(domGl_hook_abstract));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domGl_hook_abstract::_Meta = NULL;


