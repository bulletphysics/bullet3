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
#include <dom/domBind_material.h>

daeElementRef
domBind_material::create(daeInt bytes)
{
	domBind_materialRef ref = new(bytes) domBind_material;
	return ref;
}


daeMetaElement *
domBind_material::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bind_material" );
	_Meta->setStaticPointerAddress(&domBind_material::_Meta);
	_Meta->registerConstructor(domBind_material::create);

	// Add elements: param, technique_common, technique
    _Meta->appendArrayElement(domParam::registerElement(),daeOffsetOf(domBind_material,elemParam_array));
    _Meta->appendElement(domBind_material::domTechnique_common::registerElement(),daeOffsetOf(domBind_material,elemTechnique_common));
    _Meta->appendArrayElement(domTechnique::registerElement(),daeOffsetOf(domBind_material,elemTechnique_array));
	
	
	_Meta->setElementSize(sizeof(domBind_material));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domBind_material::domTechnique_common::create(daeInt bytes)
{
	domBind_material::domTechnique_commonRef ref = new(bytes) domBind_material::domTechnique_common;
	return ref;
}


daeMetaElement *
domBind_material::domTechnique_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "technique_common" );
	_Meta->setStaticPointerAddress(&domBind_material::domTechnique_common::_Meta);
	_Meta->registerConstructor(domBind_material::domTechnique_common::create);

	// Add elements: instance_material
    _Meta->appendArrayElement(domInstance_material::registerElement(),daeOffsetOf(domBind_material::domTechnique_common,elemInstance_material_array));
	
	
	_Meta->setElementSize(sizeof(domBind_material::domTechnique_common));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domBind_material::_Meta = NULL;
daeMetaElement * domBind_material::domTechnique_common::_Meta = NULL;


