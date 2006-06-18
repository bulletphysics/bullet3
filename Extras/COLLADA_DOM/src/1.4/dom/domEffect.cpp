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
#include <dom/domEffect.h>

daeElementRef
domEffect::create(daeInt bytes)
{
	domEffectRef ref = new(bytes) domEffect;
	return ref;
}

#include <dom/domProfile_GLSL.h>
#include <dom/domProfile_COMMON.h>
#include <dom/domProfile_CG.h>
#include <dom/domProfile_GLES.h>

daeMetaElement *
domEffect::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "effect" );
	_Meta->setStaticPointerAddress(&domEffect::_Meta);
	_Meta->registerConstructor(domEffect::create);

	// Add elements: asset, annotate, image, newparam, fx_profile_abstract, extra
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domEffect,elemAsset));
    _Meta->appendArrayElement(domFx_annotate_common::registerElement(),daeOffsetOf(domEffect,elemAnnotate_array),"annotate"); 
    _Meta->appendArrayElement(domImage::registerElement(),daeOffsetOf(domEffect,elemImage_array));
    _Meta->appendArrayElement(domFx_newparam_common::registerElement(),daeOffsetOf(domEffect,elemNewparam_array),"newparam"); 
    _Meta->appendArrayElement(domFx_profile_abstract::registerElement(),daeOffsetOf(domEffect,elemFx_profile_abstract_array));
    _Meta->appendArrayElement(domProfile_GLSL::registerElement(),daeOffsetOf(domEffect,elemFx_profile_abstract_array));
    _Meta->appendArrayElement(domProfile_COMMON::registerElement(),daeOffsetOf(domEffect,elemFx_profile_abstract_array));
    _Meta->appendArrayElement(domProfile_CG::registerElement(),daeOffsetOf(domEffect,elemFx_profile_abstract_array));
    _Meta->appendArrayElement(domProfile_GLES::registerElement(),daeOffsetOf(domEffect,elemFx_profile_abstract_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domEffect,elemExtra_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domEffect,_contents));


	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domEffect , attrId ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domEffect , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domEffect));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domEffect::_Meta = NULL;


