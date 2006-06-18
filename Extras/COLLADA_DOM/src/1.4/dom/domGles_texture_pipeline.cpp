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
#include <dom/domGles_texture_pipeline.h>

daeElementRef
domGles_texture_pipeline::create(daeInt bytes)
{
	domGles_texture_pipelineRef ref = new(bytes) domGles_texture_pipeline;
	return ref;
}


daeMetaElement *
domGles_texture_pipeline::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "gles_texture_pipeline" );
	_Meta->setStaticPointerAddress(&domGles_texture_pipeline::_Meta);
	_Meta->registerConstructor(domGles_texture_pipeline::create);

	// Add elements: texcombiner, texenv, extra
    _Meta->appendArrayElement(domGles_texcombiner_command_type::registerElement(),daeOffsetOf(domGles_texture_pipeline,elemTexcombiner_array),"texcombiner"); 
    _Meta->appendArrayElement(domGles_texenv_command_type::registerElement(),daeOffsetOf(domGles_texture_pipeline,elemTexenv_array),"texenv"); 
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domGles_texture_pipeline,elemExtra_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domGles_texture_pipeline,_contents));


	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_texture_pipeline , attrSid ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_texture_pipeline));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domGles_texture_pipeline::_Meta = NULL;


