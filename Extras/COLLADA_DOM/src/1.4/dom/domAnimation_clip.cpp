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
#include <dom/domAnimation_clip.h>

daeElementRef
domAnimation_clip::create(daeInt bytes)
{
	domAnimation_clipRef ref = new(bytes) domAnimation_clip;
	return ref;
}


daeMetaElement *
domAnimation_clip::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "animation_clip" );
	_Meta->setStaticPointerAddress(&domAnimation_clip::_Meta);
	_Meta->registerConstructor(domAnimation_clip::create);

	// Add elements: asset, instance_animation, extra
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domAnimation_clip,elemAsset));
    _Meta->appendArrayElement(domInstanceWithExtra::registerElement(),daeOffsetOf(domAnimation_clip,elemInstance_animation_array),"instance_animation"); 
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domAnimation_clip,elemExtra_array));

	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domAnimation_clip , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domAnimation_clip , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: start
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "start" );
		ma->setType( daeAtomicType::get("xsDouble"));
		ma->setOffset( daeOffsetOf( domAnimation_clip , attrStart ));
		ma->setContainer( _Meta );
		ma->setDefault( "0.0");
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: end
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "end" );
		ma->setType( daeAtomicType::get("xsDouble"));
		ma->setOffset( daeOffsetOf( domAnimation_clip , attrEnd ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domAnimation_clip));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domAnimation_clip::_Meta = NULL;


