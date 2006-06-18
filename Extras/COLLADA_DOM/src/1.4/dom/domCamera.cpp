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
#include <dom/domCamera.h>

daeElementRef
domCamera::create(daeInt bytes)
{
	domCameraRef ref = new(bytes) domCamera;
	return ref;
}


daeMetaElement *
domCamera::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "camera" );
	_Meta->setStaticPointerAddress(&domCamera::_Meta);
	_Meta->registerConstructor(domCamera::create);

	// Add elements: asset, optics, imager, extra
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domCamera,elemAsset));
    _Meta->appendElement(domCamera::domOptics::registerElement(),daeOffsetOf(domCamera,elemOptics));
    _Meta->appendElement(domCamera::domImager::registerElement(),daeOffsetOf(domCamera,elemImager));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domCamera,elemExtra_array));

	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domCamera , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domCamera , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCamera));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCamera::domOptics::create(daeInt bytes)
{
	domCamera::domOpticsRef ref = new(bytes) domCamera::domOptics;
	return ref;
}


daeMetaElement *
domCamera::domOptics::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "optics" );
	_Meta->setStaticPointerAddress(&domCamera::domOptics::_Meta);
	_Meta->registerConstructor(domCamera::domOptics::create);

	// Add elements: technique_common, technique, extra
    _Meta->appendElement(domCamera::domOptics::domTechnique_common::registerElement(),daeOffsetOf(domCamera::domOptics,elemTechnique_common));
    _Meta->appendArrayElement(domTechnique::registerElement(),daeOffsetOf(domCamera::domOptics,elemTechnique_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domCamera::domOptics,elemExtra_array));
	
	
	_Meta->setElementSize(sizeof(domCamera::domOptics));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCamera::domOptics::domTechnique_common::create(daeInt bytes)
{
	domCamera::domOptics::domTechnique_commonRef ref = new(bytes) domCamera::domOptics::domTechnique_common;
	return ref;
}


daeMetaElement *
domCamera::domOptics::domTechnique_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "technique_common" );
	_Meta->setStaticPointerAddress(&domCamera::domOptics::domTechnique_common::_Meta);
	_Meta->registerConstructor(domCamera::domOptics::domTechnique_common::create);

	// Add elements: orthographic, perspective
    _Meta->appendElement(domCamera::domOptics::domTechnique_common::domOrthographic::registerElement(),daeOffsetOf(domCamera::domOptics::domTechnique_common,elemOrthographic));
    _Meta->appendElement(domCamera::domOptics::domTechnique_common::domPerspective::registerElement(),daeOffsetOf(domCamera::domOptics::domTechnique_common,elemPerspective));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domCamera::domOptics::domTechnique_common,_contents));

	
	
	_Meta->setElementSize(sizeof(domCamera::domOptics::domTechnique_common));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCamera::domOptics::domTechnique_common::domOrthographic::create(daeInt bytes)
{
	domCamera::domOptics::domTechnique_common::domOrthographicRef ref = new(bytes) domCamera::domOptics::domTechnique_common::domOrthographic;
	return ref;
}


daeMetaElement *
domCamera::domOptics::domTechnique_common::domOrthographic::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "orthographic" );
	_Meta->setStaticPointerAddress(&domCamera::domOptics::domTechnique_common::domOrthographic::_Meta);
	_Meta->registerConstructor(domCamera::domOptics::domTechnique_common::domOrthographic::create);

	// Add elements: xmag, ymag, aspect_ratio, znear, zfar
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domCamera::domOptics::domTechnique_common::domOrthographic,elemXmag),"xmag"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domCamera::domOptics::domTechnique_common::domOrthographic,elemYmag),"ymag"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domCamera::domOptics::domTechnique_common::domOrthographic,elemAspect_ratio),"aspect_ratio"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domCamera::domOptics::domTechnique_common::domOrthographic,elemZnear),"znear"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domCamera::domOptics::domTechnique_common::domOrthographic,elemZfar),"zfar"); 
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domCamera::domOptics::domTechnique_common::domOrthographic,_contents));

	
	
	_Meta->setElementSize(sizeof(domCamera::domOptics::domTechnique_common::domOrthographic));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCamera::domOptics::domTechnique_common::domPerspective::create(daeInt bytes)
{
	domCamera::domOptics::domTechnique_common::domPerspectiveRef ref = new(bytes) domCamera::domOptics::domTechnique_common::domPerspective;
	return ref;
}


daeMetaElement *
domCamera::domOptics::domTechnique_common::domPerspective::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "perspective" );
	_Meta->setStaticPointerAddress(&domCamera::domOptics::domTechnique_common::domPerspective::_Meta);
	_Meta->registerConstructor(domCamera::domOptics::domTechnique_common::domPerspective::create);

	// Add elements: xfov, yfov, aspect_ratio, znear, zfar
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domCamera::domOptics::domTechnique_common::domPerspective,elemXfov),"xfov"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domCamera::domOptics::domTechnique_common::domPerspective,elemYfov),"yfov"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domCamera::domOptics::domTechnique_common::domPerspective,elemAspect_ratio),"aspect_ratio"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domCamera::domOptics::domTechnique_common::domPerspective,elemZnear),"znear"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domCamera::domOptics::domTechnique_common::domPerspective,elemZfar),"zfar"); 
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domCamera::domOptics::domTechnique_common::domPerspective,_contents));

	
	
	_Meta->setElementSize(sizeof(domCamera::domOptics::domTechnique_common::domPerspective));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCamera::domImager::create(daeInt bytes)
{
	domCamera::domImagerRef ref = new(bytes) domCamera::domImager;
	return ref;
}


daeMetaElement *
domCamera::domImager::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "imager" );
	_Meta->setStaticPointerAddress(&domCamera::domImager::_Meta);
	_Meta->registerConstructor(domCamera::domImager::create);

	// Add elements: technique, extra
    _Meta->appendArrayElement(domTechnique::registerElement(),daeOffsetOf(domCamera::domImager,elemTechnique_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domCamera::domImager,elemExtra_array));
	
	
	_Meta->setElementSize(sizeof(domCamera::domImager));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domCamera::_Meta = NULL;
daeMetaElement * domCamera::domOptics::_Meta = NULL;
daeMetaElement * domCamera::domOptics::domTechnique_common::_Meta = NULL;
daeMetaElement * domCamera::domOptics::domTechnique_common::domOrthographic::_Meta = NULL;
daeMetaElement * domCamera::domOptics::domTechnique_common::domPerspective::_Meta = NULL;
daeMetaElement * domCamera::domImager::_Meta = NULL;


