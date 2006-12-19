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
#include <dom/domProfile_GLES.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domProfile_GLES::create(daeInt bytes)
{
	domProfile_GLESRef ref = new(bytes) domProfile_GLES;
	return ref;
}


daeMetaElement *
domProfile_GLES::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "profile_GLES" );
	_Meta->registerClass(domProfile_GLES::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domProfile_GLES,elemAsset) );
	mea->setElementType( domAsset::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaChoice( _Meta, cm, 1, 0, -1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "image" );
	mea->setOffset( daeOffsetOf(domProfile_GLES,elemImage_array) );
	mea->setElementType( domImage::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "newparam" );
	mea->setOffset( daeOffsetOf(domProfile_GLES,elemNewparam_array) );
	mea->setElementType( domGles_newparam::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3002, 1, -1 );
	mea->setName( "technique" );
	mea->setOffset( daeOffsetOf(domProfile_GLES,elemTechnique_array) );
	mea->setElementType( domProfile_GLES::domTechnique::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3003, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domProfile_GLES,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 3003 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domProfile_GLES,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domProfile_GLES,_contentsOrder));


	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domProfile_GLES , attrId ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: platform
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "platform" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_GLES , attrPlatform ));
		ma->setContainer( _Meta );
		ma->setDefault( "PC");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_GLES));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_GLES::domTechnique::create(daeInt bytes)
{
	domProfile_GLES::domTechniqueRef ref = new(bytes) domProfile_GLES::domTechnique;
	return ref;
}


daeMetaElement *
domProfile_GLES::domTechnique::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "technique" );
	_Meta->registerClass(domProfile_GLES::domTechnique::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domProfile_GLES::domTechnique,elemAsset) );
	mea->setElementType( domAsset::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 1, 0, -1 );
	mea->setName( "annotate" );
	mea->setOffset( daeOffsetOf(domProfile_GLES::domTechnique,elemAnnotate_array) );
	mea->setElementType( domFx_annotate_common::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaChoice( _Meta, cm, 2, 0, -1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "image" );
	mea->setOffset( daeOffsetOf(domProfile_GLES::domTechnique,elemImage_array) );
	mea->setElementType( domImage::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "newparam" );
	mea->setOffset( daeOffsetOf(domProfile_GLES::domTechnique,elemNewparam_array) );
	mea->setElementType( domGles_newparam::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "setparam" );
	mea->setOffset( daeOffsetOf(domProfile_GLES::domTechnique,elemSetparam_array) );
	mea->setElementType( domProfile_GLES::domTechnique::domSetparam::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3003, 1, -1 );
	mea->setName( "pass" );
	mea->setOffset( daeOffsetOf(domProfile_GLES::domTechnique,elemPass_array) );
	mea->setElementType( domProfile_GLES::domTechnique::domPass::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3004, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domProfile_GLES::domTechnique,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 3004 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domProfile_GLES::domTechnique,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domProfile_GLES::domTechnique,_contentsOrder));


	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domProfile_GLES::domTechnique , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_GLES::domTechnique , attrSid ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_GLES::domTechnique));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_GLES::domTechnique::domSetparam::create(daeInt bytes)
{
	domProfile_GLES::domTechnique::domSetparamRef ref = new(bytes) domProfile_GLES::domTechnique::domSetparam;
	return ref;
}


daeMetaElement *
domProfile_GLES::domTechnique::domSetparam::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "setparam" );
	_Meta->registerClass(domProfile_GLES::domTechnique::domSetparam::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 0, -1 );
	mea->setName( "annotate" );
	mea->setOffset( daeOffsetOf(domProfile_GLES::domTechnique::domSetparam,elemAnnotate_array) );
	mea->setElementType( domFx_annotate_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 1, 1 );
	mea->setName( "gles_basic_type_common" );
	mea->setOffset( daeOffsetOf(domProfile_GLES::domTechnique::domSetparam,elemGles_basic_type_common) );
	mea->setElementType( domGles_basic_type_common::registerElement() );
	cm->appendChild( new daeMetaGroup( mea, _Meta, cm, 1, 1, 1 ) );
	
	cm->setMaxOrdinal( 1 );
	_Meta->setCMRoot( cm );	

	//	Add attribute: ref
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_GLES::domTechnique::domSetparam , attrRef ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_GLES::domTechnique::domSetparam));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_GLES::domTechnique::domPass::create(daeInt bytes)
{
	domProfile_GLES::domTechnique::domPassRef ref = new(bytes) domProfile_GLES::domTechnique::domPass;
	return ref;
}


daeMetaElement *
domProfile_GLES::domTechnique::domPass::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "pass" );
	_Meta->registerClass(domProfile_GLES::domTechnique::domPass::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 0, -1 );
	mea->setName( "annotate" );
	mea->setOffset( daeOffsetOf(domProfile_GLES::domTechnique::domPass,elemAnnotate_array) );
	mea->setElementType( domFx_annotate_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "color_target" );
	mea->setOffset( daeOffsetOf(domProfile_GLES::domTechnique::domPass,elemColor_target) );
	mea->setElementType( domProfile_GLES::domTechnique::domPass::domColor_target::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "depth_target" );
	mea->setOffset( daeOffsetOf(domProfile_GLES::domTechnique::domPass,elemDepth_target) );
	mea->setElementType( domProfile_GLES::domTechnique::domPass::domDepth_target::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 3, 0, 1 );
	mea->setName( "stencil_target" );
	mea->setOffset( daeOffsetOf(domProfile_GLES::domTechnique::domPass,elemStencil_target) );
	mea->setElementType( domProfile_GLES::domTechnique::domPass::domStencil_target::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 4, 0, 1 );
	mea->setName( "color_clear" );
	mea->setOffset( daeOffsetOf(domProfile_GLES::domTechnique::domPass,elemColor_clear) );
	mea->setElementType( domProfile_GLES::domTechnique::domPass::domColor_clear::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 5, 0, 1 );
	mea->setName( "depth_clear" );
	mea->setOffset( daeOffsetOf(domProfile_GLES::domTechnique::domPass,elemDepth_clear) );
	mea->setElementType( domProfile_GLES::domTechnique::domPass::domDepth_clear::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 6, 0, 1 );
	mea->setName( "stencil_clear" );
	mea->setOffset( daeOffsetOf(domProfile_GLES::domTechnique::domPass,elemStencil_clear) );
	mea->setElementType( domProfile_GLES::domTechnique::domPass::domStencil_clear::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 7, 0, 1 );
	mea->setName( "draw" );
	mea->setOffset( daeOffsetOf(domProfile_GLES::domTechnique::domPass,elemDraw) );
	mea->setElementType( domProfile_GLES::domTechnique::domPass::domDraw::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaChoice( _Meta, cm, 8, 0, -1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "gles_pipeline_settings" );
	mea->setOffset( daeOffsetOf(domProfile_GLES::domTechnique::domPass,elemGles_pipeline_settings_array) );
	mea->setElementType( domGles_pipeline_settings::registerElement() );
	cm->appendChild( new daeMetaGroup( mea, _Meta, cm, 0, 1, 1 ) );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3009, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domProfile_GLES::domTechnique::domPass,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 3009 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domProfile_GLES::domTechnique::domPass,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domProfile_GLES::domTechnique::domPass,_contentsOrder));


	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_GLES::domTechnique::domPass , attrSid ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_GLES::domTechnique::domPass));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_GLES::domTechnique::domPass::domColor_target::create(daeInt bytes)
{
	domProfile_GLES::domTechnique::domPass::domColor_targetRef ref = new(bytes) domProfile_GLES::domTechnique::domPass::domColor_target;
	return ref;
}


daeMetaElement *
domProfile_GLES::domTechnique::domPass::domColor_target::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "color_target" );
	_Meta->registerClass(domProfile_GLES::domTechnique::domPass::domColor_target::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Gles_rendertarget_common"));
		ma->setOffset( daeOffsetOf( domProfile_GLES::domTechnique::domPass::domColor_target , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_GLES::domTechnique::domPass::domColor_target));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_GLES::domTechnique::domPass::domDepth_target::create(daeInt bytes)
{
	domProfile_GLES::domTechnique::domPass::domDepth_targetRef ref = new(bytes) domProfile_GLES::domTechnique::domPass::domDepth_target;
	return ref;
}


daeMetaElement *
domProfile_GLES::domTechnique::domPass::domDepth_target::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "depth_target" );
	_Meta->registerClass(domProfile_GLES::domTechnique::domPass::domDepth_target::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Gles_rendertarget_common"));
		ma->setOffset( daeOffsetOf( domProfile_GLES::domTechnique::domPass::domDepth_target , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_GLES::domTechnique::domPass::domDepth_target));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_GLES::domTechnique::domPass::domStencil_target::create(daeInt bytes)
{
	domProfile_GLES::domTechnique::domPass::domStencil_targetRef ref = new(bytes) domProfile_GLES::domTechnique::domPass::domStencil_target;
	return ref;
}


daeMetaElement *
domProfile_GLES::domTechnique::domPass::domStencil_target::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "stencil_target" );
	_Meta->registerClass(domProfile_GLES::domTechnique::domPass::domStencil_target::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Gles_rendertarget_common"));
		ma->setOffset( daeOffsetOf( domProfile_GLES::domTechnique::domPass::domStencil_target , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_GLES::domTechnique::domPass::domStencil_target));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_GLES::domTechnique::domPass::domColor_clear::create(daeInt bytes)
{
	domProfile_GLES::domTechnique::domPass::domColor_clearRef ref = new(bytes) domProfile_GLES::domTechnique::domPass::domColor_clear;
	return ref;
}


daeMetaElement *
domProfile_GLES::domTechnique::domPass::domColor_clear::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "color_clear" );
	_Meta->registerClass(domProfile_GLES::domTechnique::domPass::domColor_clear::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_color_common"));
		ma->setOffset( daeOffsetOf( domProfile_GLES::domTechnique::domPass::domColor_clear , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_GLES::domTechnique::domPass::domColor_clear));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_GLES::domTechnique::domPass::domDepth_clear::create(daeInt bytes)
{
	domProfile_GLES::domTechnique::domPass::domDepth_clearRef ref = new(bytes) domProfile_GLES::domTechnique::domPass::domDepth_clear;
	return ref;
}


daeMetaElement *
domProfile_GLES::domTechnique::domPass::domDepth_clear::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "depth_clear" );
	_Meta->registerClass(domProfile_GLES::domTechnique::domPass::domDepth_clear::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domProfile_GLES::domTechnique::domPass::domDepth_clear , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_GLES::domTechnique::domPass::domDepth_clear));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_GLES::domTechnique::domPass::domStencil_clear::create(daeInt bytes)
{
	domProfile_GLES::domTechnique::domPass::domStencil_clearRef ref = new(bytes) domProfile_GLES::domTechnique::domPass::domStencil_clear;
	return ref;
}


daeMetaElement *
domProfile_GLES::domTechnique::domPass::domStencil_clear::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "stencil_clear" );
	_Meta->registerClass(domProfile_GLES::domTechnique::domPass::domStencil_clear::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsByte"));
		ma->setOffset( daeOffsetOf( domProfile_GLES::domTechnique::domPass::domStencil_clear , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_GLES::domTechnique::domPass::domStencil_clear));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_GLES::domTechnique::domPass::domDraw::create(daeInt bytes)
{
	domProfile_GLES::domTechnique::domPass::domDrawRef ref = new(bytes) domProfile_GLES::domTechnique::domPass::domDraw;
	return ref;
}


daeMetaElement *
domProfile_GLES::domTechnique::domPass::domDraw::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "draw" );
	_Meta->registerClass(domProfile_GLES::domTechnique::domPass::domDraw::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_draw_common"));
		ma->setOffset( daeOffsetOf( domProfile_GLES::domTechnique::domPass::domDraw , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_GLES::domTechnique::domPass::domDraw));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domProfile_GLES::_Meta = NULL;
daeMetaElement * domProfile_GLES::domTechnique::_Meta = NULL;
daeMetaElement * domProfile_GLES::domTechnique::domSetparam::_Meta = NULL;
daeMetaElement * domProfile_GLES::domTechnique::domPass::_Meta = NULL;
daeMetaElement * domProfile_GLES::domTechnique::domPass::domColor_target::_Meta = NULL;
daeMetaElement * domProfile_GLES::domTechnique::domPass::domDepth_target::_Meta = NULL;
daeMetaElement * domProfile_GLES::domTechnique::domPass::domStencil_target::_Meta = NULL;
daeMetaElement * domProfile_GLES::domTechnique::domPass::domColor_clear::_Meta = NULL;
daeMetaElement * domProfile_GLES::domTechnique::domPass::domDepth_clear::_Meta = NULL;
daeMetaElement * domProfile_GLES::domTechnique::domPass::domStencil_clear::_Meta = NULL;
daeMetaElement * domProfile_GLES::domTechnique::domPass::domDraw::_Meta = NULL;


