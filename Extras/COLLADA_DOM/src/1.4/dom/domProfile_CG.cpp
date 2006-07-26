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
#include <dom/domProfile_CG.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domProfile_CG::create(daeInt bytes)
{
	domProfile_CGRef ref = new(bytes) domProfile_CG;
	return ref;
}


daeMetaElement *
domProfile_CG::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "profile_CG" );
	_Meta->registerConstructor(domProfile_CG::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domProfile_CG,elemAsset) );
	mea->setElementType( domAsset::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaChoice( _Meta, cm, 1, 0, -1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "code" );
	mea->setOffset( daeOffsetOf(domProfile_CG,elemCode_array) );
	mea->setElementType( domFx_code_profile::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "include" );
	mea->setOffset( daeOffsetOf(domProfile_CG,elemInclude_array) );
	mea->setElementType( domFx_include_common::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	cm = new daeMetaChoice( _Meta, cm, 3002, 0, -1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "image" );
	mea->setOffset( daeOffsetOf(domProfile_CG,elemImage_array) );
	mea->setElementType( domImage::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "newparam" );
	mea->setOffset( daeOffsetOf(domProfile_CG,elemNewparam_array) );
	mea->setElementType( domCg_newparam::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 6003, 1, -1 );
	mea->setName( "technique" );
	mea->setOffset( daeOffsetOf(domProfile_CG,elemTechnique_array) );
	mea->setElementType( domProfile_CG::domTechnique::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 6004, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domProfile_CG,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 6004 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domProfile_CG,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domProfile_CG,_contentsOrder));


	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domProfile_CG , attrId ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: platform
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "platform" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_CG , attrPlatform ));
		ma->setContainer( _Meta );
		ma->setDefault( "PC");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_CG));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_CG::domTechnique::create(daeInt bytes)
{
	domProfile_CG::domTechniqueRef ref = new(bytes) domProfile_CG::domTechnique;
	return ref;
}


daeMetaElement *
domProfile_CG::domTechnique::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "technique" );
	_Meta->registerConstructor(domProfile_CG::domTechnique::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique,elemAsset) );
	mea->setElementType( domAsset::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 1, 0, -1 );
	mea->setName( "annotate" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique,elemAnnotate_array) );
	mea->setElementType( domFx_annotate_common::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaChoice( _Meta, cm, 2, 0, -1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "code" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique,elemCode_array) );
	mea->setElementType( domFx_code_profile::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "include" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique,elemInclude_array) );
	mea->setElementType( domFx_include_common::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	cm = new daeMetaChoice( _Meta, cm, 3003, 0, -1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "image" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique,elemImage_array) );
	mea->setElementType( domImage::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "newparam" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique,elemNewparam_array) );
	mea->setElementType( domCg_newparam::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "setparam" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique,elemSetparam_array) );
	mea->setElementType( domCg_setparam::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 6004, 1, -1 );
	mea->setName( "pass" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique,elemPass_array) );
	mea->setElementType( domProfile_CG::domTechnique::domPass::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 6005, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 6005 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domProfile_CG::domTechnique,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domProfile_CG::domTechnique,_contentsOrder));


	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domProfile_CG::domTechnique , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_CG::domTechnique , attrSid ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_CG::domTechnique));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_CG::domTechnique::domPass::create(daeInt bytes)
{
	domProfile_CG::domTechnique::domPassRef ref = new(bytes) domProfile_CG::domTechnique::domPass;
	return ref;
}


daeMetaElement *
domProfile_CG::domTechnique::domPass::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "pass" );
	_Meta->registerConstructor(domProfile_CG::domTechnique::domPass::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 0, -1 );
	mea->setName( "annotate" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique::domPass,elemAnnotate_array) );
	mea->setElementType( domFx_annotate_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 1, 0, -1 );
	mea->setName( "color_target" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique::domPass,elemColor_target_array) );
	mea->setElementType( domFx_colortarget_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 2, 0, -1 );
	mea->setName( "depth_target" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique::domPass,elemDepth_target_array) );
	mea->setElementType( domFx_depthtarget_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3, 0, -1 );
	mea->setName( "stencil_target" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique::domPass,elemStencil_target_array) );
	mea->setElementType( domFx_stenciltarget_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 4, 0, -1 );
	mea->setName( "color_clear" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique::domPass,elemColor_clear_array) );
	mea->setElementType( domFx_clearcolor_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 5, 0, -1 );
	mea->setName( "depth_clear" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique::domPass,elemDepth_clear_array) );
	mea->setElementType( domFx_cleardepth_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 6, 0, -1 );
	mea->setName( "stencil_clear" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique::domPass,elemStencil_clear_array) );
	mea->setElementType( domFx_clearstencil_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 7, 0, 1 );
	mea->setName( "draw" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique::domPass,elemDraw) );
	mea->setElementType( domProfile_CG::domTechnique::domPass::domDraw::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaChoice( _Meta, cm, 8, 1, -1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "gl_pipeline_settings" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique::domPass,elemGl_pipeline_settings_array) );
	mea->setElementType( domGl_pipeline_settings::registerElement() );
	cm->appendChild( new daeMetaGroup( mea, _Meta, cm, 0, 1, 1 ) );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "shader" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique::domPass,elemShader_array) );
	mea->setElementType( domProfile_CG::domTechnique::domPass::domShader::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3009, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique::domPass,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 3009 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domProfile_CG::domTechnique::domPass,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domProfile_CG::domTechnique::domPass,_contentsOrder));


	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_CG::domTechnique::domPass , attrSid ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_CG::domTechnique::domPass));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_CG::domTechnique::domPass::domDraw::create(daeInt bytes)
{
	domProfile_CG::domTechnique::domPass::domDrawRef ref = new(bytes) domProfile_CG::domTechnique::domPass::domDraw;
	return ref;
}


daeMetaElement *
domProfile_CG::domTechnique::domPass::domDraw::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "draw" );
	_Meta->registerConstructor(domProfile_CG::domTechnique::domPass::domDraw::create);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_draw_common"));
		ma->setOffset( daeOffsetOf( domProfile_CG::domTechnique::domPass::domDraw , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_CG::domTechnique::domPass::domDraw));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_CG::domTechnique::domPass::domShader::create(daeInt bytes)
{
	domProfile_CG::domTechnique::domPass::domShaderRef ref = new(bytes) domProfile_CG::domTechnique::domPass::domShader;
	return ref;
}


daeMetaElement *
domProfile_CG::domTechnique::domPass::domShader::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "shader" );
	_Meta->registerConstructor(domProfile_CG::domTechnique::domPass::domShader::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 0, -1 );
	mea->setName( "annotate" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique::domPass::domShader,elemAnnotate_array) );
	mea->setElementType( domFx_annotate_common::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaSequence( _Meta, cm, 1, 0, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "compiler_target" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique::domPass::domShader,elemCompiler_target) );
	mea->setElementType( domProfile_CG::domTechnique::domPass::domShader::domCompiler_target::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "compiler_options" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique::domPass::domShader,elemCompiler_options) );
	mea->setElementType( domProfile_CG::domTechnique::domPass::domShader::domCompiler_options::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 1 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	mea = new daeMetaElementAttribute( _Meta, cm, 3, 1, 1 );
	mea->setName( "name" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique::domPass::domShader,elemName) );
	mea->setElementType( domProfile_CG::domTechnique::domPass::domShader::domName::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 4, 0, -1 );
	mea->setName( "bind" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique::domPass::domShader,elemBind_array) );
	mea->setElementType( domProfile_CG::domTechnique::domPass::domShader::domBind::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 4 );
	_Meta->setCMRoot( cm );	

	//	Add attribute: stage
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "stage" );
		ma->setType( daeAtomicType::get("Cg_pipeline_stage"));
		ma->setOffset( daeOffsetOf( domProfile_CG::domTechnique::domPass::domShader , attrStage ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_CG::domTechnique::domPass::domShader));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_CG::domTechnique::domPass::domShader::domCompiler_target::create(daeInt bytes)
{
	domProfile_CG::domTechnique::domPass::domShader::domCompiler_targetRef ref = new(bytes) domProfile_CG::domTechnique::domPass::domShader::domCompiler_target;
	return ref;
}


daeMetaElement *
domProfile_CG::domTechnique::domPass::domShader::domCompiler_target::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "compiler_target" );
	_Meta->registerConstructor(domProfile_CG::domTechnique::domPass::domShader::domCompiler_target::create);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNMTOKEN"));
		ma->setOffset( daeOffsetOf( domProfile_CG::domTechnique::domPass::domShader::domCompiler_target , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_CG::domTechnique::domPass::domShader::domCompiler_target));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_CG::domTechnique::domPass::domShader::domCompiler_options::create(daeInt bytes)
{
	domProfile_CG::domTechnique::domPass::domShader::domCompiler_optionsRef ref = new(bytes) domProfile_CG::domTechnique::domPass::domShader::domCompiler_options;
	return ref;
}


daeMetaElement *
domProfile_CG::domTechnique::domPass::domShader::domCompiler_options::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "compiler_options" );
	_Meta->registerConstructor(domProfile_CG::domTechnique::domPass::domShader::domCompiler_options::create);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsString"));
		ma->setOffset( daeOffsetOf( domProfile_CG::domTechnique::domPass::domShader::domCompiler_options , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_CG::domTechnique::domPass::domShader::domCompiler_options));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_CG::domTechnique::domPass::domShader::domName::create(daeInt bytes)
{
	domProfile_CG::domTechnique::domPass::domShader::domNameRef ref = new(bytes) domProfile_CG::domTechnique::domPass::domShader::domName;
	return ref;
}


daeMetaElement *
domProfile_CG::domTechnique::domPass::domShader::domName::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "name" );
	_Meta->registerConstructor(domProfile_CG::domTechnique::domPass::domShader::domName::create);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_CG::domTechnique::domPass::domShader::domName , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: source
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "source" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_CG::domTechnique::domPass::domShader::domName , attrSource ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_CG::domTechnique::domPass::domShader::domName));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_CG::domTechnique::domPass::domShader::domBind::create(daeInt bytes)
{
	domProfile_CG::domTechnique::domPass::domShader::domBindRef ref = new(bytes) domProfile_CG::domTechnique::domPass::domShader::domBind;
	return ref;
}


daeMetaElement *
domProfile_CG::domTechnique::domPass::domShader::domBind::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bind" );
	_Meta->registerConstructor(domProfile_CG::domTechnique::domPass::domShader::domBind::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "cg_param_type" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique::domPass::domShader::domBind,elemCg_param_type) );
	mea->setElementType( domCg_param_type::registerElement() );
	cm->appendChild( new daeMetaGroup( mea, _Meta, cm, 0, 1, 1 ) );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "param" );
	mea->setOffset( daeOffsetOf(domProfile_CG::domTechnique::domPass::domShader::domBind,elemParam) );
	mea->setElementType( domProfile_CG::domTechnique::domPass::domShader::domBind::domParam::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domProfile_CG::domTechnique::domPass::domShader::domBind,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domProfile_CG::domTechnique::domPass::domShader::domBind,_contentsOrder));


	//	Add attribute: symbol
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "symbol" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_CG::domTechnique::domPass::domShader::domBind , attrSymbol ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_CG::domTechnique::domPass::domShader::domBind));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_CG::domTechnique::domPass::domShader::domBind::domParam::create(daeInt bytes)
{
	domProfile_CG::domTechnique::domPass::domShader::domBind::domParamRef ref = new(bytes) domProfile_CG::domTechnique::domPass::domShader::domBind::domParam;
	return ref;
}


daeMetaElement *
domProfile_CG::domTechnique::domPass::domShader::domBind::domParam::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "param" );
	_Meta->registerConstructor(domProfile_CG::domTechnique::domPass::domShader::domBind::domParam::create);

	_Meta->setIsInnerClass( true );

	//	Add attribute: ref
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_CG::domTechnique::domPass::domShader::domBind::domParam , attrRef ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_CG::domTechnique::domPass::domShader::domBind::domParam));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domProfile_CG::_Meta = NULL;
daeMetaElement * domProfile_CG::domTechnique::_Meta = NULL;
daeMetaElement * domProfile_CG::domTechnique::domPass::_Meta = NULL;
daeMetaElement * domProfile_CG::domTechnique::domPass::domDraw::_Meta = NULL;
daeMetaElement * domProfile_CG::domTechnique::domPass::domShader::_Meta = NULL;
daeMetaElement * domProfile_CG::domTechnique::domPass::domShader::domCompiler_target::_Meta = NULL;
daeMetaElement * domProfile_CG::domTechnique::domPass::domShader::domCompiler_options::_Meta = NULL;
daeMetaElement * domProfile_CG::domTechnique::domPass::domShader::domName::_Meta = NULL;
daeMetaElement * domProfile_CG::domTechnique::domPass::domShader::domBind::_Meta = NULL;
daeMetaElement * domProfile_CG::domTechnique::domPass::domShader::domBind::domParam::_Meta = NULL;


