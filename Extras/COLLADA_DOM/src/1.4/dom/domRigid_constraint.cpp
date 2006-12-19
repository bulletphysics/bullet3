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
#include <dom/domRigid_constraint.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domRigid_constraint::create(daeInt bytes)
{
	domRigid_constraintRef ref = new(bytes) domRigid_constraint;
	return ref;
}


daeMetaElement *
domRigid_constraint::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "rigid_constraint" );
	_Meta->registerClass(domRigid_constraint::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "ref_attachment" );
	mea->setOffset( daeOffsetOf(domRigid_constraint,elemRef_attachment) );
	mea->setElementType( domRigid_constraint::domRef_attachment::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 1, 1 );
	mea->setName( "attachment" );
	mea->setOffset( daeOffsetOf(domRigid_constraint,elemAttachment) );
	mea->setElementType( domRigid_constraint::domAttachment::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 1, 1 );
	mea->setName( "technique_common" );
	mea->setOffset( daeOffsetOf(domRigid_constraint,elemTechnique_common) );
	mea->setElementType( domRigid_constraint::domTechnique_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3, 0, -1 );
	mea->setName( "technique" );
	mea->setOffset( daeOffsetOf(domRigid_constraint,elemTechnique_array) );
	mea->setElementType( domTechnique::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 4, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domRigid_constraint,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 4 );
	_Meta->setCMRoot( cm );	

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domRigid_constraint , attrSid ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domRigid_constraint , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domRigid_constraint));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domRigid_constraint::domRef_attachment::create(daeInt bytes)
{
	domRigid_constraint::domRef_attachmentRef ref = new(bytes) domRigid_constraint::domRef_attachment;
	ref->attrRigid_body.setContainer( (domRigid_constraint::domRef_attachment*)ref );
	return ref;
}


daeMetaElement *
domRigid_constraint::domRef_attachment::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "ref_attachment" );
	_Meta->registerClass(domRigid_constraint::domRef_attachment::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( _Meta, cm, 0, 0, -1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "translate" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domRef_attachment,elemTranslate_array) );
	mea->setElementType( domTranslate::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "rotate" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domRef_attachment,elemRotate_array) );
	mea->setElementType( domRotate::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domRef_attachment,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 3000 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domRigid_constraint::domRef_attachment,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domRigid_constraint::domRef_attachment,_contentsOrder));


	//	Add attribute: rigid_body
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "rigid_body" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domRigid_constraint::domRef_attachment , attrRigid_body ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domRigid_constraint::domRef_attachment));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domRigid_constraint::domAttachment::create(daeInt bytes)
{
	domRigid_constraint::domAttachmentRef ref = new(bytes) domRigid_constraint::domAttachment;
	ref->attrRigid_body.setContainer( (domRigid_constraint::domAttachment*)ref );
	return ref;
}


daeMetaElement *
domRigid_constraint::domAttachment::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "attachment" );
	_Meta->registerClass(domRigid_constraint::domAttachment::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( _Meta, cm, 0, 0, -1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "translate" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domAttachment,elemTranslate_array) );
	mea->setElementType( domTranslate::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "rotate" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domAttachment,elemRotate_array) );
	mea->setElementType( domRotate::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domAttachment,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 3000 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domRigid_constraint::domAttachment,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domRigid_constraint::domAttachment,_contentsOrder));


	//	Add attribute: rigid_body
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "rigid_body" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domRigid_constraint::domAttachment , attrRigid_body ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domRigid_constraint::domAttachment));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domRigid_constraint::domTechnique_common::create(daeInt bytes)
{
	domRigid_constraint::domTechnique_commonRef ref = new(bytes) domRigid_constraint::domTechnique_common;
	return ref;
}


daeMetaElement *
domRigid_constraint::domTechnique_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "technique_common" );
	_Meta->registerClass(domRigid_constraint::domTechnique_common::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "enabled" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domTechnique_common,elemEnabled) );
	mea->setElementType( domRigid_constraint::domTechnique_common::domEnabled::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "interpenetrate" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domTechnique_common,elemInterpenetrate) );
	mea->setElementType( domRigid_constraint::domTechnique_common::domInterpenetrate::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "limits" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domTechnique_common,elemLimits) );
	mea->setElementType( domRigid_constraint::domTechnique_common::domLimits::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 3, 0, 1 );
	mea->setName( "spring" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domTechnique_common,elemSpring) );
	mea->setElementType( domRigid_constraint::domTechnique_common::domSpring::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 3 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domRigid_constraint::domTechnique_common));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domRigid_constraint::domTechnique_common::domEnabled::create(daeInt bytes)
{
	domRigid_constraint::domTechnique_common::domEnabledRef ref = new(bytes) domRigid_constraint::domTechnique_common::domEnabled;
	return ref;
}


daeMetaElement *
domRigid_constraint::domTechnique_common::domEnabled::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "enabled" );
	_Meta->registerClass(domRigid_constraint::domTechnique_common::domEnabled::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domRigid_constraint::domTechnique_common::domEnabled , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domRigid_constraint::domTechnique_common::domEnabled , attrSid ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domRigid_constraint::domTechnique_common::domEnabled));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domRigid_constraint::domTechnique_common::domInterpenetrate::create(daeInt bytes)
{
	domRigid_constraint::domTechnique_common::domInterpenetrateRef ref = new(bytes) domRigid_constraint::domTechnique_common::domInterpenetrate;
	return ref;
}


daeMetaElement *
domRigid_constraint::domTechnique_common::domInterpenetrate::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "interpenetrate" );
	_Meta->registerClass(domRigid_constraint::domTechnique_common::domInterpenetrate::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domRigid_constraint::domTechnique_common::domInterpenetrate , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domRigid_constraint::domTechnique_common::domInterpenetrate , attrSid ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domRigid_constraint::domTechnique_common::domInterpenetrate));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domRigid_constraint::domTechnique_common::domLimits::create(daeInt bytes)
{
	domRigid_constraint::domTechnique_common::domLimitsRef ref = new(bytes) domRigid_constraint::domTechnique_common::domLimits;
	return ref;
}


daeMetaElement *
domRigid_constraint::domTechnique_common::domLimits::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "limits" );
	_Meta->registerClass(domRigid_constraint::domTechnique_common::domLimits::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "swing_cone_and_twist" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domTechnique_common::domLimits,elemSwing_cone_and_twist) );
	mea->setElementType( domRigid_constraint::domTechnique_common::domLimits::domSwing_cone_and_twist::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "linear" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domTechnique_common::domLimits,elemLinear) );
	mea->setElementType( domRigid_constraint::domTechnique_common::domLimits::domLinear::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 1 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domRigid_constraint::domTechnique_common::domLimits));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domRigid_constraint::domTechnique_common::domLimits::domSwing_cone_and_twist::create(daeInt bytes)
{
	domRigid_constraint::domTechnique_common::domLimits::domSwing_cone_and_twistRef ref = new(bytes) domRigid_constraint::domTechnique_common::domLimits::domSwing_cone_and_twist;
	return ref;
}


daeMetaElement *
domRigid_constraint::domTechnique_common::domLimits::domSwing_cone_and_twist::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "swing_cone_and_twist" );
	_Meta->registerClass(domRigid_constraint::domTechnique_common::domLimits::domSwing_cone_and_twist::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "min" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domTechnique_common::domLimits::domSwing_cone_and_twist,elemMin) );
	mea->setElementType( domTargetableFloat3::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "max" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domTechnique_common::domLimits::domSwing_cone_and_twist,elemMax) );
	mea->setElementType( domTargetableFloat3::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 1 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domRigid_constraint::domTechnique_common::domLimits::domSwing_cone_and_twist));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domRigid_constraint::domTechnique_common::domLimits::domLinear::create(daeInt bytes)
{
	domRigid_constraint::domTechnique_common::domLimits::domLinearRef ref = new(bytes) domRigid_constraint::domTechnique_common::domLimits::domLinear;
	return ref;
}


daeMetaElement *
domRigid_constraint::domTechnique_common::domLimits::domLinear::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "linear" );
	_Meta->registerClass(domRigid_constraint::domTechnique_common::domLimits::domLinear::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "min" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domTechnique_common::domLimits::domLinear,elemMin) );
	mea->setElementType( domTargetableFloat3::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "max" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domTechnique_common::domLimits::domLinear,elemMax) );
	mea->setElementType( domTargetableFloat3::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 1 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domRigid_constraint::domTechnique_common::domLimits::domLinear));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domRigid_constraint::domTechnique_common::domSpring::create(daeInt bytes)
{
	domRigid_constraint::domTechnique_common::domSpringRef ref = new(bytes) domRigid_constraint::domTechnique_common::domSpring;
	return ref;
}


daeMetaElement *
domRigid_constraint::domTechnique_common::domSpring::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "spring" );
	_Meta->registerClass(domRigid_constraint::domTechnique_common::domSpring::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "angular" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domTechnique_common::domSpring,elemAngular) );
	mea->setElementType( domRigid_constraint::domTechnique_common::domSpring::domAngular::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "linear" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domTechnique_common::domSpring,elemLinear) );
	mea->setElementType( domRigid_constraint::domTechnique_common::domSpring::domLinear::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 1 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domRigid_constraint::domTechnique_common::domSpring));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domRigid_constraint::domTechnique_common::domSpring::domAngular::create(daeInt bytes)
{
	domRigid_constraint::domTechnique_common::domSpring::domAngularRef ref = new(bytes) domRigid_constraint::domTechnique_common::domSpring::domAngular;
	return ref;
}


daeMetaElement *
domRigid_constraint::domTechnique_common::domSpring::domAngular::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "angular" );
	_Meta->registerClass(domRigid_constraint::domTechnique_common::domSpring::domAngular::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "stiffness" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domTechnique_common::domSpring::domAngular,elemStiffness) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "damping" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domTechnique_common::domSpring::domAngular,elemDamping) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "target_value" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domTechnique_common::domSpring::domAngular,elemTarget_value) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 2 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domRigid_constraint::domTechnique_common::domSpring::domAngular));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domRigid_constraint::domTechnique_common::domSpring::domLinear::create(daeInt bytes)
{
	domRigid_constraint::domTechnique_common::domSpring::domLinearRef ref = new(bytes) domRigid_constraint::domTechnique_common::domSpring::domLinear;
	return ref;
}


daeMetaElement *
domRigid_constraint::domTechnique_common::domSpring::domLinear::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "linear" );
	_Meta->registerClass(domRigid_constraint::domTechnique_common::domSpring::domLinear::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "stiffness" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domTechnique_common::domSpring::domLinear,elemStiffness) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "damping" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domTechnique_common::domSpring::domLinear,elemDamping) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "target_value" );
	mea->setOffset( daeOffsetOf(domRigid_constraint::domTechnique_common::domSpring::domLinear,elemTarget_value) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 2 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domRigid_constraint::domTechnique_common::domSpring::domLinear));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domRigid_constraint::_Meta = NULL;
daeMetaElement * domRigid_constraint::domRef_attachment::_Meta = NULL;
daeMetaElement * domRigid_constraint::domAttachment::_Meta = NULL;
daeMetaElement * domRigid_constraint::domTechnique_common::_Meta = NULL;
daeMetaElement * domRigid_constraint::domTechnique_common::domEnabled::_Meta = NULL;
daeMetaElement * domRigid_constraint::domTechnique_common::domInterpenetrate::_Meta = NULL;
daeMetaElement * domRigid_constraint::domTechnique_common::domLimits::_Meta = NULL;
daeMetaElement * domRigid_constraint::domTechnique_common::domLimits::domSwing_cone_and_twist::_Meta = NULL;
daeMetaElement * domRigid_constraint::domTechnique_common::domLimits::domLinear::_Meta = NULL;
daeMetaElement * domRigid_constraint::domTechnique_common::domSpring::_Meta = NULL;
daeMetaElement * domRigid_constraint::domTechnique_common::domSpring::domAngular::_Meta = NULL;
daeMetaElement * domRigid_constraint::domTechnique_common::domSpring::domLinear::_Meta = NULL;


