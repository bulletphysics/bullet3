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
	_Meta->setStaticPointerAddress(&domRigid_constraint::_Meta);
	_Meta->registerConstructor(domRigid_constraint::create);

	// Add elements: ref_attachment, attachment, technique_common, technique, extra
    _Meta->appendElement(domRigid_constraint::domRef_attachment::registerElement(),daeOffsetOf(domRigid_constraint,elemRef_attachment));
    _Meta->appendElement(domRigid_constraint::domAttachment::registerElement(),daeOffsetOf(domRigid_constraint,elemAttachment));
    _Meta->appendElement(domRigid_constraint::domTechnique_common::registerElement(),daeOffsetOf(domRigid_constraint,elemTechnique_common));
    _Meta->appendArrayElement(domTechnique::registerElement(),daeOffsetOf(domRigid_constraint,elemTechnique_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domRigid_constraint,elemExtra_array));

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
	_Meta->setStaticPointerAddress(&domRigid_constraint::domRef_attachment::_Meta);
	_Meta->registerConstructor(domRigid_constraint::domRef_attachment::create);

	// Add elements: translate, rotate, extra
    _Meta->appendArrayElement(domTranslate::registerElement(),daeOffsetOf(domRigid_constraint::domRef_attachment,elemTranslate_array));
    _Meta->appendArrayElement(domRotate::registerElement(),daeOffsetOf(domRigid_constraint::domRef_attachment,elemRotate_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domRigid_constraint::domRef_attachment,elemExtra_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domRigid_constraint::domRef_attachment,_contents));


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
	_Meta->setStaticPointerAddress(&domRigid_constraint::domAttachment::_Meta);
	_Meta->registerConstructor(domRigid_constraint::domAttachment::create);

	// Add elements: translate, rotate, extra
    _Meta->appendArrayElement(domTranslate::registerElement(),daeOffsetOf(domRigid_constraint::domAttachment,elemTranslate_array));
    _Meta->appendArrayElement(domRotate::registerElement(),daeOffsetOf(domRigid_constraint::domAttachment,elemRotate_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domRigid_constraint::domAttachment,elemExtra_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domRigid_constraint::domAttachment,_contents));


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
	_Meta->setStaticPointerAddress(&domRigid_constraint::domTechnique_common::_Meta);
	_Meta->registerConstructor(domRigid_constraint::domTechnique_common::create);

	// Add elements: enabled, interpenetrate, limits, spring
    _Meta->appendElement(domRigid_constraint::domTechnique_common::domEnabled::registerElement(),daeOffsetOf(domRigid_constraint::domTechnique_common,elemEnabled));
    _Meta->appendElement(domRigid_constraint::domTechnique_common::domInterpenetrate::registerElement(),daeOffsetOf(domRigid_constraint::domTechnique_common,elemInterpenetrate));
    _Meta->appendElement(domRigid_constraint::domTechnique_common::domLimits::registerElement(),daeOffsetOf(domRigid_constraint::domTechnique_common,elemLimits));
    _Meta->appendElement(domRigid_constraint::domTechnique_common::domSpring::registerElement(),daeOffsetOf(domRigid_constraint::domTechnique_common,elemSpring));
	
	
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
	_Meta->setStaticPointerAddress(&domRigid_constraint::domTechnique_common::domEnabled::_Meta);
	_Meta->registerConstructor(domRigid_constraint::domTechnique_common::domEnabled::create);

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
	_Meta->setStaticPointerAddress(&domRigid_constraint::domTechnique_common::domInterpenetrate::_Meta);
	_Meta->registerConstructor(domRigid_constraint::domTechnique_common::domInterpenetrate::create);

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
	_Meta->setStaticPointerAddress(&domRigid_constraint::domTechnique_common::domLimits::_Meta);
	_Meta->registerConstructor(domRigid_constraint::domTechnique_common::domLimits::create);

	// Add elements: swing_cone_and_twist, linear
    _Meta->appendElement(domRigid_constraint::domTechnique_common::domLimits::domSwing_cone_and_twist::registerElement(),daeOffsetOf(domRigid_constraint::domTechnique_common::domLimits,elemSwing_cone_and_twist));
    _Meta->appendElement(domRigid_constraint::domTechnique_common::domLimits::domLinear::registerElement(),daeOffsetOf(domRigid_constraint::domTechnique_common::domLimits,elemLinear));
	
	
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
	_Meta->setStaticPointerAddress(&domRigid_constraint::domTechnique_common::domLimits::domSwing_cone_and_twist::_Meta);
	_Meta->registerConstructor(domRigid_constraint::domTechnique_common::domLimits::domSwing_cone_and_twist::create);

	// Add elements: min, max
    _Meta->appendElement(domTargetableFloat3::registerElement(),daeOffsetOf(domRigid_constraint::domTechnique_common::domLimits::domSwing_cone_and_twist,elemMin),"min"); 
    _Meta->appendElement(domTargetableFloat3::registerElement(),daeOffsetOf(domRigid_constraint::domTechnique_common::domLimits::domSwing_cone_and_twist,elemMax),"max"); 
	
	
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
	_Meta->setStaticPointerAddress(&domRigid_constraint::domTechnique_common::domLimits::domLinear::_Meta);
	_Meta->registerConstructor(domRigid_constraint::domTechnique_common::domLimits::domLinear::create);

	// Add elements: min, max
    _Meta->appendElement(domTargetableFloat3::registerElement(),daeOffsetOf(domRigid_constraint::domTechnique_common::domLimits::domLinear,elemMin),"min"); 
    _Meta->appendElement(domTargetableFloat3::registerElement(),daeOffsetOf(domRigid_constraint::domTechnique_common::domLimits::domLinear,elemMax),"max"); 
	
	
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
	_Meta->setStaticPointerAddress(&domRigid_constraint::domTechnique_common::domSpring::_Meta);
	_Meta->registerConstructor(domRigid_constraint::domTechnique_common::domSpring::create);

	// Add elements: angular, linear
    _Meta->appendElement(domRigid_constraint::domTechnique_common::domSpring::domAngular::registerElement(),daeOffsetOf(domRigid_constraint::domTechnique_common::domSpring,elemAngular));
    _Meta->appendElement(domRigid_constraint::domTechnique_common::domSpring::domLinear::registerElement(),daeOffsetOf(domRigid_constraint::domTechnique_common::domSpring,elemLinear));
	
	
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
	_Meta->setStaticPointerAddress(&domRigid_constraint::domTechnique_common::domSpring::domAngular::_Meta);
	_Meta->registerConstructor(domRigid_constraint::domTechnique_common::domSpring::domAngular::create);

	// Add elements: stiffness, damping, target_value
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domRigid_constraint::domTechnique_common::domSpring::domAngular,elemStiffness),"stiffness"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domRigid_constraint::domTechnique_common::domSpring::domAngular,elemDamping),"damping"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domRigid_constraint::domTechnique_common::domSpring::domAngular,elemTarget_value),"target_value"); 
	
	
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
	_Meta->setStaticPointerAddress(&domRigid_constraint::domTechnique_common::domSpring::domLinear::_Meta);
	_Meta->registerConstructor(domRigid_constraint::domTechnique_common::domSpring::domLinear::create);

	// Add elements: stiffness, damping, target_value
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domRigid_constraint::domTechnique_common::domSpring::domLinear,elemStiffness),"stiffness"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domRigid_constraint::domTechnique_common::domSpring::domLinear,elemDamping),"damping"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domRigid_constraint::domTechnique_common::domSpring::domLinear,elemTarget_value),"target_value"); 
	
	
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


