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
#include <dom/domCommon_float_or_param_type.h>

daeElementRef
domCommon_float_or_param_type::create(daeInt bytes)
{
	domCommon_float_or_param_typeRef ref = new(bytes) domCommon_float_or_param_type;
	return ref;
}


daeMetaElement *
domCommon_float_or_param_type::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "common_float_or_param_type" );
	_Meta->setStaticPointerAddress(&domCommon_float_or_param_type::_Meta);
	_Meta->registerConstructor(domCommon_float_or_param_type::create);

	// Add elements: float, param
    _Meta->appendElement(domCommon_float_or_param_type::domFloat::registerElement(),daeOffsetOf(domCommon_float_or_param_type,elemFloat));
    _Meta->appendElement(domCommon_float_or_param_type::domParam::registerElement(),daeOffsetOf(domCommon_float_or_param_type,elemParam));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domCommon_float_or_param_type,_contents));

	
	
	_Meta->setElementSize(sizeof(domCommon_float_or_param_type));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCommon_float_or_param_type::domFloat::create(daeInt bytes)
{
	domCommon_float_or_param_type::domFloatRef ref = new(bytes) domCommon_float_or_param_type::domFloat;
	return ref;
}


daeMetaElement *
domCommon_float_or_param_type::domFloat::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float" );
	_Meta->setStaticPointerAddress(&domCommon_float_or_param_type::domFloat::_Meta);
	_Meta->registerConstructor(domCommon_float_or_param_type::domFloat::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domCommon_float_or_param_type::domFloat , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domCommon_float_or_param_type::domFloat , attrSid ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCommon_float_or_param_type::domFloat));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCommon_float_or_param_type::domParam::create(daeInt bytes)
{
	domCommon_float_or_param_type::domParamRef ref = new(bytes) domCommon_float_or_param_type::domParam;
	return ref;
}


daeMetaElement *
domCommon_float_or_param_type::domParam::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "param" );
	_Meta->setStaticPointerAddress(&domCommon_float_or_param_type::domParam::_Meta);
	_Meta->registerConstructor(domCommon_float_or_param_type::domParam::create);


	//	Add attribute: ref
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domCommon_float_or_param_type::domParam , attrRef ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCommon_float_or_param_type::domParam));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domCommon_float_or_param_type::_Meta = NULL;
daeMetaElement * domCommon_float_or_param_type::domFloat::_Meta = NULL;
daeMetaElement * domCommon_float_or_param_type::domParam::_Meta = NULL;


