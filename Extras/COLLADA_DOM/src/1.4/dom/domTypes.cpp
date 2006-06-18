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

 
#include <dom/domTypes.h>
#include <dae/daeDom.h>
#include <dom/domCOLLADA.h>


void registerDomTypes()
{

	daeAtomicType* type = NULL;
	// TYPEDEF: Bool	//check if this type has an existing base
	type = daeAtomicType::get("xsBoolean");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Bool");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Bool");
	}
	
	// TYPEDEF: DateTime	//check if this type has an existing base
	type = daeAtomicType::get("xsDateTime");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("DateTime");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("DateTime");
	}
	
	// TYPEDEF: Float	//check if this type has an existing base
	type = daeAtomicType::get("xsDouble");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Float");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float");
	}
	
	// TYPEDEF: Int	//check if this type has an existing base
	type = daeAtomicType::get("xsLong");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Int");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Int");
	}
	
	// TYPEDEF: Name	//check if this type has an existing base
	type = daeAtomicType::get("xsName");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Name");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Name");
	}
	
	// TYPEDEF: String	//check if this type has an existing base
	type = daeAtomicType::get("xsString");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("String");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("String");
	}
	
	// TYPEDEF: Token	//check if this type has an existing base
	type = daeAtomicType::get("xsToken");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Token");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Token");
	}
	
	// TYPEDEF: Uint	//check if this type has an existing base
	type = daeAtomicType::get("xsUnsignedLong");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Uint");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Uint");
	}
	
	// TYPEDEF: ListOfBools	//check if this type has an existing base
	type = daeAtomicType::get("Bool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("ListOfBools");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("ListOfBools");
	}
	
	// TYPEDEF: ListOfFloats	//check if this type has an existing base
	type = daeAtomicType::get("Float");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("ListOfFloats");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("ListOfFloats");
	}
	
	// TYPEDEF: ListOfHexBinary	//check if this type has an existing base
	type = daeAtomicType::get("xsHexBinary");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("ListOfHexBinary");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("ListOfHexBinary");
	}
	
	// TYPEDEF: ListOfInts	//check if this type has an existing base
	type = daeAtomicType::get("Int");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("ListOfInts");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("ListOfInts");
	}
	
	// TYPEDEF: ListOfNames	//check if this type has an existing base
	type = daeAtomicType::get("Name");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("ListOfNames");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("ListOfNames");
	}
	
	// TYPEDEF: ListOfTokens	//check if this type has an existing base
	type = daeAtomicType::get("Token");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("ListOfTokens");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("ListOfTokens");
	}
	
	// TYPEDEF: ListOfUInts	//check if this type has an existing base
	type = daeAtomicType::get("Uint");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("ListOfUInts");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("ListOfUInts");
	}
	
	// TYPEDEF: Bool2	//check if this type has an existing base
	type = daeAtomicType::get("ListOfBools");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Bool2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Bool2");
	}
	
	// TYPEDEF: Bool3	//check if this type has an existing base
	type = daeAtomicType::get("ListOfBools");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Bool3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Bool3");
	}
	
	// TYPEDEF: Bool4	//check if this type has an existing base
	type = daeAtomicType::get("ListOfBools");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Bool4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Bool4");
	}
	
	// TYPEDEF: Float2	//check if this type has an existing base
	type = daeAtomicType::get("ListOfFloats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Float2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float2");
	}
	
	// TYPEDEF: Float3	//check if this type has an existing base
	type = daeAtomicType::get("ListOfFloats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Float3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float3");
	}
	
	// TYPEDEF: Float4	//check if this type has an existing base
	type = daeAtomicType::get("ListOfFloats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Float4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float4");
	}
	
	// TYPEDEF: Float7	//check if this type has an existing base
	type = daeAtomicType::get("ListOfFloats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Float7");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float7");
	}
	
	// TYPEDEF: Float2x2	//check if this type has an existing base
	type = daeAtomicType::get("ListOfFloats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Float2x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float2x2");
	}
	
	// TYPEDEF: Float3x3	//check if this type has an existing base
	type = daeAtomicType::get("ListOfFloats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Float3x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float3x3");
	}
	
	// TYPEDEF: Float4x4	//check if this type has an existing base
	type = daeAtomicType::get("ListOfFloats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Float4x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float4x4");
	}
	
	// TYPEDEF: Float2x3	//check if this type has an existing base
	type = daeAtomicType::get("ListOfFloats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Float2x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float2x3");
	}
	
	// TYPEDEF: Float2x4	//check if this type has an existing base
	type = daeAtomicType::get("ListOfFloats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Float2x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float2x4");
	}
	
	// TYPEDEF: Float3x2	//check if this type has an existing base
	type = daeAtomicType::get("ListOfFloats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Float3x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float3x2");
	}
	
	// TYPEDEF: Float3x4	//check if this type has an existing base
	type = daeAtomicType::get("ListOfFloats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Float3x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float3x4");
	}
	
	// TYPEDEF: Float4x2	//check if this type has an existing base
	type = daeAtomicType::get("ListOfFloats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Float4x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float4x2");
	}
	
	// TYPEDEF: Float4x3	//check if this type has an existing base
	type = daeAtomicType::get("ListOfFloats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Float4x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float4x3");
	}
	
	// TYPEDEF: Int2	//check if this type has an existing base
	type = daeAtomicType::get("ListOfInts");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Int2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Int2");
	}
	
	// TYPEDEF: Int3	//check if this type has an existing base
	type = daeAtomicType::get("ListOfInts");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Int3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Int3");
	}
	
	// TYPEDEF: Int4	//check if this type has an existing base
	type = daeAtomicType::get("ListOfInts");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Int4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Int4");
	}
	
	// TYPEDEF: Int2x2	//check if this type has an existing base
	type = daeAtomicType::get("ListOfInts");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Int2x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Int2x2");
	}
	
	// TYPEDEF: Int3x3	//check if this type has an existing base
	type = daeAtomicType::get("ListOfInts");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Int3x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Int3x3");
	}
	
	// TYPEDEF: Int4x4	//check if this type has an existing base
	type = daeAtomicType::get("ListOfInts");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Int4x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Int4x4");
	}
	
    // ENUM: MorphMethodType    
    type = new daeEnumType;
    type->_nameBindings.append("MorphMethodType");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("NORMALIZED");
	((daeEnumType*)type)->_values->append(MORPHMETHODTYPE_NORMALIZED);    
	((daeEnumType*)type)->_strings->append("RELATIVE");
	((daeEnumType*)type)->_values->append(MORPHMETHODTYPE_RELATIVE);    
	daeAtomicType::append( type );

    // ENUM: NodeType    
    type = new daeEnumType;
    type->_nameBindings.append("NodeType");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("JOINT");
	((daeEnumType*)type)->_values->append(NODETYPE_JOINT);    
	((daeEnumType*)type)->_strings->append("NODE");
	((daeEnumType*)type)->_values->append(NODETYPE_NODE);    
	daeAtomicType::append( type );

	// TYPEDEF: URIFragmentType	//check if this type has an existing base
	type = daeAtomicType::get("xsAnyURI");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("URIFragmentType");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("URIFragmentType");
	}
	
    // ENUM: UpAxisType    
    type = new daeEnumType;
    type->_nameBindings.append("UpAxisType");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("X_UP");
	((daeEnumType*)type)->_values->append(UPAXISTYPE_X_UP);    
	((daeEnumType*)type)->_strings->append("Y_UP");
	((daeEnumType*)type)->_values->append(UPAXISTYPE_Y_UP);    
	((daeEnumType*)type)->_strings->append("Z_UP");
	((daeEnumType*)type)->_values->append(UPAXISTYPE_Z_UP);    
	daeAtomicType::append( type );

    // ENUM: VersionType    
    type = new daeEnumType;
    type->_nameBindings.append("VersionType");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("1.4.0");
	((daeEnumType*)type)->_values->append(VERSIONTYPE_1_4_0);    
	daeAtomicType::append( type );

	// TYPEDEF: Fx_color_common	//check if this type has an existing base
	type = daeAtomicType::get("Float4");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Fx_color_common");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Fx_color_common");
	}
	
    // ENUM: Fx_surface_type_enum    
    type = new daeEnumType;
    type->_nameBindings.append("Fx_surface_type_enum");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("UNTYPED");
	((daeEnumType*)type)->_values->append(FX_SURFACE_TYPE_ENUM_UNTYPED);    
	((daeEnumType*)type)->_strings->append("1D");
	((daeEnumType*)type)->_values->append(FX_SURFACE_TYPE_ENUM_1D);    
	((daeEnumType*)type)->_strings->append("2D");
	((daeEnumType*)type)->_values->append(FX_SURFACE_TYPE_ENUM_2D);    
	((daeEnumType*)type)->_strings->append("3D");
	((daeEnumType*)type)->_values->append(FX_SURFACE_TYPE_ENUM_3D);    
	((daeEnumType*)type)->_strings->append("RECT");
	((daeEnumType*)type)->_values->append(FX_SURFACE_TYPE_ENUM_RECT);    
	((daeEnumType*)type)->_strings->append("CUBE");
	((daeEnumType*)type)->_values->append(FX_SURFACE_TYPE_ENUM_CUBE);    
	((daeEnumType*)type)->_strings->append("DEPTH");
	((daeEnumType*)type)->_values->append(FX_SURFACE_TYPE_ENUM_DEPTH);    
	daeAtomicType::append( type );

    // ENUM: Fx_surface_face_enum    
    type = new daeEnumType;
    type->_nameBindings.append("Fx_surface_face_enum");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("POSITIVE_X");
	((daeEnumType*)type)->_values->append(FX_SURFACE_FACE_ENUM_POSITIVE_X);    
	((daeEnumType*)type)->_strings->append("NEGATIVE_X");
	((daeEnumType*)type)->_values->append(FX_SURFACE_FACE_ENUM_NEGATIVE_X);    
	((daeEnumType*)type)->_strings->append("POSITIVE_Y");
	((daeEnumType*)type)->_values->append(FX_SURFACE_FACE_ENUM_POSITIVE_Y);    
	((daeEnumType*)type)->_strings->append("NEGATIVE_Y");
	((daeEnumType*)type)->_values->append(FX_SURFACE_FACE_ENUM_NEGATIVE_Y);    
	((daeEnumType*)type)->_strings->append("POSITIVE_Z");
	((daeEnumType*)type)->_values->append(FX_SURFACE_FACE_ENUM_POSITIVE_Z);    
	((daeEnumType*)type)->_strings->append("NEGATIVE_Z");
	((daeEnumType*)type)->_values->append(FX_SURFACE_FACE_ENUM_NEGATIVE_Z);    
	daeAtomicType::append( type );

    // ENUM: Fx_sampler_wrap_common    
    type = new daeEnumType;
    type->_nameBindings.append("Fx_sampler_wrap_common");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("NONE");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_WRAP_COMMON_NONE);    
	((daeEnumType*)type)->_strings->append("WRAP");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_WRAP_COMMON_WRAP);    
	((daeEnumType*)type)->_strings->append("MIRROR");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_WRAP_COMMON_MIRROR);    
	((daeEnumType*)type)->_strings->append("CLAMP");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_WRAP_COMMON_CLAMP);    
	((daeEnumType*)type)->_strings->append("BORDER");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_WRAP_COMMON_BORDER);    
	daeAtomicType::append( type );

    // ENUM: Fx_sampler_filter_common    
    type = new daeEnumType;
    type->_nameBindings.append("Fx_sampler_filter_common");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("NONE");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_FILTER_COMMON_NONE);    
	((daeEnumType*)type)->_strings->append("NEAREST");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_FILTER_COMMON_NEAREST);    
	((daeEnumType*)type)->_strings->append("LINEAR");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_FILTER_COMMON_LINEAR);    
	((daeEnumType*)type)->_strings->append("NEAREST_MIPMAP_NEAREST");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_FILTER_COMMON_NEAREST_MIPMAP_NEAREST);    
	((daeEnumType*)type)->_strings->append("LINEAR_MIPMAP_NEAREST");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_FILTER_COMMON_LINEAR_MIPMAP_NEAREST);    
	((daeEnumType*)type)->_strings->append("NEAREST_MIPMAP_LINEAR");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_FILTER_COMMON_NEAREST_MIPMAP_LINEAR);    
	((daeEnumType*)type)->_strings->append("LINEAR_MIPMAP_LINEAR");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_FILTER_COMMON_LINEAR_MIPMAP_LINEAR);    
	daeAtomicType::append( type );

    // ENUM: Fx_modifier_enum_common    
    type = new daeEnumType;
    type->_nameBindings.append("Fx_modifier_enum_common");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("CONST");
	((daeEnumType*)type)->_values->append(FX_MODIFIER_ENUM_COMMON_CONST);    
	((daeEnumType*)type)->_strings->append("UNIFORM");
	((daeEnumType*)type)->_values->append(FX_MODIFIER_ENUM_COMMON_UNIFORM);    
	((daeEnumType*)type)->_strings->append("VARYING");
	((daeEnumType*)type)->_values->append(FX_MODIFIER_ENUM_COMMON_VARYING);    
	((daeEnumType*)type)->_strings->append("STATIC");
	((daeEnumType*)type)->_values->append(FX_MODIFIER_ENUM_COMMON_STATIC);    
	((daeEnumType*)type)->_strings->append("VOLATILE");
	((daeEnumType*)type)->_values->append(FX_MODIFIER_ENUM_COMMON_VOLATILE);    
	((daeEnumType*)type)->_strings->append("EXTERN");
	((daeEnumType*)type)->_values->append(FX_MODIFIER_ENUM_COMMON_EXTERN);    
	((daeEnumType*)type)->_strings->append("SHARED");
	((daeEnumType*)type)->_values->append(FX_MODIFIER_ENUM_COMMON_SHARED);    
	daeAtomicType::append( type );

	// TYPEDEF: Fx_draw_common	//check if this type has an existing base
	type = daeAtomicType::get("xsString");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Fx_draw_common");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Fx_draw_common");
	}
	
    // ENUM: Fx_pipeline_stage_common    
    type = new daeEnumType;
    type->_nameBindings.append("Fx_pipeline_stage_common");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("VERTEXPROGRAM");
	((daeEnumType*)type)->_values->append(FX_PIPELINE_STAGE_COMMON_VERTEXPROGRAM);    
	((daeEnumType*)type)->_strings->append("FRAGMENTPROGRAM");
	((daeEnumType*)type)->_values->append(FX_PIPELINE_STAGE_COMMON_FRAGMENTPROGRAM);    
	((daeEnumType*)type)->_strings->append("VERTEXSHADER");
	((daeEnumType*)type)->_values->append(FX_PIPELINE_STAGE_COMMON_VERTEXSHADER);    
	((daeEnumType*)type)->_strings->append("PIXELSHADER");
	((daeEnumType*)type)->_values->append(FX_PIPELINE_STAGE_COMMON_PIXELSHADER);    
	daeAtomicType::append( type );

	// TYPEDEF: GL_MAX_LIGHTS_index	//check if this type has an existing base
	type = daeAtomicType::get("xsNonNegativeInteger");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("GL_MAX_LIGHTS_index");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("GL_MAX_LIGHTS_index");
	}
	
	// TYPEDEF: GL_MAX_CLIP_PLANES_index	//check if this type has an existing base
	type = daeAtomicType::get("xsNonNegativeInteger");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("GL_MAX_CLIP_PLANES_index");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("GL_MAX_CLIP_PLANES_index");
	}
	
	// TYPEDEF: GL_MAX_TEXTURE_IMAGE_UNITS_index	//check if this type has an existing base
	type = daeAtomicType::get("xsNonNegativeInteger");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("GL_MAX_TEXTURE_IMAGE_UNITS_index");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("GL_MAX_TEXTURE_IMAGE_UNITS_index");
	}
	
    // ENUM: Gl_blend_type    
    type = new daeEnumType;
    type->_nameBindings.append("Gl_blend_type");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("ZERO");
	((daeEnumType*)type)->_values->append(GL_BLEND_TYPE_ZERO);    
	((daeEnumType*)type)->_strings->append("ONE");
	((daeEnumType*)type)->_values->append(GL_BLEND_TYPE_ONE);    
	((daeEnumType*)type)->_strings->append("SRC_COLOR");
	((daeEnumType*)type)->_values->append(GL_BLEND_TYPE_SRC_COLOR);    
	((daeEnumType*)type)->_strings->append("ONE_MINUS_SRC_COLOR");
	((daeEnumType*)type)->_values->append(GL_BLEND_TYPE_ONE_MINUS_SRC_COLOR);    
	((daeEnumType*)type)->_strings->append("DEST_COLOR");
	((daeEnumType*)type)->_values->append(GL_BLEND_TYPE_DEST_COLOR);    
	((daeEnumType*)type)->_strings->append("ONE_MINUS_DEST_COLOR");
	((daeEnumType*)type)->_values->append(GL_BLEND_TYPE_ONE_MINUS_DEST_COLOR);    
	((daeEnumType*)type)->_strings->append("SRC_ALPHA");
	((daeEnumType*)type)->_values->append(GL_BLEND_TYPE_SRC_ALPHA);    
	((daeEnumType*)type)->_strings->append("ONE_MINUS_SRC_ALPHA");
	((daeEnumType*)type)->_values->append(GL_BLEND_TYPE_ONE_MINUS_SRC_ALPHA);    
	((daeEnumType*)type)->_strings->append("DST_ALPHA");
	((daeEnumType*)type)->_values->append(GL_BLEND_TYPE_DST_ALPHA);    
	((daeEnumType*)type)->_strings->append("ONE_MINUS_DST_ALPHA");
	((daeEnumType*)type)->_values->append(GL_BLEND_TYPE_ONE_MINUS_DST_ALPHA);    
	((daeEnumType*)type)->_strings->append("CONSTANT_COLOR");
	((daeEnumType*)type)->_values->append(GL_BLEND_TYPE_CONSTANT_COLOR);    
	((daeEnumType*)type)->_strings->append("ONE_MINUS_CONSTANT_COLOR");
	((daeEnumType*)type)->_values->append(GL_BLEND_TYPE_ONE_MINUS_CONSTANT_COLOR);    
	((daeEnumType*)type)->_strings->append("CONSTANT_ALPHA");
	((daeEnumType*)type)->_values->append(GL_BLEND_TYPE_CONSTANT_ALPHA);    
	((daeEnumType*)type)->_strings->append("ONE_MINUS_CONSTANT_ALPHA");
	((daeEnumType*)type)->_values->append(GL_BLEND_TYPE_ONE_MINUS_CONSTANT_ALPHA);    
	((daeEnumType*)type)->_strings->append("SRC_ALPHA_SATURATE");
	((daeEnumType*)type)->_values->append(GL_BLEND_TYPE_SRC_ALPHA_SATURATE);    
	daeAtomicType::append( type );

    // ENUM: Gl_face_type    
    type = new daeEnumType;
    type->_nameBindings.append("Gl_face_type");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("FRONT");
	((daeEnumType*)type)->_values->append(GL_FACE_TYPE_FRONT);    
	((daeEnumType*)type)->_strings->append("BACK");
	((daeEnumType*)type)->_values->append(GL_FACE_TYPE_BACK);    
	((daeEnumType*)type)->_strings->append("FRONT_AND_BACK");
	((daeEnumType*)type)->_values->append(GL_FACE_TYPE_FRONT_AND_BACK);    
	daeAtomicType::append( type );

    // ENUM: Gl_blend_equation_type    
    type = new daeEnumType;
    type->_nameBindings.append("Gl_blend_equation_type");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("FUNC_ADD");
	((daeEnumType*)type)->_values->append(GL_BLEND_EQUATION_TYPE_FUNC_ADD);    
	((daeEnumType*)type)->_strings->append("FUNC_SUBTRACT");
	((daeEnumType*)type)->_values->append(GL_BLEND_EQUATION_TYPE_FUNC_SUBTRACT);    
	((daeEnumType*)type)->_strings->append("FUNC_REVERSE_SUBTRACT");
	((daeEnumType*)type)->_values->append(GL_BLEND_EQUATION_TYPE_FUNC_REVERSE_SUBTRACT);    
	((daeEnumType*)type)->_strings->append("MIN");
	((daeEnumType*)type)->_values->append(GL_BLEND_EQUATION_TYPE_MIN);    
	((daeEnumType*)type)->_strings->append("MAX");
	((daeEnumType*)type)->_values->append(GL_BLEND_EQUATION_TYPE_MAX);    
	daeAtomicType::append( type );

    // ENUM: Gl_func_type    
    type = new daeEnumType;
    type->_nameBindings.append("Gl_func_type");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("NEVER");
	((daeEnumType*)type)->_values->append(GL_FUNC_TYPE_NEVER);    
	((daeEnumType*)type)->_strings->append("LESS");
	((daeEnumType*)type)->_values->append(GL_FUNC_TYPE_LESS);    
	((daeEnumType*)type)->_strings->append("LEQUAL");
	((daeEnumType*)type)->_values->append(GL_FUNC_TYPE_LEQUAL);    
	((daeEnumType*)type)->_strings->append("EQUAL");
	((daeEnumType*)type)->_values->append(GL_FUNC_TYPE_EQUAL);    
	((daeEnumType*)type)->_strings->append("GREATER");
	((daeEnumType*)type)->_values->append(GL_FUNC_TYPE_GREATER);    
	((daeEnumType*)type)->_strings->append("NOTEQUAL");
	((daeEnumType*)type)->_values->append(GL_FUNC_TYPE_NOTEQUAL);    
	((daeEnumType*)type)->_strings->append("GEQUAL");
	((daeEnumType*)type)->_values->append(GL_FUNC_TYPE_GEQUAL);    
	((daeEnumType*)type)->_strings->append("ALWAYS");
	((daeEnumType*)type)->_values->append(GL_FUNC_TYPE_ALWAYS);    
	daeAtomicType::append( type );

    // ENUM: Gl_stencil_op_type    
    type = new daeEnumType;
    type->_nameBindings.append("Gl_stencil_op_type");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("KEEP");
	((daeEnumType*)type)->_values->append(GL_STENCIL_OP_TYPE_KEEP);    
	((daeEnumType*)type)->_strings->append("ZERO");
	((daeEnumType*)type)->_values->append(GL_STENCIL_OP_TYPE_ZERO);    
	((daeEnumType*)type)->_strings->append("REPLACE");
	((daeEnumType*)type)->_values->append(GL_STENCIL_OP_TYPE_REPLACE);    
	((daeEnumType*)type)->_strings->append("INCR");
	((daeEnumType*)type)->_values->append(GL_STENCIL_OP_TYPE_INCR);    
	((daeEnumType*)type)->_strings->append("DECR");
	((daeEnumType*)type)->_values->append(GL_STENCIL_OP_TYPE_DECR);    
	((daeEnumType*)type)->_strings->append("INVERT");
	((daeEnumType*)type)->_values->append(GL_STENCIL_OP_TYPE_INVERT);    
	((daeEnumType*)type)->_strings->append("INCR_WRAP");
	((daeEnumType*)type)->_values->append(GL_STENCIL_OP_TYPE_INCR_WRAP);    
	((daeEnumType*)type)->_strings->append("DECR_WRAP");
	((daeEnumType*)type)->_values->append(GL_STENCIL_OP_TYPE_DECR_WRAP);    
	daeAtomicType::append( type );

    // ENUM: Gl_material_type    
    type = new daeEnumType;
    type->_nameBindings.append("Gl_material_type");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("EMISSION");
	((daeEnumType*)type)->_values->append(GL_MATERIAL_TYPE_EMISSION);    
	((daeEnumType*)type)->_strings->append("AMBIENT");
	((daeEnumType*)type)->_values->append(GL_MATERIAL_TYPE_AMBIENT);    
	((daeEnumType*)type)->_strings->append("DIFFUSE");
	((daeEnumType*)type)->_values->append(GL_MATERIAL_TYPE_DIFFUSE);    
	((daeEnumType*)type)->_strings->append("SPECULAR");
	((daeEnumType*)type)->_values->append(GL_MATERIAL_TYPE_SPECULAR);    
	((daeEnumType*)type)->_strings->append("AMBIENT_AND_DIFFUSE");
	((daeEnumType*)type)->_values->append(GL_MATERIAL_TYPE_AMBIENT_AND_DIFFUSE);    
	daeAtomicType::append( type );

    // ENUM: Gl_fog_type    
    type = new daeEnumType;
    type->_nameBindings.append("Gl_fog_type");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("LINEAR");
	((daeEnumType*)type)->_values->append(GL_FOG_TYPE_LINEAR);    
	((daeEnumType*)type)->_strings->append("EXP");
	((daeEnumType*)type)->_values->append(GL_FOG_TYPE_EXP);    
	((daeEnumType*)type)->_strings->append("EXP2");
	((daeEnumType*)type)->_values->append(GL_FOG_TYPE_EXP2);    
	daeAtomicType::append( type );

    // ENUM: Gl_fog_coord_src_type    
    type = new daeEnumType;
    type->_nameBindings.append("Gl_fog_coord_src_type");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("FOG_COORDINATE");
	((daeEnumType*)type)->_values->append(GL_FOG_COORD_SRC_TYPE_FOG_COORDINATE);    
	((daeEnumType*)type)->_strings->append("FRAGMENT_DEPTH");
	((daeEnumType*)type)->_values->append(GL_FOG_COORD_SRC_TYPE_FRAGMENT_DEPTH);    
	daeAtomicType::append( type );

    // ENUM: Gl_front_face_type    
    type = new daeEnumType;
    type->_nameBindings.append("Gl_front_face_type");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("CW");
	((daeEnumType*)type)->_values->append(GL_FRONT_FACE_TYPE_CW);    
	((daeEnumType*)type)->_strings->append("CCW");
	((daeEnumType*)type)->_values->append(GL_FRONT_FACE_TYPE_CCW);    
	daeAtomicType::append( type );

    // ENUM: Gl_light_model_color_control_type    
    type = new daeEnumType;
    type->_nameBindings.append("Gl_light_model_color_control_type");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("SINGLE_COLOR");
	((daeEnumType*)type)->_values->append(GL_LIGHT_MODEL_COLOR_CONTROL_TYPE_SINGLE_COLOR);    
	((daeEnumType*)type)->_strings->append("SEPARATE_SPECULAR_COLOR");
	((daeEnumType*)type)->_values->append(GL_LIGHT_MODEL_COLOR_CONTROL_TYPE_SEPARATE_SPECULAR_COLOR);    
	daeAtomicType::append( type );

    // ENUM: Gl_logic_op_type    
    type = new daeEnumType;
    type->_nameBindings.append("Gl_logic_op_type");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("CLEAR");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_TYPE_CLEAR);    
	((daeEnumType*)type)->_strings->append("AND");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_TYPE_AND);    
	((daeEnumType*)type)->_strings->append("AND_REVERSE");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_TYPE_AND_REVERSE);    
	((daeEnumType*)type)->_strings->append("COPY");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_TYPE_COPY);    
	((daeEnumType*)type)->_strings->append("AND_INVERTED");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_TYPE_AND_INVERTED);    
	((daeEnumType*)type)->_strings->append("NOOP");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_TYPE_NOOP);    
	((daeEnumType*)type)->_strings->append("XOR");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_TYPE_XOR);    
	((daeEnumType*)type)->_strings->append("OR");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_TYPE_OR);    
	((daeEnumType*)type)->_strings->append("NOR");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_TYPE_NOR);    
	((daeEnumType*)type)->_strings->append("EQUIV");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_TYPE_EQUIV);    
	((daeEnumType*)type)->_strings->append("INVERT");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_TYPE_INVERT);    
	((daeEnumType*)type)->_strings->append("OR_REVERSE");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_TYPE_OR_REVERSE);    
	((daeEnumType*)type)->_strings->append("COPY_INVERTED");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_TYPE_COPY_INVERTED);    
	((daeEnumType*)type)->_strings->append("NAND");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_TYPE_NAND);    
	((daeEnumType*)type)->_strings->append("SET");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_TYPE_SET);    
	daeAtomicType::append( type );

    // ENUM: Gl_polygon_mode_type    
    type = new daeEnumType;
    type->_nameBindings.append("Gl_polygon_mode_type");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("POINT");
	((daeEnumType*)type)->_values->append(GL_POLYGON_MODE_TYPE_POINT);    
	((daeEnumType*)type)->_strings->append("LINE");
	((daeEnumType*)type)->_values->append(GL_POLYGON_MODE_TYPE_LINE);    
	((daeEnumType*)type)->_strings->append("FILL");
	((daeEnumType*)type)->_values->append(GL_POLYGON_MODE_TYPE_FILL);    
	daeAtomicType::append( type );

    // ENUM: Gl_shade_model_type    
    type = new daeEnumType;
    type->_nameBindings.append("Gl_shade_model_type");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("FLAT");
	((daeEnumType*)type)->_values->append(GL_SHADE_MODEL_TYPE_FLAT);    
	((daeEnumType*)type)->_strings->append("SMOOTH");
	((daeEnumType*)type)->_values->append(GL_SHADE_MODEL_TYPE_SMOOTH);    
	daeAtomicType::append( type );

	// TYPEDEF: Gl_alpha_value_type	//check if this type has an existing base
	type = daeAtomicType::get("xsFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Gl_alpha_value_type");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Gl_alpha_value_type");
	}
	
    // ENUM: Gl_enumeration    
    type = new daeEnumType;
    type->_nameBindings.append("Gl_enumeration");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("ZERO");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_ZERO);    
	((daeEnumType*)type)->_strings->append("ONE");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_ONE);    
	((daeEnumType*)type)->_strings->append("SRC_COLOR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_SRC_COLOR);    
	((daeEnumType*)type)->_strings->append("ONE_MINUS_SRC_COLOR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_ONE_MINUS_SRC_COLOR);    
	((daeEnumType*)type)->_strings->append("DEST_COLOR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_DEST_COLOR);    
	((daeEnumType*)type)->_strings->append("ONE_MINUS_DEST_COLOR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_ONE_MINUS_DEST_COLOR);    
	((daeEnumType*)type)->_strings->append("SRC_ALPHA");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_SRC_ALPHA);    
	((daeEnumType*)type)->_strings->append("ONE_MINUS_SRC_ALPHA");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_ONE_MINUS_SRC_ALPHA);    
	((daeEnumType*)type)->_strings->append("DST_ALPHA");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_DST_ALPHA);    
	((daeEnumType*)type)->_strings->append("ONE_MINUS_DST_ALPHA");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_ONE_MINUS_DST_ALPHA);    
	((daeEnumType*)type)->_strings->append("CONSTANT_COLOR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_CONSTANT_COLOR);    
	((daeEnumType*)type)->_strings->append("ONE_MINUS_CONSTANT_COLOR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_ONE_MINUS_CONSTANT_COLOR);    
	((daeEnumType*)type)->_strings->append("CONSTANT_ALPHA");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_CONSTANT_ALPHA);    
	((daeEnumType*)type)->_strings->append("ONE_MINUS_CONSTANT_ALPHA");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_ONE_MINUS_CONSTANT_ALPHA);    
	((daeEnumType*)type)->_strings->append("SRC_ALPHA_SATURATE");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_SRC_ALPHA_SATURATE);    
	((daeEnumType*)type)->_strings->append("FRONT");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_FRONT);    
	((daeEnumType*)type)->_strings->append("BACK");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_BACK);    
	((daeEnumType*)type)->_strings->append("FRONT_AND_BACK");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_FRONT_AND_BACK);    
	((daeEnumType*)type)->_strings->append("FUNC_ADD");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_FUNC_ADD);    
	((daeEnumType*)type)->_strings->append("FUNC_SUBTRACT");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_FUNC_SUBTRACT);    
	((daeEnumType*)type)->_strings->append("FUNC_REVERSE_SUBTRACT");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_FUNC_REVERSE_SUBTRACT);    
	((daeEnumType*)type)->_strings->append("MIN");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_MIN);    
	((daeEnumType*)type)->_strings->append("MAX");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_MAX);    
	((daeEnumType*)type)->_strings->append("NEVER");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_NEVER);    
	((daeEnumType*)type)->_strings->append("LESS");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_LESS);    
	((daeEnumType*)type)->_strings->append("LEQUAL");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_LEQUAL);    
	((daeEnumType*)type)->_strings->append("EQUAL");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_EQUAL);    
	((daeEnumType*)type)->_strings->append("GREATER");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_GREATER);    
	((daeEnumType*)type)->_strings->append("NOTEQUAL");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_NOTEQUAL);    
	((daeEnumType*)type)->_strings->append("GEQUAL");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_GEQUAL);    
	((daeEnumType*)type)->_strings->append("ALWAYS");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_ALWAYS);    
	((daeEnumType*)type)->_strings->append("KEEP");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_KEEP);    
	((daeEnumType*)type)->_strings->append("REPLACE");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_REPLACE);    
	((daeEnumType*)type)->_strings->append("INCR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_INCR);    
	((daeEnumType*)type)->_strings->append("DECR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_DECR);    
	((daeEnumType*)type)->_strings->append("INVERT");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_INVERT);    
	((daeEnumType*)type)->_strings->append("INCR_WRAP");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_INCR_WRAP);    
	((daeEnumType*)type)->_strings->append("DECR_WRAP");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_DECR_WRAP);    
	((daeEnumType*)type)->_strings->append("EMISSION");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_EMISSION);    
	((daeEnumType*)type)->_strings->append("AMBIENT");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_AMBIENT);    
	((daeEnumType*)type)->_strings->append("DIFFUSE");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_DIFFUSE);    
	((daeEnumType*)type)->_strings->append("SPECULAR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_SPECULAR);    
	((daeEnumType*)type)->_strings->append("AMBIENT_AND_DIFFUSE");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_AMBIENT_AND_DIFFUSE);    
	((daeEnumType*)type)->_strings->append("LINEAR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_LINEAR);    
	((daeEnumType*)type)->_strings->append("EXP");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_EXP);    
	((daeEnumType*)type)->_strings->append("EXP2");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_EXP2);    
	((daeEnumType*)type)->_strings->append("FOG_COORDINATE");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_FOG_COORDINATE);    
	((daeEnumType*)type)->_strings->append("FRAGMENT_DEPTH");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_FRAGMENT_DEPTH);    
	((daeEnumType*)type)->_strings->append("CW");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_CW);    
	((daeEnumType*)type)->_strings->append("CCW");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_CCW);    
	((daeEnumType*)type)->_strings->append("SINGLE_COLOR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_SINGLE_COLOR);    
	((daeEnumType*)type)->_strings->append("SEPARATE_SPECULAR_COLOR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_SEPARATE_SPECULAR_COLOR);    
	((daeEnumType*)type)->_strings->append("CLEAR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_CLEAR);    
	((daeEnumType*)type)->_strings->append("AND");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_AND);    
	((daeEnumType*)type)->_strings->append("AND_REVERSE");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_AND_REVERSE);    
	((daeEnumType*)type)->_strings->append("COPY");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_COPY);    
	((daeEnumType*)type)->_strings->append("AND_INVERTED");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_AND_INVERTED);    
	((daeEnumType*)type)->_strings->append("NOOP");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_NOOP);    
	((daeEnumType*)type)->_strings->append("XOR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_XOR);    
	((daeEnumType*)type)->_strings->append("OR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_OR);    
	((daeEnumType*)type)->_strings->append("NOR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_NOR);    
	((daeEnumType*)type)->_strings->append("EQUIV");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_EQUIV);    
	((daeEnumType*)type)->_strings->append("OR_REVERSE");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_OR_REVERSE);    
	((daeEnumType*)type)->_strings->append("COPY_INVERTED");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_COPY_INVERTED);    
	((daeEnumType*)type)->_strings->append("NAND");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_NAND);    
	((daeEnumType*)type)->_strings->append("SET");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_SET);    
	((daeEnumType*)type)->_strings->append("POINT");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_POINT);    
	((daeEnumType*)type)->_strings->append("LINE");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_LINE);    
	((daeEnumType*)type)->_strings->append("FILL");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_FILL);    
	((daeEnumType*)type)->_strings->append("FLAT");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_FLAT);    
	((daeEnumType*)type)->_strings->append("SMOOTH");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_SMOOTH);    
	daeAtomicType::append( type );

	// TYPEDEF: Glsl_float	//check if this type has an existing base
	type = daeAtomicType::get("xsFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Glsl_float");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Glsl_float");
	}
	
	// TYPEDEF: Glsl_int	//check if this type has an existing base
	type = daeAtomicType::get("xsInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Glsl_int");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Glsl_int");
	}
	
	// TYPEDEF: Glsl_bool	//check if this type has an existing base
	type = daeAtomicType::get("xsBoolean");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Glsl_bool");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Glsl_bool");
	}
	
	// TYPEDEF: Glsl_ListOfBool	//check if this type has an existing base
	type = daeAtomicType::get("Glsl_bool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Glsl_ListOfBool");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Glsl_ListOfBool");
	}
	
	// TYPEDEF: Glsl_ListOfFloat	//check if this type has an existing base
	type = daeAtomicType::get("Glsl_float");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Glsl_ListOfFloat");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Glsl_ListOfFloat");
	}
	
	// TYPEDEF: Glsl_ListOfInt	//check if this type has an existing base
	type = daeAtomicType::get("Glsl_int");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Glsl_ListOfInt");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Glsl_ListOfInt");
	}
	
	// TYPEDEF: Glsl_bool2	//check if this type has an existing base
	type = daeAtomicType::get("Glsl_ListOfBool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Glsl_bool2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Glsl_bool2");
	}
	
	// TYPEDEF: Glsl_bool3	//check if this type has an existing base
	type = daeAtomicType::get("Glsl_ListOfBool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Glsl_bool3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Glsl_bool3");
	}
	
	// TYPEDEF: Glsl_bool4	//check if this type has an existing base
	type = daeAtomicType::get("Glsl_ListOfBool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Glsl_bool4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Glsl_bool4");
	}
	
	// TYPEDEF: Glsl_float2	//check if this type has an existing base
	type = daeAtomicType::get("Glsl_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Glsl_float2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Glsl_float2");
	}
	
	// TYPEDEF: Glsl_float3	//check if this type has an existing base
	type = daeAtomicType::get("Glsl_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Glsl_float3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Glsl_float3");
	}
	
	// TYPEDEF: Glsl_float4	//check if this type has an existing base
	type = daeAtomicType::get("Glsl_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Glsl_float4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Glsl_float4");
	}
	
	// TYPEDEF: Glsl_float2x2	//check if this type has an existing base
	type = daeAtomicType::get("Glsl_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Glsl_float2x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Glsl_float2x2");
	}
	
	// TYPEDEF: Glsl_float3x3	//check if this type has an existing base
	type = daeAtomicType::get("Glsl_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Glsl_float3x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Glsl_float3x3");
	}
	
	// TYPEDEF: Glsl_float4x4	//check if this type has an existing base
	type = daeAtomicType::get("Glsl_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Glsl_float4x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Glsl_float4x4");
	}
	
	// TYPEDEF: Glsl_int2	//check if this type has an existing base
	type = daeAtomicType::get("Glsl_ListOfInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Glsl_int2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Glsl_int2");
	}
	
	// TYPEDEF: Glsl_int3	//check if this type has an existing base
	type = daeAtomicType::get("Glsl_ListOfInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Glsl_int3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Glsl_int3");
	}
	
	// TYPEDEF: Glsl_int4	//check if this type has an existing base
	type = daeAtomicType::get("Glsl_ListOfInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Glsl_int4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Glsl_int4");
	}
	
    // ENUM: Glsl_pipeline_stage    
    type = new daeEnumType;
    type->_nameBindings.append("Glsl_pipeline_stage");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("VERTEXPROGRAM");
	((daeEnumType*)type)->_values->append(GLSL_PIPELINE_STAGE_VERTEXPROGRAM);    
	((daeEnumType*)type)->_strings->append("FRAGMENTPROGRAM");
	((daeEnumType*)type)->_values->append(GLSL_PIPELINE_STAGE_FRAGMENTPROGRAM);    
	daeAtomicType::append( type );

	// TYPEDEF: Glsl_identifier	//check if this type has an existing base
	type = daeAtomicType::get("xsString");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Glsl_identifier");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Glsl_identifier");
	}
	
	// TYPEDEF: Cg_bool	//check if this type has an existing base
	type = daeAtomicType::get("xsBoolean");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_bool");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_bool");
	}
	
	// TYPEDEF: Cg_float	//check if this type has an existing base
	type = daeAtomicType::get("xsFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_float");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_float");
	}
	
	// TYPEDEF: Cg_int	//check if this type has an existing base
	type = daeAtomicType::get("xsInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_int");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_int");
	}
	
	// TYPEDEF: Cg_half	//check if this type has an existing base
	type = daeAtomicType::get("xsFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_half");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_half");
	}
	
	// TYPEDEF: Cg_fixed	//check if this type has an existing base
	type = daeAtomicType::get("xsFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_fixed");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_fixed");
	}
	
	// TYPEDEF: Cg_bool1	//check if this type has an existing base
	type = daeAtomicType::get("xsBoolean");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_bool1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_bool1");
	}
	
	// TYPEDEF: Cg_float1	//check if this type has an existing base
	type = daeAtomicType::get("xsFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_float1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_float1");
	}
	
	// TYPEDEF: Cg_int1	//check if this type has an existing base
	type = daeAtomicType::get("xsInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_int1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_int1");
	}
	
	// TYPEDEF: Cg_half1	//check if this type has an existing base
	type = daeAtomicType::get("xsFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_half1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_half1");
	}
	
	// TYPEDEF: Cg_fixed1	//check if this type has an existing base
	type = daeAtomicType::get("xsFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_fixed1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_fixed1");
	}
	
	// TYPEDEF: Cg_ListOfBool	//check if this type has an existing base
	type = daeAtomicType::get("Cg_bool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_ListOfBool");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_ListOfBool");
	}
	
	// TYPEDEF: Cg_ListOfFloat	//check if this type has an existing base
	type = daeAtomicType::get("Cg_float");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_ListOfFloat");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_ListOfFloat");
	}
	
	// TYPEDEF: Cg_ListOfInt	//check if this type has an existing base
	type = daeAtomicType::get("Cg_int");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_ListOfInt");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_ListOfInt");
	}
	
	// TYPEDEF: Cg_ListOfHalf	//check if this type has an existing base
	type = daeAtomicType::get("Cg_half");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_ListOfHalf");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_ListOfHalf");
	}
	
	// TYPEDEF: Cg_ListOfFixed	//check if this type has an existing base
	type = daeAtomicType::get("Cg_fixed");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_ListOfFixed");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_ListOfFixed");
	}
	
	// TYPEDEF: Cg_bool2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfBool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_bool2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_bool2");
	}
	
	// TYPEDEF: Cg_bool3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfBool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_bool3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_bool3");
	}
	
	// TYPEDEF: Cg_bool4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfBool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_bool4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_bool4");
	}
	
	// TYPEDEF: Cg_bool1x1	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfBool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_bool1x1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_bool1x1");
	}
	
	// TYPEDEF: Cg_bool1x2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfBool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_bool1x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_bool1x2");
	}
	
	// TYPEDEF: Cg_bool1x3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfBool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_bool1x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_bool1x3");
	}
	
	// TYPEDEF: Cg_bool1x4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfBool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_bool1x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_bool1x4");
	}
	
	// TYPEDEF: Cg_bool2x1	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfBool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_bool2x1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_bool2x1");
	}
	
	// TYPEDEF: Cg_bool2x2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfBool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_bool2x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_bool2x2");
	}
	
	// TYPEDEF: Cg_bool2x3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfBool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_bool2x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_bool2x3");
	}
	
	// TYPEDEF: Cg_bool2x4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfBool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_bool2x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_bool2x4");
	}
	
	// TYPEDEF: Cg_bool3x1	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfBool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_bool3x1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_bool3x1");
	}
	
	// TYPEDEF: Cg_bool3x2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfBool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_bool3x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_bool3x2");
	}
	
	// TYPEDEF: Cg_bool3x3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfBool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_bool3x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_bool3x3");
	}
	
	// TYPEDEF: Cg_bool3x4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfBool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_bool3x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_bool3x4");
	}
	
	// TYPEDEF: Cg_bool4x1	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfBool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_bool4x1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_bool4x1");
	}
	
	// TYPEDEF: Cg_bool4x2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfBool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_bool4x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_bool4x2");
	}
	
	// TYPEDEF: Cg_bool4x3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfBool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_bool4x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_bool4x3");
	}
	
	// TYPEDEF: Cg_bool4x4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfBool");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_bool4x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_bool4x4");
	}
	
	// TYPEDEF: Cg_float2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_float2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_float2");
	}
	
	// TYPEDEF: Cg_float3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_float3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_float3");
	}
	
	// TYPEDEF: Cg_float4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_float4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_float4");
	}
	
	// TYPEDEF: Cg_float1x1	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_float1x1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_float1x1");
	}
	
	// TYPEDEF: Cg_float1x2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_float1x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_float1x2");
	}
	
	// TYPEDEF: Cg_float1x3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_float1x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_float1x3");
	}
	
	// TYPEDEF: Cg_float1x4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_float1x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_float1x4");
	}
	
	// TYPEDEF: Cg_float2x1	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_float2x1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_float2x1");
	}
	
	// TYPEDEF: Cg_float2x2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_float2x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_float2x2");
	}
	
	// TYPEDEF: Cg_float2x3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_float2x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_float2x3");
	}
	
	// TYPEDEF: Cg_float2x4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_float2x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_float2x4");
	}
	
	// TYPEDEF: Cg_float3x1	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_float3x1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_float3x1");
	}
	
	// TYPEDEF: Cg_float3x2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_float3x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_float3x2");
	}
	
	// TYPEDEF: Cg_float3x3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_float3x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_float3x3");
	}
	
	// TYPEDEF: Cg_float3x4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_float3x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_float3x4");
	}
	
	// TYPEDEF: Cg_float4x1	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_float4x1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_float4x1");
	}
	
	// TYPEDEF: Cg_float4x2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_float4x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_float4x2");
	}
	
	// TYPEDEF: Cg_float4x3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_float4x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_float4x3");
	}
	
	// TYPEDEF: Cg_float4x4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_float4x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_float4x4");
	}
	
	// TYPEDEF: Cg_int2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_int2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_int2");
	}
	
	// TYPEDEF: Cg_int3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_int3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_int3");
	}
	
	// TYPEDEF: Cg_int4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_int4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_int4");
	}
	
	// TYPEDEF: Cg_int1x1	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_int1x1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_int1x1");
	}
	
	// TYPEDEF: Cg_int1x2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_int1x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_int1x2");
	}
	
	// TYPEDEF: Cg_int1x3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_int1x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_int1x3");
	}
	
	// TYPEDEF: Cg_int1x4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_int1x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_int1x4");
	}
	
	// TYPEDEF: Cg_int2x1	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_int2x1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_int2x1");
	}
	
	// TYPEDEF: Cg_int2x2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_int2x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_int2x2");
	}
	
	// TYPEDEF: Cg_int2x3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_int2x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_int2x3");
	}
	
	// TYPEDEF: Cg_int2x4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_int2x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_int2x4");
	}
	
	// TYPEDEF: Cg_int3x1	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_int3x1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_int3x1");
	}
	
	// TYPEDEF: Cg_int3x2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_int3x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_int3x2");
	}
	
	// TYPEDEF: Cg_int3x3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_int3x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_int3x3");
	}
	
	// TYPEDEF: Cg_int3x4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_int3x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_int3x4");
	}
	
	// TYPEDEF: Cg_int4x1	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_int4x1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_int4x1");
	}
	
	// TYPEDEF: Cg_int4x2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_int4x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_int4x2");
	}
	
	// TYPEDEF: Cg_int4x3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_int4x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_int4x3");
	}
	
	// TYPEDEF: Cg_int4x4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfInt");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_int4x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_int4x4");
	}
	
	// TYPEDEF: Cg_half2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfHalf");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_half2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_half2");
	}
	
	// TYPEDEF: Cg_half3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfHalf");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_half3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_half3");
	}
	
	// TYPEDEF: Cg_half4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfHalf");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_half4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_half4");
	}
	
	// TYPEDEF: Cg_half1x1	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfHalf");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_half1x1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_half1x1");
	}
	
	// TYPEDEF: Cg_half1x2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfHalf");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_half1x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_half1x2");
	}
	
	// TYPEDEF: Cg_half1x3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfHalf");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_half1x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_half1x3");
	}
	
	// TYPEDEF: Cg_half1x4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfHalf");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_half1x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_half1x4");
	}
	
	// TYPEDEF: Cg_half2x1	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfHalf");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_half2x1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_half2x1");
	}
	
	// TYPEDEF: Cg_half2x2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfHalf");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_half2x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_half2x2");
	}
	
	// TYPEDEF: Cg_half2x3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfHalf");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_half2x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_half2x3");
	}
	
	// TYPEDEF: Cg_half2x4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfHalf");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_half2x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_half2x4");
	}
	
	// TYPEDEF: Cg_half3x1	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfHalf");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_half3x1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_half3x1");
	}
	
	// TYPEDEF: Cg_half3x2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfHalf");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_half3x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_half3x2");
	}
	
	// TYPEDEF: Cg_half3x3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfHalf");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_half3x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_half3x3");
	}
	
	// TYPEDEF: Cg_half3x4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfHalf");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_half3x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_half3x4");
	}
	
	// TYPEDEF: Cg_half4x1	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfHalf");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_half4x1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_half4x1");
	}
	
	// TYPEDEF: Cg_half4x2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfHalf");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_half4x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_half4x2");
	}
	
	// TYPEDEF: Cg_half4x3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfHalf");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_half4x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_half4x3");
	}
	
	// TYPEDEF: Cg_half4x4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfHalf");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_half4x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_half4x4");
	}
	
	// TYPEDEF: Cg_fixed2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFixed");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_fixed2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_fixed2");
	}
	
	// TYPEDEF: Cg_fixed3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFixed");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_fixed3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_fixed3");
	}
	
	// TYPEDEF: Cg_fixed4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFixed");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_fixed4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_fixed4");
	}
	
	// TYPEDEF: Cg_fixed1x1	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFixed");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_fixed1x1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_fixed1x1");
	}
	
	// TYPEDEF: Cg_fixed1x2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFixed");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_fixed1x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_fixed1x2");
	}
	
	// TYPEDEF: Cg_fixed1x3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFixed");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_fixed1x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_fixed1x3");
	}
	
	// TYPEDEF: Cg_fixed1x4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFixed");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_fixed1x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_fixed1x4");
	}
	
	// TYPEDEF: Cg_fixed2x1	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFixed");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_fixed2x1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_fixed2x1");
	}
	
	// TYPEDEF: Cg_fixed2x2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFixed");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_fixed2x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_fixed2x2");
	}
	
	// TYPEDEF: Cg_fixed2x3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFixed");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_fixed2x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_fixed2x3");
	}
	
	// TYPEDEF: Cg_fixed2x4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFixed");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_fixed2x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_fixed2x4");
	}
	
	// TYPEDEF: Cg_fixed3x1	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFixed");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_fixed3x1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_fixed3x1");
	}
	
	// TYPEDEF: Cg_fixed3x2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFixed");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_fixed3x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_fixed3x2");
	}
	
	// TYPEDEF: Cg_fixed3x3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFixed");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_fixed3x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_fixed3x3");
	}
	
	// TYPEDEF: Cg_fixed3x4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFixed");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_fixed3x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_fixed3x4");
	}
	
	// TYPEDEF: Cg_fixed4x1	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFixed");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_fixed4x1");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_fixed4x1");
	}
	
	// TYPEDEF: Cg_fixed4x2	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFixed");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_fixed4x2");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_fixed4x2");
	}
	
	// TYPEDEF: Cg_fixed4x3	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFixed");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_fixed4x3");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_fixed4x3");
	}
	
	// TYPEDEF: Cg_fixed4x4	//check if this type has an existing base
	type = daeAtomicType::get("Cg_ListOfFixed");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_fixed4x4");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_fixed4x4");
	}
	
    // ENUM: Cg_pipeline_stage    
    type = new daeEnumType;
    type->_nameBindings.append("Cg_pipeline_stage");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("VERTEX");
	((daeEnumType*)type)->_values->append(CG_PIPELINE_STAGE_VERTEX);    
	((daeEnumType*)type)->_strings->append("FRAGMENT");
	((daeEnumType*)type)->_values->append(CG_PIPELINE_STAGE_FRAGMENT);    
	daeAtomicType::append( type );

	// TYPEDEF: Cg_identifier	//check if this type has an existing base
	type = daeAtomicType::get("xsString");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Cg_identifier");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Cg_identifier");
	}
	
	// TYPEDEF: GLES_MAX_LIGHTS_index	//check if this type has an existing base
	type = daeAtomicType::get("xsNonNegativeInteger");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("GLES_MAX_LIGHTS_index");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("GLES_MAX_LIGHTS_index");
	}
	
	// TYPEDEF: GLES_MAX_CLIP_PLANES_index	//check if this type has an existing base
	type = daeAtomicType::get("xsNonNegativeInteger");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("GLES_MAX_CLIP_PLANES_index");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("GLES_MAX_CLIP_PLANES_index");
	}
	
	// TYPEDEF: GLES_MAX_TEXTURE_COORDS_index	//check if this type has an existing base
	type = daeAtomicType::get("xsNonNegativeInteger");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("GLES_MAX_TEXTURE_COORDS_index");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("GLES_MAX_TEXTURE_COORDS_index");
	}
	
	// TYPEDEF: GLES_MAX_TEXTURE_IMAGE_UNITS_index	//check if this type has an existing base
	type = daeAtomicType::get("xsNonNegativeInteger");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("GLES_MAX_TEXTURE_IMAGE_UNITS_index");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("GLES_MAX_TEXTURE_IMAGE_UNITS_index");
	}
	
    // ENUM: Gles_texenv_mode_enums    
    type = new daeEnumType;
    type->_nameBindings.append("Gles_texenv_mode_enums");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("REPLACE");
	((daeEnumType*)type)->_values->append(GLES_TEXENV_MODE_ENUMS_REPLACE);    
	((daeEnumType*)type)->_strings->append("MODULATE");
	((daeEnumType*)type)->_values->append(GLES_TEXENV_MODE_ENUMS_MODULATE);    
	((daeEnumType*)type)->_strings->append("DECAL");
	((daeEnumType*)type)->_values->append(GLES_TEXENV_MODE_ENUMS_DECAL);    
	((daeEnumType*)type)->_strings->append("BLEND");
	((daeEnumType*)type)->_values->append(GLES_TEXENV_MODE_ENUMS_BLEND);    
	((daeEnumType*)type)->_strings->append("ADD");
	((daeEnumType*)type)->_values->append(GLES_TEXENV_MODE_ENUMS_ADD);    
	daeAtomicType::append( type );

    // ENUM: Gles_texcombiner_operatorRGB_enums    
    type = new daeEnumType;
    type->_nameBindings.append("Gles_texcombiner_operatorRGB_enums");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("REPLACE");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATORRGB_ENUMS_REPLACE);    
	((daeEnumType*)type)->_strings->append("MODULATE");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATORRGB_ENUMS_MODULATE);    
	((daeEnumType*)type)->_strings->append("ADD");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATORRGB_ENUMS_ADD);    
	((daeEnumType*)type)->_strings->append("ADD_SIGNED");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATORRGB_ENUMS_ADD_SIGNED);    
	((daeEnumType*)type)->_strings->append("INTERPOLATE");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATORRGB_ENUMS_INTERPOLATE);    
	((daeEnumType*)type)->_strings->append("SUBTRACT");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATORRGB_ENUMS_SUBTRACT);    
	((daeEnumType*)type)->_strings->append("DOT3_RGB");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATORRGB_ENUMS_DOT3_RGB);    
	((daeEnumType*)type)->_strings->append("DOT3_RGBA");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATORRGB_ENUMS_DOT3_RGBA);    
	daeAtomicType::append( type );

    // ENUM: Gles_texcombiner_operatorAlpha_enums    
    type = new daeEnumType;
    type->_nameBindings.append("Gles_texcombiner_operatorAlpha_enums");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("REPLACE");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATORALPHA_ENUMS_REPLACE);    
	((daeEnumType*)type)->_strings->append("MODULATE");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATORALPHA_ENUMS_MODULATE);    
	((daeEnumType*)type)->_strings->append("ADD");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATORALPHA_ENUMS_ADD);    
	((daeEnumType*)type)->_strings->append("ADD_SIGNED");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATORALPHA_ENUMS_ADD_SIGNED);    
	((daeEnumType*)type)->_strings->append("INTERPOLATE");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATORALPHA_ENUMS_INTERPOLATE);    
	((daeEnumType*)type)->_strings->append("SUBTRACT");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATORALPHA_ENUMS_SUBTRACT);    
	daeAtomicType::append( type );

    // ENUM: Gles_texcombiner_source_enums    
    type = new daeEnumType;
    type->_nameBindings.append("Gles_texcombiner_source_enums");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("TEXTURE");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_SOURCE_ENUMS_TEXTURE);    
	((daeEnumType*)type)->_strings->append("CONSTANT");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_SOURCE_ENUMS_CONSTANT);    
	((daeEnumType*)type)->_strings->append("PRIMARY");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_SOURCE_ENUMS_PRIMARY);    
	((daeEnumType*)type)->_strings->append("PREVIOUS");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_SOURCE_ENUMS_PREVIOUS);    
	daeAtomicType::append( type );

    // ENUM: Gles_texcombiner_operandRGB_enums    
    type = new daeEnumType;
    type->_nameBindings.append("Gles_texcombiner_operandRGB_enums");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("SRC_COLOR");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERANDRGB_ENUMS_SRC_COLOR);    
	((daeEnumType*)type)->_strings->append("ONE_MINUS_SRC_COLOR");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERANDRGB_ENUMS_ONE_MINUS_SRC_COLOR);    
	((daeEnumType*)type)->_strings->append("SRC_ALPHA");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERANDRGB_ENUMS_SRC_ALPHA);    
	((daeEnumType*)type)->_strings->append("ONE_MINUS_SRC_ALPHA");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERANDRGB_ENUMS_ONE_MINUS_SRC_ALPHA);    
	daeAtomicType::append( type );

    // ENUM: Gles_texcombiner_operandAlpha_enums    
    type = new daeEnumType;
    type->_nameBindings.append("Gles_texcombiner_operandAlpha_enums");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("SRC_ALPHA");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERANDALPHA_ENUMS_SRC_ALPHA);    
	((daeEnumType*)type)->_strings->append("ONE_MINUS_SRC_ALPHA");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERANDALPHA_ENUMS_ONE_MINUS_SRC_ALPHA);    
	daeAtomicType::append( type );

	// TYPEDEF: Gles_texcombiner_argument_index_type	//check if this type has an existing base
	type = daeAtomicType::get("xsNonNegativeInteger");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Gles_texcombiner_argument_index_type");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Gles_texcombiner_argument_index_type");
	}
	
    // ENUM: Gles_sampler_wrap    
    type = new daeEnumType;
    type->_nameBindings.append("Gles_sampler_wrap");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("REPEAT");
	((daeEnumType*)type)->_values->append(GLES_SAMPLER_WRAP_REPEAT);    
	((daeEnumType*)type)->_strings->append("CLAMP");
	((daeEnumType*)type)->_values->append(GLES_SAMPLER_WRAP_CLAMP);    
	((daeEnumType*)type)->_strings->append("CLAMP_TO_EDGE");
	((daeEnumType*)type)->_values->append(GLES_SAMPLER_WRAP_CLAMP_TO_EDGE);    
	((daeEnumType*)type)->_strings->append("MIRRORED_REPEAT");
	((daeEnumType*)type)->_values->append(GLES_SAMPLER_WRAP_MIRRORED_REPEAT);    
	daeAtomicType::append( type );

    // ENUM: Gles_stencil_op_type    
    type = new daeEnumType;
    type->_nameBindings.append("Gles_stencil_op_type");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("KEEP");
	((daeEnumType*)type)->_values->append(GLES_STENCIL_OP_TYPE_KEEP);    
	((daeEnumType*)type)->_strings->append("ZERO");
	((daeEnumType*)type)->_values->append(GLES_STENCIL_OP_TYPE_ZERO);    
	((daeEnumType*)type)->_strings->append("REPLACE");
	((daeEnumType*)type)->_values->append(GLES_STENCIL_OP_TYPE_REPLACE);    
	((daeEnumType*)type)->_strings->append("INCR");
	((daeEnumType*)type)->_values->append(GLES_STENCIL_OP_TYPE_INCR);    
	((daeEnumType*)type)->_strings->append("DECR");
	((daeEnumType*)type)->_values->append(GLES_STENCIL_OP_TYPE_DECR);    
	((daeEnumType*)type)->_strings->append("INVERT");
	((daeEnumType*)type)->_values->append(GLES_STENCIL_OP_TYPE_INVERT);    
	daeAtomicType::append( type );

    // ENUM: Gles_enumeration    
    type = new daeEnumType;
    type->_nameBindings.append("Gles_enumeration");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("ZERO");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_ZERO);    
	((daeEnumType*)type)->_strings->append("ONE");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_ONE);    
	((daeEnumType*)type)->_strings->append("SRC_COLOR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_SRC_COLOR);    
	((daeEnumType*)type)->_strings->append("ONE_MINUS_SRC_COLOR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_ONE_MINUS_SRC_COLOR);    
	((daeEnumType*)type)->_strings->append("DEST_COLOR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_DEST_COLOR);    
	((daeEnumType*)type)->_strings->append("ONE_MINUS_DEST_COLOR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_ONE_MINUS_DEST_COLOR);    
	((daeEnumType*)type)->_strings->append("SRC_ALPHA");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_SRC_ALPHA);    
	((daeEnumType*)type)->_strings->append("ONE_MINUS_SRC_ALPHA");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_ONE_MINUS_SRC_ALPHA);    
	((daeEnumType*)type)->_strings->append("DST_ALPHA");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_DST_ALPHA);    
	((daeEnumType*)type)->_strings->append("ONE_MINUS_DST_ALPHA");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_ONE_MINUS_DST_ALPHA);    
	((daeEnumType*)type)->_strings->append("CONSTANT_COLOR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_CONSTANT_COLOR);    
	((daeEnumType*)type)->_strings->append("ONE_MINUS_CONSTANT_COLOR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_ONE_MINUS_CONSTANT_COLOR);    
	((daeEnumType*)type)->_strings->append("CONSTANT_ALPHA");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_CONSTANT_ALPHA);    
	((daeEnumType*)type)->_strings->append("ONE_MINUS_CONSTANT_ALPHA");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_ONE_MINUS_CONSTANT_ALPHA);    
	((daeEnumType*)type)->_strings->append("SRC_ALPHA_SATURATE");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_SRC_ALPHA_SATURATE);    
	((daeEnumType*)type)->_strings->append("FRONT");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_FRONT);    
	((daeEnumType*)type)->_strings->append("BACK");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_BACK);    
	((daeEnumType*)type)->_strings->append("FRONT_AND_BACK");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_FRONT_AND_BACK);    
	((daeEnumType*)type)->_strings->append("NEVER");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_NEVER);    
	((daeEnumType*)type)->_strings->append("LESS");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_LESS);    
	((daeEnumType*)type)->_strings->append("LEQUAL");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_LEQUAL);    
	((daeEnumType*)type)->_strings->append("EQUAL");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_EQUAL);    
	((daeEnumType*)type)->_strings->append("GREATER");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_GREATER);    
	((daeEnumType*)type)->_strings->append("NOTEQUAL");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_NOTEQUAL);    
	((daeEnumType*)type)->_strings->append("GEQUAL");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_GEQUAL);    
	((daeEnumType*)type)->_strings->append("ALWAYS");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_ALWAYS);    
	((daeEnumType*)type)->_strings->append("KEEP");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_KEEP);    
	((daeEnumType*)type)->_strings->append("REPLACE");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_REPLACE);    
	((daeEnumType*)type)->_strings->append("INCR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_INCR);    
	((daeEnumType*)type)->_strings->append("DECR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_DECR);    
	((daeEnumType*)type)->_strings->append("INVERT");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_INVERT);    
	((daeEnumType*)type)->_strings->append("INCR_WRAP");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_INCR_WRAP);    
	((daeEnumType*)type)->_strings->append("DECR_WRAP");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_DECR_WRAP);    
	((daeEnumType*)type)->_strings->append("EMISSION");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_EMISSION);    
	((daeEnumType*)type)->_strings->append("AMBIENT");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_AMBIENT);    
	((daeEnumType*)type)->_strings->append("DIFFUSE");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_DIFFUSE);    
	((daeEnumType*)type)->_strings->append("SPECULAR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_SPECULAR);    
	((daeEnumType*)type)->_strings->append("AMBIENT_AND_DIFFUSE");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_AMBIENT_AND_DIFFUSE);    
	((daeEnumType*)type)->_strings->append("LINEAR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_LINEAR);    
	((daeEnumType*)type)->_strings->append("EXP");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_EXP);    
	((daeEnumType*)type)->_strings->append("EXP2");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_EXP2);    
	((daeEnumType*)type)->_strings->append("CW");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_CW);    
	((daeEnumType*)type)->_strings->append("CCW");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_CCW);    
	((daeEnumType*)type)->_strings->append("SINGLE_COLOR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_SINGLE_COLOR);    
	((daeEnumType*)type)->_strings->append("SEPARATE_SPECULAR_COLOR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_SEPARATE_SPECULAR_COLOR);    
	((daeEnumType*)type)->_strings->append("CLEAR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_CLEAR);    
	((daeEnumType*)type)->_strings->append("AND");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_AND);    
	((daeEnumType*)type)->_strings->append("AND_REVERSE");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_AND_REVERSE);    
	((daeEnumType*)type)->_strings->append("COPY");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_COPY);    
	((daeEnumType*)type)->_strings->append("AND_INVERTED");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_AND_INVERTED);    
	((daeEnumType*)type)->_strings->append("NOOP");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_NOOP);    
	((daeEnumType*)type)->_strings->append("XOR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_XOR);    
	((daeEnumType*)type)->_strings->append("OR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_OR);    
	((daeEnumType*)type)->_strings->append("NOR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_NOR);    
	((daeEnumType*)type)->_strings->append("EQUIV");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_EQUIV);    
	((daeEnumType*)type)->_strings->append("OR_REVERSE");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_OR_REVERSE);    
	((daeEnumType*)type)->_strings->append("COPY_INVERTED");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_COPY_INVERTED);    
	((daeEnumType*)type)->_strings->append("NAND");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_NAND);    
	((daeEnumType*)type)->_strings->append("SET");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_SET);    
	((daeEnumType*)type)->_strings->append("POINT");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_POINT);    
	((daeEnumType*)type)->_strings->append("LINE");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_LINE);    
	((daeEnumType*)type)->_strings->append("FILL");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_FILL);    
	((daeEnumType*)type)->_strings->append("FLAT");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_FLAT);    
	((daeEnumType*)type)->_strings->append("SMOOTH");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_SMOOTH);    
	daeAtomicType::append( type );

	// TYPEDEF: Gles_rendertarget_common	//check if this type has an existing base
	type = daeAtomicType::get("xsNCName");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType;
		type->_nameBindings.append("Gles_rendertarget_common");
		daeAtomicType::append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Gles_rendertarget_common");
	}
	
    // ENUM: SpringType    
    type = new daeEnumType;
    type->_nameBindings.append("SpringType");
    ((daeEnumType*)type)->_strings = new daeStringRefArray;
    ((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("LINEAR");
	((daeEnumType*)type)->_values->append(SPRINGTYPE_LINEAR);    
	((daeEnumType*)type)->_strings->append("ANGULAR");
	((daeEnumType*)type)->_values->append(SPRINGTYPE_ANGULAR);    
	daeAtomicType::append( type );

}

daeMetaElement* registerDomElements()
{
	daeMetaElement* meta = domCOLLADA::registerElement();
	// Enable tracking of top level object by default
	domCOLLADA::_Meta->setIsTrackableForQueries(true);
	return meta;	
}
