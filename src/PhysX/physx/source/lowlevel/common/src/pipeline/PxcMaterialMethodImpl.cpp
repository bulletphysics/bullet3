//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#include "PxGeometry.h"
#include "PxcMaterialMethodImpl.h"

namespace physx
{
bool PxcGetMaterialShapeShape			(MATERIAL_METHOD_ARGS);
bool PxcGetMaterialShapeMesh			(MATERIAL_METHOD_ARGS);
bool PxcGetMaterialShapeHeightField		(MATERIAL_METHOD_ARGS);
bool PxcGetMaterialShape				(SINGLE_MATERIAL_METHOD_ARGS);
bool PxcGetMaterialMesh					(SINGLE_MATERIAL_METHOD_ARGS);
bool PxcGetMaterialHeightField			(SINGLE_MATERIAL_METHOD_ARGS);


PxcGetSingleMaterialMethod g_GetSingleMaterialMethodTable[PxGeometryType::eGEOMETRY_COUNT] = 
{
	PxcGetMaterialShape,			//PxGeometryType::eSPHERE
	PxcGetMaterialShape,			//PxGeometryType::ePLANE
	PxcGetMaterialShape,			//PxGeometryType::eCAPSULE
	PxcGetMaterialShape,			//PxGeometryType::eBOX
	PxcGetMaterialShape,			//PxGeometryType::eCONVEXMESH
	PxcGetMaterialMesh,				//PxGeometryType::eTRIANGLEMESH	//not used: mesh always uses swept method for midphase.
	PxcGetMaterialHeightField,		//PxGeometryType::eHEIGHTFIELD	//TODO: make HF midphase that will mask this
		
};

//Table of contact methods for different shape-type combinations
PxcGetMaterialMethod g_GetMaterialMethodTable[][PxGeometryType::eGEOMETRY_COUNT] = 
{
	
	//PxGeometryType::eSPHERE
	{
		PxcGetMaterialShapeShape,			//PxGeometryType::eSPHERE
		PxcGetMaterialShapeShape,			//PxGeometryType::ePLANE
		PxcGetMaterialShapeShape,			//PxGeometryType::eCAPSULE
		PxcGetMaterialShapeShape,			//PxGeometryType::eBOX
		PxcGetMaterialShapeShape,			//PxGeometryType::eCONVEXMESH
		PxcGetMaterialShapeMesh,			//PxGeometryType::eTRIANGLEMESH	//not used: mesh always uses swept method for midphase.
		PxcGetMaterialShapeHeightField,		//PxGeometryType::eHEIGHTFIELD	//TODO: make HF midphase that will mask this
		
	},

	//PxGeometryType::ePLANE
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		PxcGetMaterialShapeShape,		//PxGeometryType::eCAPSULE
		PxcGetMaterialShapeShape,		//PxGeometryType::eBOX
		PxcGetMaterialShapeShape,		//PxGeometryType::eCONVEXMESH
		0,								//PxGeometryType::eTRIANGLEMESH
		0,								//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::eCAPSULE
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		PxcGetMaterialShapeShape,		//PxGeometryType::eCAPSULE
		PxcGetMaterialShapeShape,		//PxGeometryType::eBOX
		PxcGetMaterialShapeShape,		//PxGeometryType::eCONVEXMESH
		PxcGetMaterialShapeMesh,		//PxGeometryType::eTRIANGLEMESH		//not used: mesh always uses swept method for midphase.
		PxcGetMaterialShapeHeightField,		//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
	},

	//PxGeometryType::eBOX
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		0,								//PxGeometryType::eCAPSULE
		PxcGetMaterialShapeShape,				//PxGeometryType::eBOX
		PxcGetMaterialShapeShape,			//PxGeometryType::eCONVEXMESH
		PxcGetMaterialShapeMesh,				//PxGeometryType::eTRIANGLEMESH		//not used: mesh always uses swept method for midphase.
		PxcGetMaterialShapeHeightField,		//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
	},

	//PxGeometryType::eCONVEXMESH
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		0,								//PxGeometryType::eCAPSULE
		0,								//PxGeometryType::eBOX
		PxcGetMaterialShapeShape,			//PxGeometryType::eCONVEXMESH
		PxcGetMaterialShapeMesh,			//PxGeometryType::eTRIANGLEMESH		//not used: mesh always uses swept method for midphase.
		PxcGetMaterialShapeHeightField,	//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
	},

	//PxGeometryType::eTRIANGLEMESH
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		0,								//PxGeometryType::eCAPSULE
		0,								//PxGeometryType::eBOX
		0,								//PxGeometryType::eCONVEXMESH
		0,								//PxGeometryType::eTRIANGLEMESH
		0,								//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::eHEIGHTFIELD
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		0,								//PxGeometryType::eCAPSULE
		0,								//PxGeometryType::eBOX
		0,								//PxGeometryType::eCONVEXMESH
		0,								//PxGeometryType::eTRIANGLEMESH
		0,								//PxGeometryType::eHEIGHTFIELD
	},
		
};

}
