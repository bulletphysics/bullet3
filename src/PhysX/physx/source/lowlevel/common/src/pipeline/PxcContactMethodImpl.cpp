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
#include "PxcContactMethodImpl.h"

namespace physx
{

// PT: those prototypes shouldn't be public. Keep them here.

// Sphere - other
bool PxcContactSphereSphere				(GU_CONTACT_METHOD_ARGS);
bool PxcContactSpherePlane				(GU_CONTACT_METHOD_ARGS);
bool PxcContactSphereCapsule			(GU_CONTACT_METHOD_ARGS);
bool PxcContactSphereBox				(GU_CONTACT_METHOD_ARGS);
bool PxcContactSphereConvex				(GU_CONTACT_METHOD_ARGS);
bool PxcContactSphereMesh				(GU_CONTACT_METHOD_ARGS);
bool PxcContactSphereHeightField		(GU_CONTACT_METHOD_ARGS);

// Plane - other
bool PxcContactPlaneCapsule				(GU_CONTACT_METHOD_ARGS);
bool PxcContactPlaneBox					(GU_CONTACT_METHOD_ARGS);
bool PxcContactPlaneConvex				(GU_CONTACT_METHOD_ARGS);

// Capsule - other
bool PxcContactCapsuleCapsule			(GU_CONTACT_METHOD_ARGS);
bool PxcContactCapsuleBox				(GU_CONTACT_METHOD_ARGS);
bool PxcContactCapsuleConvex			(GU_CONTACT_METHOD_ARGS);
bool PxcContactCapsuleMesh				(GU_CONTACT_METHOD_ARGS);
bool PxcContactCapsuleHeightField		(GU_CONTACT_METHOD_ARGS);

// Box - other
bool PxcContactBoxBox					(GU_CONTACT_METHOD_ARGS);
bool PxcContactBoxConvex				(GU_CONTACT_METHOD_ARGS);
bool PxcContactBoxMesh					(GU_CONTACT_METHOD_ARGS);
bool PxcContactBoxHeightField			(GU_CONTACT_METHOD_ARGS);

// Convex - other
bool PxcContactConvexConvex				(GU_CONTACT_METHOD_ARGS);
bool PxcContactConvexMesh				(GU_CONTACT_METHOD_ARGS);
bool PxcContactConvexHeightField		(GU_CONTACT_METHOD_ARGS);


static bool PxcInvalidContactPair		(CONTACT_METHOD_ARGS_UNUSED)
{
	return false;
}

//PCM Sphere - other
bool PxcPCMContactSphereSphere			(GU_CONTACT_METHOD_ARGS);
bool PxcPCMContactSpherePlane			(GU_CONTACT_METHOD_ARGS);
bool PxcPCMContactSphereBox				(GU_CONTACT_METHOD_ARGS);
bool PxcPCMContactSphereCapsule			(GU_CONTACT_METHOD_ARGS);
bool PxcPCMContactSphereConvex			(GU_CONTACT_METHOD_ARGS);
bool PxcPCMContactSphereMesh			(GU_CONTACT_METHOD_ARGS);
bool PxcPCMContactSphereHeightField		(GU_CONTACT_METHOD_ARGS);

// Plane - other
bool PxcPCMContactPlaneCapsule			(GU_CONTACT_METHOD_ARGS);
bool PxcPCMContactPlaneBox				(GU_CONTACT_METHOD_ARGS);
bool PxcPCMContactPlaneConvex			(GU_CONTACT_METHOD_ARGS);

//PCM Capsule - other
bool PxcPCMContactCapsuleCapsule		(GU_CONTACT_METHOD_ARGS);
bool PxcPCMContactCapsuleBox			(GU_CONTACT_METHOD_ARGS);
bool PxcPCMContactCapsuleConvex			(GU_CONTACT_METHOD_ARGS);
bool PxcPCMContactCapsuleMesh			(GU_CONTACT_METHOD_ARGS);
bool PxcPCMContactCapsuleHeightField	(GU_CONTACT_METHOD_ARGS);

//PCM Box - other
bool PxcPCMContactBoxBox				(GU_CONTACT_METHOD_ARGS);
bool PxcPCMContactBoxConvex				(GU_CONTACT_METHOD_ARGS);
bool PxcPCMContactBoxMesh				(GU_CONTACT_METHOD_ARGS);
bool PxcPCMContactBoxHeightField		(GU_CONTACT_METHOD_ARGS);

//PCM Convex
bool PxcPCMContactConvexConvex			(GU_CONTACT_METHOD_ARGS);
bool PxcPCMContactConvexMesh			(GU_CONTACT_METHOD_ARGS);
bool PxcPCMContactConvexHeightField		(GU_CONTACT_METHOD_ARGS);


#define DYNAMIC_CONTACT_REGISTRATION(x) PxcInvalidContactPair
//#define DYNAMIC_CONTACT_REGISTRATION(x) x

//Table of contact methods for different shape-type combinations
PxcContactMethod g_ContactMethodTable[][PxGeometryType::eGEOMETRY_COUNT] = 
{
	//PxGeometryType::eSPHERE
	{
		PxcContactSphereSphere,			//PxGeometryType::eSPHERE
		PxcContactSpherePlane,			//PxGeometryType::ePLANE
		PxcContactSphereCapsule,		//PxGeometryType::eCAPSULE
		PxcContactSphereBox,			//PxGeometryType::eBOX
		PxcContactSphereConvex,			//PxGeometryType::eCONVEXMESH
		PxcContactSphereMesh,			//PxGeometryType::eTRIANGLEMESH
		DYNAMIC_CONTACT_REGISTRATION(PxcContactSphereHeightField),	//PxGeometryType::eHEIGHTFIELD	//TODO: make HF midphase that will mask this
		
	},

	//PxGeometryType::ePLANE
	{
		0,								//PxGeometryType::eSPHERE
		PxcInvalidContactPair,			//PxGeometryType::ePLANE
		PxcContactPlaneCapsule,			//PxGeometryType::eCAPSULE
		PxcContactPlaneBox,				//PxGeometryType::eBOX
		PxcContactPlaneConvex,			//PxGeometryType::eCONVEXMESH
		PxcInvalidContactPair,			//PxGeometryType::eTRIANGLEMESH
		PxcInvalidContactPair,			//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::eCAPSULE
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		PxcContactCapsuleCapsule,		//PxGeometryType::eCAPSULE
		PxcContactCapsuleBox,			//PxGeometryType::eBOX
		PxcContactCapsuleConvex,		//PxGeometryType::eCONVEXMESH
		PxcContactCapsuleMesh,			//PxGeometryType::eTRIANGLEMESH
		DYNAMIC_CONTACT_REGISTRATION(PxcContactCapsuleHeightField),	//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
	},

	//PxGeometryType::eBOX
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		0,								//PxGeometryType::eCAPSULE
		PxcContactBoxBox,				//PxGeometryType::eBOX
		PxcContactBoxConvex,			//PxGeometryType::eCONVEXMESH
		PxcContactBoxMesh,				//PxGeometryType::eTRIANGLEMESH
		DYNAMIC_CONTACT_REGISTRATION(PxcContactBoxHeightField),		//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
	},

	//PxGeometryType::eCONVEXMESH
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		0,								//PxGeometryType::eCAPSULE
		0,								//PxGeometryType::eBOX
		PxcContactConvexConvex,			//PxGeometryType::eCONVEXMESH
		PxcContactConvexMesh,			//PxGeometryType::eTRIANGLEMESH
		DYNAMIC_CONTACT_REGISTRATION(PxcContactConvexHeightField),	//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
	},

	//PxGeometryType::eTRIANGLEMESH
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		0,								//PxGeometryType::eCAPSULE
		0,								//PxGeometryType::eBOX
		0,								//PxGeometryType::eCONVEXMESH
		PxcInvalidContactPair,			//PxGeometryType::eTRIANGLEMESH
		PxcInvalidContactPair,			//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::eHEIGHTFIELD
	{
		0,								//PxGeometryType::eSPHERE
		0,								//PxGeometryType::ePLANE
		0,								//PxGeometryType::eCAPSULE
		0,								//PxGeometryType::eBOX
		0,								//PxGeometryType::eCONVEXMESH
		0,								//PxGeometryType::eTRIANGLEMESH
		PxcInvalidContactPair,			//PxGeometryType::eHEIGHTFIELD
	},
};


//#if	PERSISTENT_CONTACT_MANIFOLD
//Table of contact methods for different shape-type combinations
PxcContactMethod g_PCMContactMethodTable[][PxGeometryType::eGEOMETRY_COUNT] = 
{
	//PxGeometryType::eSPHERE
	{
		PxcPCMContactSphereSphere,										//PxGeometryType::eSPHERE
		PxcPCMContactSpherePlane,										//PxGeometryType::ePLANE
		PxcPCMContactSphereCapsule,										//PxGeometryType::eCAPSULE
		PxcPCMContactSphereBox,											//PxGeometryType::eBOX
		PxcPCMContactSphereConvex,										//PxGeometryType::eCONVEXMESH
		PxcPCMContactSphereMesh,										//PxGeometryType::eTRIANGLEMESH
		DYNAMIC_CONTACT_REGISTRATION(PxcPCMContactSphereHeightField),	//PxGeometryType::eHEIGHTFIELD	//TODO: make HF midphase that will mask this	
	},

	//PxGeometryType::ePLANE
	{
		0,															//PxGeometryType::eSPHERE
		PxcInvalidContactPair,										//PxGeometryType::ePLANE
		PxcPCMContactPlaneCapsule,									//PxGeometryType::eCAPSULE
		PxcPCMContactPlaneBox,										//PxGeometryType::eBOX  
		PxcPCMContactPlaneConvex,										//PxGeometryType::eCONVEXMESH
		PxcInvalidContactPair,										//PxGeometryType::eTRIANGLEMESH
		PxcInvalidContactPair,										//PxGeometryType::eHEIGHTFIELD
	},  

	//PxGeometryType::eCAPSULE
	{
		0,																//PxGeometryType::eSPHERE
		0,																//PxGeometryType::ePLANE
		PxcPCMContactCapsuleCapsule,									//PxGeometryType::eCAPSULE
		PxcPCMContactCapsuleBox,										//PxGeometryType::eBOX
		PxcPCMContactCapsuleConvex,										//PxGeometryType::eCONVEXMESH
		PxcPCMContactCapsuleMesh,										//PxGeometryType::eTRIANGLEMESH	
		DYNAMIC_CONTACT_REGISTRATION(PxcPCMContactCapsuleHeightField),	//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
	},

	//PxGeometryType::eBOX
	{
		0,																//PxGeometryType::eSPHERE
		0,																//PxGeometryType::ePLANE
		0,																//PxGeometryType::eCAPSULE
		PxcPCMContactBoxBox,											//PxGeometryType::eBOX
		PxcPCMContactBoxConvex,											//PxGeometryType::eCONVEXMESH
		PxcPCMContactBoxMesh,											//PxGeometryType::eTRIANGLEMESH
		DYNAMIC_CONTACT_REGISTRATION(PxcPCMContactBoxHeightField),		//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this

	},

	//PxGeometryType::eCONVEXMESH
	{
		0,																	//PxGeometryType::eSPHERE
		0,																	//PxGeometryType::ePLANE
		0,																	//PxGeometryType::eCAPSULE
		0,																	//PxGeometryType::eBOX
		PxcPCMContactConvexConvex,											//PxGeometryType::eCONVEXMESH
		PxcPCMContactConvexMesh,											//PxGeometryType::eTRIANGLEMESH
		DYNAMIC_CONTACT_REGISTRATION(PxcPCMContactConvexHeightField),		//PxGeometryType::eHEIGHTFIELD		//TODO: make HF midphase that will mask this
	},

	//PxGeometryType::eTRIANGLEMESH
	{
		0,																//PxGeometryType::eSPHERE
		0,																//PxGeometryType::ePLANE
		0,																//PxGeometryType::eCAPSULE
		0,																//PxGeometryType::eBOX
		0,																//PxGeometryType::eCONVEXMESH
		PxcInvalidContactPair,											//PxGeometryType::eTRIANGLEMESH
		PxcInvalidContactPair,											//PxGeometryType::eHEIGHTFIELD
	},   

	//PxGeometryType::eHEIGHTFIELD
	{
		0,																//PxGeometryType::eSPHERE
		0,																//PxGeometryType::ePLANE
		0,																//PxGeometryType::eCAPSULE
		0,																//PxGeometryType::eBOX
		0,																//PxGeometryType::eCONVEXMESH
		0,																//PxGeometryType::eTRIANGLEMESH
		PxcInvalidContactPair,											//PxGeometryType::eHEIGHTFIELD
	},

};
}
