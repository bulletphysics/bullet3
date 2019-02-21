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
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#include "PxPhysicsAPI.h"
#include "extensions/PxExtensionsAPI.h"
#include "PxVehicleMetaDataObjects.h"
#include "PxExtensionMetaDataObjects.h"

namespace physx
{
	inline void SetMFrictionVsSlipGraph( PxVehicleTireData* inTireData, PxU32 idx1, PxU32 idx2, PxReal val ) { inTireData->mFrictionVsSlipGraph[idx1][idx2] = val; }
	inline PxReal GetMFrictionVsSlipGraph( const PxVehicleTireData* inTireData, PxU32 idx1, PxU32 idx2 ) 
	{ 
		return inTireData->mFrictionVsSlipGraph[idx1][idx2]; 
	}
	PX_PHYSX_CORE_API MFrictionVsSlipGraphProperty::MFrictionVsSlipGraphProperty()
									: PxExtendedDualIndexedPropertyInfo<PxVehiclePropertyInfoName::PxVehicleTireData_MFrictionVsSlipGraph
																, PxVehicleTireData
																, PxU32
																, PxU32
																, PxReal> ( "MFrictionVsSlipGraph", SetMFrictionVsSlipGraph, GetMFrictionVsSlipGraph, 3, 2 )
	{

	}
		
	inline PxU32 GetNbWheels( const PxVehicleWheels* inStats ) { return inStats->mWheelsSimData.getNbWheels(); }

	inline PxU32 GetNbTorqueCurvePair( const PxVehicleEngineData* inStats ) { return inStats->mTorqueCurve.getNbDataPairs(); }
		
    
	inline PxReal getXTorqueCurvePair( const PxVehicleEngineData* inStats, PxU32 index)
	{ 
		return inStats->mTorqueCurve.getX(index);
	}
	inline PxReal getYTorqueCurvePair( const PxVehicleEngineData* inStats, PxU32 index)
	{ 
		return inStats->mTorqueCurve.getY(index);
	}
	
	void addTorqueCurvePair(PxVehicleEngineData* inStats, const PxReal x, const PxReal y) 
	{ 
		inStats->mTorqueCurve.addPair(x, y); 
	}
	
	void clearTorqueCurvePair(PxVehicleEngineData* inStats) 
	{ 
		inStats->mTorqueCurve.clear(); 
	}

	PX_PHYSX_CORE_API MTorqueCurveProperty::MTorqueCurveProperty()
		: PxFixedSizeLookupTablePropertyInfo<PxVehiclePropertyInfoName::PxVehicleEngineData_MTorqueCurve
				, PxVehicleEngineData
				, PxU32
				, PxReal>("MTorqueCurve", getXTorqueCurvePair, getYTorqueCurvePair, GetNbTorqueCurvePair, addTorqueCurvePair, clearTorqueCurvePair)
	{
	}


}

