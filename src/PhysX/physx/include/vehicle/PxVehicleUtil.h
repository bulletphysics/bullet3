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

#ifndef PX_VEHICLE_UTILHELPER_H
#define PX_VEHICLE_UTILHELPER_H
/** \addtogroup vehicle
  @{
*/

#include "foundation/Px.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

struct PxVehicleWheelQueryResult;

/**
\brief Test if all wheels of a vehicle are in the air by querying the wheel query data 
stored in the last call to PxVehicleUpdates. If all wheels are in the air then true is returned.  

\note False is returned if any wheel can reach to the ground.

\note If vehWheelQueryResults.wheelQueryResults is NULL or vehWheelQueryResults.nbWheelQueryResults is 0 then true is returned.
This function does not account for wheels that have been disabled since the last execution of PxVehicleUpdates so it is possible
that wheels disabled more recently than the last call to PxVehicleUpdates report are treated as touching the ground.

\return True if the vehicle is in the air, false if any wheel is touching the ground.
*/
bool PxVehicleIsInAir(const PxVehicleWheelQueryResult& vehWheelQueryResults);

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif //PX_VEHICLE_UTILHELPER_H
