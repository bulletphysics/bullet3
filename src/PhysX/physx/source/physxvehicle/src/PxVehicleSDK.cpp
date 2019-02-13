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

#include "PxVehicleSDK.h"
#include "PxPhysics.h"
#include "PxTolerancesScale.h"
#include "CmPhysXCommon.h"
#include "PsFoundation.h"
#include "PsUtilities.h"
#include "PxVehicleDrive4W.h"
#include "PxVehicleMetaDataObjects.h"
#include "PxVehicleSerialization.h"
#include "SnRepXSerializerImpl.h"
#include "PxSerializer.h"
#include "PxVehicleDriveTank.h"
#include "PxVehicleNoDrive.h"
#include "PxVehicleDriveNW.h"

namespace physx
{

void setVehicleToleranceScale(const PxTolerancesScale& ts);
void resetVehicleToleranceScale();
void setSerializationRegistryPtr(const PxSerializationRegistry* sr);
const PxSerializationRegistry* resetSerializationRegistryPtr();
void setVehicleDefaults();

bool PxInitVehicleSDK(PxPhysics& physics, PxSerializationRegistry* sr)
{
	PX_ASSERT(static_cast<Ps::Foundation*>(&physics.getFoundation()) == &Ps::Foundation::getInstance());
	Ps::Foundation::incRefCount();
	setVehicleToleranceScale(physics.getTolerancesScale());

	setVehicleDefaults();

	setSerializationRegistryPtr(sr);
	if(sr)
	{
		sr->registerRepXSerializer(PxVehicleConcreteType::eVehicleDrive4W,		PX_NEW_REPX_SERIALIZER(PxVehicleRepXSerializer<PxVehicleDrive4W>));
		sr->registerRepXSerializer(PxVehicleConcreteType::eVehicleDriveTank,	PX_NEW_REPX_SERIALIZER(PxVehicleRepXSerializer<PxVehicleDriveTank>));
		sr->registerRepXSerializer(PxVehicleConcreteType::eVehicleDriveNW,		PX_NEW_REPX_SERIALIZER(PxVehicleRepXSerializer<PxVehicleDriveNW>));
		sr->registerRepXSerializer(PxVehicleConcreteType::eVehicleNoDrive,		PX_NEW_REPX_SERIALIZER(PxVehicleRepXSerializer<PxVehicleNoDrive>));
		
		sr->registerSerializer(PxVehicleConcreteType::eVehicleDrive4W,   		PX_NEW_SERIALIZER_ADAPTER(PxVehicleDrive4W));
		sr->registerSerializer(PxVehicleConcreteType::eVehicleDriveTank, 		PX_NEW_SERIALIZER_ADAPTER(PxVehicleDriveTank));
		sr->registerSerializer(PxVehicleConcreteType::eVehicleNoDrive,   		PX_NEW_SERIALIZER_ADAPTER(PxVehicleNoDrive));
		sr->registerSerializer(PxVehicleConcreteType::eVehicleDriveNW,   		PX_NEW_SERIALIZER_ADAPTER(PxVehicleDriveNW));

		sr->registerBinaryMetaDataCallback(PxVehicleDrive4W::getBinaryMetaData);	
		sr->registerBinaryMetaDataCallback(PxVehicleDriveTank::getBinaryMetaData);	
		sr->registerBinaryMetaDataCallback(PxVehicleNoDrive::getBinaryMetaData);
		sr->registerBinaryMetaDataCallback(PxVehicleDriveNW::getBinaryMetaData);
	}
	return true;
}

void PxCloseVehicleSDK(PxSerializationRegistry* sr)
{
	Ps::Foundation::decRefCount();
	resetVehicleToleranceScale();

	setVehicleDefaults();

	if (sr != resetSerializationRegistryPtr())
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "PxCloseVehicleSDK called with different PxSerializationRegistry instance than PxInitVehicleSDK.");
		return;
	}

	if(sr)
	{
		PX_DELETE_SERIALIZER_ADAPTER(sr->unregisterSerializer(PxVehicleConcreteType::eVehicleDrive4W));
		PX_DELETE_SERIALIZER_ADAPTER(sr->unregisterSerializer(PxVehicleConcreteType::eVehicleDriveTank));
		PX_DELETE_SERIALIZER_ADAPTER(sr->unregisterSerializer(PxVehicleConcreteType::eVehicleNoDrive));
		PX_DELETE_SERIALIZER_ADAPTER(sr->unregisterSerializer(PxVehicleConcreteType::eVehicleDriveNW));
		
		PX_DELETE_REPX_SERIALIZER(sr->unregisterRepXSerializer(PxVehicleConcreteType::eVehicleDrive4W));
		PX_DELETE_REPX_SERIALIZER(sr->unregisterRepXSerializer(PxVehicleConcreteType::eVehicleDriveTank));
		PX_DELETE_REPX_SERIALIZER(sr->unregisterRepXSerializer(PxVehicleConcreteType::eVehicleNoDrive));
		PX_DELETE_REPX_SERIALIZER(sr->unregisterRepXSerializer(PxVehicleConcreteType::eVehicleDriveNW));
	}
}
/////////////////////////




}//physx

