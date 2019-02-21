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

#include "PxRepXSimpleType.h"
#include "PxBase.h"
#include "PxCollection.h"
#include "PxVehicleMetaDataObjects.h"
#include "SnRepXSerializerImpl.h"
#include "PxVehicleSerialization.h"
#include "PxVehicleSuspWheelTire4.h"
#include "PxVehicleSuspLimitConstraintShader.h"
#include "PsFPU.h"


namespace physx
{
	using namespace Sn;
	
	template<typename TVehicleType>
	inline void* createVehicle( PxPhysics& physics, PxRigidDynamic* vehActor, 
									const PxVehicleWheelsSimData& wheelsData, const PxVehicleDriveSimData4W& driveData, const PxVehicleDriveSimDataNW& driveDataNW,
									const PxU32 numWheels, const PxU32 numNonDrivenWheels)
	{
		PX_UNUSED(physics);
		PX_UNUSED(vehActor);
		PX_UNUSED(wheelsData);
		PX_UNUSED(driveData);
		PX_UNUSED(driveDataNW);
		PX_UNUSED(numWheels);
		PX_UNUSED(numNonDrivenWheels);
		return NULL;
	}

	template<>
	inline void* createVehicle<PxVehicleDrive4W>(PxPhysics& physics, PxRigidDynamic* vehActor,
														   const PxVehicleWheelsSimData& wheelsData, const PxVehicleDriveSimData4W& driveData, const PxVehicleDriveSimDataNW& /*driveDataNW*/,
														   const PxU32 numWheels, const PxU32 numNonDrivenWheels)
	{
		PxVehicleDrive4W* vehDrive4W = PxVehicleDrive4W::allocate(numWheels);
		vehDrive4W->setup(&physics, vehActor->is<PxRigidDynamic>(), wheelsData, driveData, numNonDrivenWheels);
		return vehDrive4W;
	}
	
	template<>
	inline void* createVehicle<PxVehicleDriveTank>(PxPhysics& physics, PxRigidDynamic* vehActor,
														   const PxVehicleWheelsSimData& wheelsData, const PxVehicleDriveSimData4W& driveData, const PxVehicleDriveSimDataNW& /*driveDataNW*/,
														   const PxU32 numWheels, const PxU32 numNonDrivenWheels)
	{
		PxVehicleDriveTank* tank = PxVehicleDriveTank::allocate(numWheels);
		tank->setup(&physics, vehActor->is<PxRigidDynamic>(), wheelsData, driveData, numWheels - numNonDrivenWheels);
		return tank;
	}
	
	template<>
	inline void* createVehicle<PxVehicleDriveNW>(PxPhysics& physics, PxRigidDynamic* vehActor,
														   const PxVehicleWheelsSimData& wheelsData, const PxVehicleDriveSimData4W& /*driveData*/, const PxVehicleDriveSimDataNW& driveDataNW,
														   const PxU32 numWheels, const PxU32 numNonDrivenWheels)
	{
		PxVehicleDriveNW* vehDriveNW = PxVehicleDriveNW::allocate(numWheels);
		vehDriveNW->setup(&physics, vehActor->is<PxRigidDynamic>(), wheelsData, driveDataNW, numWheels - numNonDrivenWheels);
		return vehDriveNW;
	}
	
	template<>
	inline void* createVehicle<PxVehicleNoDrive>(PxPhysics& physics, PxRigidDynamic* vehActor,
		const PxVehicleWheelsSimData& wheelsData, const PxVehicleDriveSimData4W& /*driveData*/, const PxVehicleDriveSimDataNW& /*driveDataNW*/,
		const PxU32 numWheels, const PxU32 /*numNonDrivenWheels*/)
	{
		PxVehicleNoDrive* vehNoDrive = PxVehicleNoDrive::allocate(numWheels);
		vehNoDrive->setup(&physics, vehActor->is<PxRigidDynamic>(), wheelsData);
		return vehNoDrive;
	}

	template<typename TVehicleType>
	PxRepXObject PxVehicleRepXSerializer<TVehicleType>::fileToObject( XmlReader& inReader, XmlMemoryAllocator& inAllocator, PxRepXInstantiationArgs& inArgs, PxCollection* inCollection )
	{
		PxRigidActor* vehActor = NULL;
		readReference<PxRigidActor>( inReader, *inCollection, "PxRigidDynamicRef", vehActor );
		if ( vehActor == NULL )
			return PxRepXObject();

		PxU32 numWheels = 0;
		readProperty( inReader, "NumWheels", numWheels );
		if( numWheels == 0)
		{
			Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
				"PxSerialization::createCollectionFromXml: PxVehicleRepXSerializer: Xml field NumWheels is zero!");
			return PxRepXObject();
		}

		PxU32 numNonDrivenWheels = 0;
		readProperty( inReader, "NumNonDrivenWheels", numNonDrivenWheels );

		//change to numwheel
		PxVehicleWheelsSimData* wheelsSimData=PxVehicleWheelsSimData::allocate(numWheels);
		{
			inReader.pushCurrentContext();
			if ( inReader.gotoChild( "MWheelsSimData" ) )
			{
				readAllProperties( inArgs, inReader, wheelsSimData, inAllocator, *inCollection );
			}

			inReader.popCurrentContext();
		}

		PxVehicleDriveSimData4W driveSimData;
		{
			inReader.pushCurrentContext();
			if ( inReader.gotoChild( "MDriveSimData" ) )
			{
				readAllProperties( inArgs, inReader, &driveSimData, inAllocator, *inCollection );
			}

			inReader.popCurrentContext();
		}

		PxVehicleDriveSimDataNW nmSimData;
		{
			inReader.pushCurrentContext();
			if ( inReader.gotoChild( "MDriveSimDataNW" ) )
			{
				readAllProperties( inArgs, inReader, &driveSimData, inAllocator, *inCollection );
			}
			inReader.popCurrentContext();
		}
		TVehicleType* drive = static_cast<TVehicleType*>(createVehicle<TVehicleType>(inArgs.physics, vehActor->is<PxRigidDynamic>(), *wheelsSimData, driveSimData, nmSimData, numWheels, numNonDrivenWheels));
		readAllProperties( inArgs, inReader, drive, inAllocator, *inCollection );

		PxVehicleWheels4DynData* wheel4DynData = drive->mWheelsDynData.getWheel4DynData();
		PX_ASSERT( wheel4DynData );    
		for(PxU32 i=0;i<wheelsSimData->getNbWheels4();i++)
		{
			PxConstraint* constraint = wheel4DynData[i].getVehicletConstraintShader().getPxConstraint();
			if( constraint )
				inCollection->add(*constraint);
		}

		if( wheelsSimData )
			wheelsSimData->free();

		return PxCreateRepXObject(drive);
	}

	template<typename TVehicleType>
	void PxVehicleRepXSerializer<TVehicleType>::objectToFileImpl( const TVehicleType* drive, PxCollection* inCollection, XmlWriter& inWriter, MemoryBuffer& inTempBuffer, PxRepXInstantiationArgs& /*inArgs*/ )
	{
		PX_SIMD_GUARD; // denorm exception triggered in PxVehicleGearsDataGeneratedInfo::visitInstanceProperties on osx
		writeReference( inWriter, *inCollection, "PxRigidDynamicRef", drive->getRigidDynamicActor() );
		writeProperty( inWriter, *inCollection, inTempBuffer, "NumWheels", drive->mWheelsSimData.getNbWheels() );
		writeProperty( inWriter, *inCollection, inTempBuffer, "NumNonDrivenWheels", drive->getNbNonDrivenWheels());
		writeAllProperties( drive, inWriter, inTempBuffer, *inCollection );
	}

	PxVehicleNoDrive::PxVehicleNoDrive()
	: PxVehicleWheels(PxVehicleConcreteType::eVehicleNoDrive, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE)
	{}	

	PxVehicleDrive4W::PxVehicleDrive4W()
	: PxVehicleDrive(PxVehicleConcreteType::eVehicleDrive4W, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE)
	{}	

	PxVehicleDriveNW::PxVehicleDriveNW()
	: PxVehicleDrive(PxVehicleConcreteType::eVehicleDriveNW, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE)
	{}

	PxVehicleDriveTank::PxVehicleDriveTank() 
	: PxVehicleDrive(PxVehicleConcreteType::eVehicleDriveTank, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE)
	, mDriveModel(PxVehicleDriveTankControlModel::eSTANDARD) 
	{}

	// explicit template instantiations
	template struct PxVehicleRepXSerializer<PxVehicleDrive4W>;
	template struct PxVehicleRepXSerializer<PxVehicleDriveTank>;
	template struct PxVehicleRepXSerializer<PxVehicleDriveNW>;
	template struct PxVehicleRepXSerializer<PxVehicleNoDrive>;

} 
