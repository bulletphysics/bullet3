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

#include "PxPhysicsAPI.h"
#include "PxMetaDataObjects.h"
#include "CmIO.h"
#include "SnPxStreamOperators.h"
#include "PsUtilities.h"
#include "SnXmlImpl.h"			
#include "SnXmlSerializer.h"		
#include "SnXmlDeserializer.h"		
#include "SnRepXCoreSerializer.h"

using namespace physx::Sn;
namespace physx { 
	typedef PxReadOnlyPropertyInfo<PxPropertyInfoName::PxArticulationLink_InboundJoint, PxArticulationLink, PxArticulationJointBase *> TIncomingJointPropType;
		
	//*************************************************************
	//	Actual RepXSerializer implementations for PxMaterial
	//*************************************************************
	PxMaterial* PxMaterialRepXSerializer::allocateObject( PxRepXInstantiationArgs& inArgs )
	{
		return inArgs.physics.createMaterial(0, 0, 0);
	}

	PxRepXObject PxShapeRepXSerializer::fileToObject( XmlReader& inReader, XmlMemoryAllocator& inAllocator, PxRepXInstantiationArgs& inArgs, PxCollection* inCollection )
	{
		PxProfileAllocatorWrapper wrapper( inAllocator.getAllocator() );
		TReaderNameStack names( wrapper );
		PxProfileArray<PxU32> contexts( wrapper );
		bool hadError = false;
		RepXVisitorReader<PxShape> theVisitor( names, contexts, inArgs, inReader, NULL, inAllocator, *inCollection, hadError );

		Ps::Array<PxMaterial*> materials;
		PxGeometry* geometry = NULL; 
		parseShape( theVisitor, geometry, materials );
		if(hadError)
			return PxRepXObject();
		PxShape *theShape = inArgs.physics.createShape( *geometry, materials.begin(), Ps::to16(materials.size()) );

		switch(geometry->getType())
		{
		case PxGeometryType::eSPHERE :
			static_cast<PxSphereGeometry*>(geometry)->~PxSphereGeometry();
			break;
		case PxGeometryType::ePLANE :
			static_cast<PxPlaneGeometry*>(geometry)->~PxPlaneGeometry();
			break;
		case PxGeometryType::eCAPSULE :
			static_cast<PxCapsuleGeometry*>(geometry)->~PxCapsuleGeometry();
			break;
		case PxGeometryType::eBOX :
			static_cast<PxBoxGeometry*>(geometry)->~PxBoxGeometry();
			break;
		case PxGeometryType::eCONVEXMESH :
			static_cast<PxConvexMeshGeometry*>(geometry)->~PxConvexMeshGeometry();
			break;
		case PxGeometryType::eTRIANGLEMESH :
			static_cast<PxTriangleMeshGeometry*>(geometry)->~PxTriangleMeshGeometry();
			break;
		case PxGeometryType::eHEIGHTFIELD :
			static_cast<PxHeightFieldGeometry*>(geometry)->~PxHeightFieldGeometry();
			break;

		case PxGeometryType::eGEOMETRY_COUNT:
		case PxGeometryType::eINVALID:
			PX_ASSERT(0);			
		}		
		inAllocator.getAllocator().deallocate(geometry);

		bool ret = readAllProperties( inArgs, inReader, theShape, inAllocator, *inCollection );

		return ret ? PxCreateRepXObject(theShape) : PxRepXObject();
	}

	//*************************************************************
	//	Actual RepXSerializer implementations for PxTriangleMesh
	//*************************************************************

	template<typename TTriIndexElem>
	inline void writeTriangle( MemoryBuffer& inTempBuffer, const Triangle<TTriIndexElem>& inTriangle )
	{
		inTempBuffer << inTriangle.mIdx0 
			<< " " << inTriangle.mIdx1
			<< " " << inTriangle.mIdx2;
	}

	PxU32 materialAccess( const PxTriangleMesh* inMesh, PxU32 inIndex ) { return inMesh->getTriangleMaterialIndex( inIndex ); }
	template<typename TDataType>
	void writeDatatype( MemoryBuffer& inTempBuffer, const TDataType& inType ) { inTempBuffer << inType; }

	void PxBVH33TriangleMeshRepXSerializer::objectToFileImpl( const PxBVH33TriangleMesh* mesh, PxCollection* /*inCollection*/, XmlWriter& inWriter, MemoryBuffer& inTempBuffer, PxRepXInstantiationArgs& inArgs )
	{
		bool hasMatIndex = mesh->getTriangleMaterialIndex(0) != 0xffff;
		PxU32 numVertices = mesh->getNbVertices();
		const PxVec3* vertices = mesh->getVertices();
		writeBuffer( inWriter, inTempBuffer, 2, vertices, numVertices, "Points", writePxVec3 );
		bool isU16 = mesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES ? true : false;
		PxU32 triCount = mesh->getNbTriangles();
		const void* indices = mesh->getTriangles();
		if ( isU16 )
			writeBuffer( inWriter, inTempBuffer, 2, reinterpret_cast<const Triangle<PxU16>* >( indices ), triCount, "Triangles", writeTriangle<PxU16> );
		else
			writeBuffer( inWriter, inTempBuffer, 2, reinterpret_cast<const Triangle<PxU32>* >( indices ), triCount, "Triangles", writeTriangle<PxU32> );
		if ( hasMatIndex )
			writeBuffer( inWriter, inTempBuffer, 6, mesh, materialAccess, triCount, "materialIndices", writeDatatype<PxU32> );

		//Cooked stream			
		PxTriangleMeshDesc meshDesc;
		meshDesc.points.count = numVertices;
		meshDesc.points.data = vertices;
		meshDesc.points.stride = sizeof(PxVec3);
		meshDesc.triangles.count = triCount;
		meshDesc.triangles.data = indices;
		meshDesc.triangles.stride = isU16?3*sizeof(PxU16):3*sizeof(PxU32);

		if(isU16)
		{
			meshDesc.triangles.stride = sizeof(PxU16)*3;
			meshDesc.flags |= PxMeshFlag::e16_BIT_INDICES;
		}
		else
		{
			meshDesc.triangles.stride = sizeof(PxU32)*3;
		}

		if(hasMatIndex)
		{
			PxMaterialTableIndex* materialIndices = new PxMaterialTableIndex[triCount];
			for(PxU32 i = 0; i < triCount; i++)
				materialIndices[i] = mesh->getTriangleMaterialIndex(i);

			meshDesc.materialIndices.data = materialIndices;
			meshDesc.materialIndices.stride = sizeof(PxMaterialTableIndex);

		}

		if(inArgs.cooker != NULL)
		{
			TMemoryPoolManager theManager(mAllocator);
			MemoryBuffer theTempBuf( &theManager );
			theTempBuf.clear();
			inArgs.cooker->cookTriangleMesh( meshDesc, theTempBuf );

			writeBuffer( inWriter, inTempBuffer, 16, theTempBuf.mBuffer, theTempBuf.mWriteOffset, "CookedData", writeDatatype<PxU8> );
		}

		delete []meshDesc.materialIndices.data;
	}

	PxRepXObject PxBVH33TriangleMeshRepXSerializer::fileToObject( XmlReader& inReader, XmlMemoryAllocator& inAllocator, PxRepXInstantiationArgs& inArgs, PxCollection* /*inCollection*/ )
	{
		//We can't do a simple inverse; we *have* to cook data to get a mesh.
		PxTriangleMeshDesc theDesc;
		readStridedBufferProperty<PxVec3>( inReader, "points", theDesc.points, inAllocator);
		readStridedBufferProperty<Triangle<PxU32> >( inReader, "triangles", theDesc.triangles, inAllocator);
		PxU32 triCount;
		readStridedBufferProperty<PxMaterialTableIndex>( inReader, "materialIndices", theDesc.materialIndices, triCount, inAllocator);
		PxStridedData cookedData;
		cookedData.stride = sizeof(PxU8);
		PxU32 dataSize;
		readStridedBufferProperty<PxU8>( inReader, "CookedData", cookedData, dataSize, inAllocator);

		TMemoryPoolManager theManager(inAllocator.getAllocator());	
		MemoryBuffer theTempBuf( &theManager );

//		PxTriangleMesh* theMesh = NULL;			
		PxBVH33TriangleMesh* theMesh = NULL;			

		if(dataSize != 0)
		{				
			theTempBuf.write(cookedData.data, dataSize*sizeof(PxU8));
//			theMesh = inArgs.physics.createTriangleMesh( theTempBuf );
			theMesh = static_cast<PxBVH33TriangleMesh*>(inArgs.physics.createTriangleMesh( theTempBuf ));
		}

		if(theMesh == NULL)
		{
			PX_ASSERT(inArgs.cooker);
			theTempBuf.clear();

			{
				PxCookingParams params = inArgs.cooker->getParams();
				params.midphaseDesc = PxMeshMidPhase::eBVH33;
				inArgs.cooker->setParams(params);
			}

			inArgs.cooker->cookTriangleMesh( theDesc, theTempBuf );
//			theMesh = inArgs.physics.createTriangleMesh( theTempBuf );
			theMesh = static_cast<PxBVH33TriangleMesh*>(inArgs.physics.createTriangleMesh( theTempBuf ));
		}					

		return PxCreateRepXObject( theMesh );
	}

	void PxBVH34TriangleMeshRepXSerializer::objectToFileImpl( const PxBVH34TriangleMesh* mesh, PxCollection* /*inCollection*/, XmlWriter& inWriter, MemoryBuffer& inTempBuffer, PxRepXInstantiationArgs& inArgs )
	{
		bool hasMatIndex = mesh->getTriangleMaterialIndex(0) != 0xffff;
		PxU32 numVertices = mesh->getNbVertices();
		const PxVec3* vertices = mesh->getVertices();
		writeBuffer( inWriter, inTempBuffer, 2, vertices, numVertices, "Points", writePxVec3 );
		bool isU16 = mesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES ? true : false;
		PxU32 triCount = mesh->getNbTriangles();
		const void* indices = mesh->getTriangles();
		if ( isU16 )
			writeBuffer( inWriter, inTempBuffer, 2, reinterpret_cast<const Triangle<PxU16>* >( indices ), triCount, "Triangles", writeTriangle<PxU16> );
		else
			writeBuffer( inWriter, inTempBuffer, 2, reinterpret_cast<const Triangle<PxU32>* >( indices ), triCount, "Triangles", writeTriangle<PxU32> );
		if ( hasMatIndex )
			writeBuffer( inWriter, inTempBuffer, 6, mesh, materialAccess, triCount, "materialIndices", writeDatatype<PxU32> );

		//Cooked stream			
		PxTriangleMeshDesc meshDesc;
		meshDesc.points.count = numVertices;
		meshDesc.points.data = vertices;
		meshDesc.points.stride = sizeof(PxVec3);
		meshDesc.triangles.count = triCount;
		meshDesc.triangles.data = indices;
		meshDesc.triangles.stride = isU16?3*sizeof(PxU16):3*sizeof(PxU32);

		if(isU16)
		{
			meshDesc.triangles.stride = sizeof(PxU16)*3;
			meshDesc.flags |= PxMeshFlag::e16_BIT_INDICES;
		}
		else
		{
			meshDesc.triangles.stride = sizeof(PxU32)*3;
		}

		if(hasMatIndex)
		{
			PxMaterialTableIndex* materialIndices = new PxMaterialTableIndex[triCount];
			for(PxU32 i = 0; i < triCount; i++)
				materialIndices[i] = mesh->getTriangleMaterialIndex(i);

			meshDesc.materialIndices.data = materialIndices;
			meshDesc.materialIndices.stride = sizeof(PxMaterialTableIndex);

		}

		if(inArgs.cooker != NULL)
		{
			TMemoryPoolManager theManager(mAllocator);
			MemoryBuffer theTempBuf( &theManager );
			theTempBuf.clear();
			inArgs.cooker->cookTriangleMesh( meshDesc, theTempBuf );

			writeBuffer( inWriter, inTempBuffer, 16, theTempBuf.mBuffer, theTempBuf.mWriteOffset, "CookedData", writeDatatype<PxU8> );
		}

		delete []meshDesc.materialIndices.data;
	}

	PxRepXObject PxBVH34TriangleMeshRepXSerializer::fileToObject( XmlReader& inReader, XmlMemoryAllocator& inAllocator, PxRepXInstantiationArgs& inArgs, PxCollection* /*inCollection*/ )
	{
		//We can't do a simple inverse; we *have* to cook data to get a mesh.
		PxTriangleMeshDesc theDesc;
		readStridedBufferProperty<PxVec3>( inReader, "points", theDesc.points, inAllocator);
		readStridedBufferProperty<Triangle<PxU32> >( inReader, "triangles", theDesc.triangles, inAllocator);
		PxU32 triCount;
		readStridedBufferProperty<PxMaterialTableIndex>( inReader, "materialIndices", theDesc.materialIndices, triCount, inAllocator);
		PxStridedData cookedData;
		cookedData.stride = sizeof(PxU8);
		PxU32 dataSize;
		readStridedBufferProperty<PxU8>( inReader, "CookedData", cookedData, dataSize, inAllocator);

		TMemoryPoolManager theManager(inAllocator.getAllocator());	
		MemoryBuffer theTempBuf( &theManager );

//		PxTriangleMesh* theMesh = NULL;			
		PxBVH34TriangleMesh* theMesh = NULL;			

		if(dataSize != 0)
		{				
			theTempBuf.write(cookedData.data, dataSize*sizeof(PxU8));
//			theMesh = inArgs.physics.createTriangleMesh( theTempBuf );
			theMesh = static_cast<PxBVH34TriangleMesh*>(inArgs.physics.createTriangleMesh( theTempBuf ));
		}

		if(theMesh == NULL)
		{
			PX_ASSERT(inArgs.cooker);
			theTempBuf.clear();

			{
				PxCookingParams params = inArgs.cooker->getParams();
				params.midphaseDesc = PxMeshMidPhase::eBVH34;
				inArgs.cooker->setParams(params);
			}

			inArgs.cooker->cookTriangleMesh( theDesc, theTempBuf );
//			theMesh = inArgs.physics.createTriangleMesh( theTempBuf );
			theMesh = static_cast<PxBVH34TriangleMesh*>(inArgs.physics.createTriangleMesh( theTempBuf ));
		}					

		return PxCreateRepXObject(theMesh);
	}


	//*************************************************************
	//	Actual RepXSerializer implementations for PxHeightField
	//*************************************************************
	void PxHeightFieldRepXSerializer::objectToFileImpl( const PxHeightField* inHeightField, PxCollection* inCollection, XmlWriter& inWriter, MemoryBuffer& inTempBuffer, PxRepXInstantiationArgs& /*inArgs*/)
	{
		PxHeightFieldDesc theDesc;

		theDesc.nbRows					= inHeightField->getNbRows();
		theDesc.nbColumns				= inHeightField->getNbColumns();
		theDesc.format					= inHeightField->getFormat();
		theDesc.samples.stride			= inHeightField->getSampleStride();
		theDesc.samples.data			= NULL;
		theDesc.convexEdgeThreshold		= inHeightField->getConvexEdgeThreshold();
		theDesc.flags					= inHeightField->getFlags();

		PxU32 theCellCount = inHeightField->getNbRows() * inHeightField->getNbColumns();
		PxU32 theSampleStride = sizeof( PxHeightFieldSample );
		PxU32 theSampleBufSize = theCellCount * theSampleStride;
		PxHeightFieldSample* theSamples = reinterpret_cast< PxHeightFieldSample*> ( inTempBuffer.mManager->allocate( theSampleBufSize ) );
		inHeightField->saveCells( theSamples, theSampleBufSize );
		theDesc.samples.data = theSamples;
		writeAllProperties( &theDesc, inWriter, inTempBuffer, *inCollection );
		writeStridedBufferProperty<PxHeightFieldSample>( inWriter, inTempBuffer, "samples", theDesc.samples, theDesc.nbRows * theDesc.nbColumns, 6, writeHeightFieldSample);
		inTempBuffer.mManager->deallocate( reinterpret_cast<PxU8*>(theSamples) );
	}

	PxRepXObject PxHeightFieldRepXSerializer::fileToObject( XmlReader& inReader, XmlMemoryAllocator& inAllocator, PxRepXInstantiationArgs& inArgs, PxCollection* inCollection )
	{
		PX_ASSERT(inArgs.cooker);
		PxHeightFieldDesc theDesc;
		readAllProperties( inArgs, inReader, &theDesc, inAllocator, *inCollection );
		//Now read the data...
		PxU32 count = 0; //ignored becaues numRows and numColumns tells the story
		readStridedBufferProperty<PxHeightFieldSample>( inReader, "samples", theDesc.samples, count, inAllocator);
		PxHeightField* retval = inArgs.cooker->createHeightField( theDesc, inArgs.physics.getPhysicsInsertionCallback() );
		return PxCreateRepXObject(retval);
	}

	//*************************************************************
	//	Actual RepXSerializer implementations for PxConvexMesh
	//*************************************************************
	void PxConvexMeshRepXSerializer::objectToFileImpl( const PxConvexMesh* mesh, PxCollection* /*inCollection*/, XmlWriter& inWriter, MemoryBuffer& inTempBuffer, PxRepXInstantiationArgs& inArgs )
	{
		writeBuffer( inWriter, inTempBuffer, 2, mesh->getVertices(), mesh->getNbVertices(), "points", writePxVec3 );

		if(inArgs.cooker != NULL)
		{
			//Cache cooked Data
			PxConvexMeshDesc theDesc;
			theDesc.points.data = mesh->getVertices();
			theDesc.points.stride = sizeof(PxVec3);
			theDesc.points.count = mesh->getNbVertices();

			theDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
			TMemoryPoolManager theManager(mAllocator);
			MemoryBuffer theTempBuf( &theManager );
			inArgs.cooker->cookConvexMesh( theDesc, theTempBuf );

			writeBuffer( inWriter, inTempBuffer, 16, theTempBuf.mBuffer, theTempBuf.mWriteOffset, "CookedData", writeDatatype<PxU8> );
		}

	}

	//Conversion from scene object to descriptor.
	PxRepXObject PxConvexMeshRepXSerializer::fileToObject( XmlReader& inReader, XmlMemoryAllocator& inAllocator, PxRepXInstantiationArgs& inArgs, PxCollection* /*inCollection*/)
	{
		PxConvexMeshDesc theDesc;
		readStridedBufferProperty<PxVec3>( inReader, "points", theDesc.points, inAllocator);
		theDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

		PxStridedData cookedData;
		cookedData.stride = sizeof(PxU8);
		PxU32 dataSize;
		readStridedBufferProperty<PxU8>( inReader, "CookedData", cookedData, dataSize, inAllocator);

		TMemoryPoolManager theManager(inAllocator.getAllocator());
		MemoryBuffer theTempBuf( &theManager );

		PxConvexMesh* theMesh = NULL;

		if(dataSize != 0)
		{
			theTempBuf.write(cookedData.data, dataSize*sizeof(PxU8));
			theMesh = inArgs.physics.createConvexMesh( theTempBuf );
		}

		if(theMesh == NULL)
		{
			PX_ASSERT(inArgs.cooker);
			theTempBuf.clear();

			inArgs.cooker->cookConvexMesh( theDesc, theTempBuf );
			theMesh = inArgs.physics.createConvexMesh( theTempBuf );
		}					

		return PxCreateRepXObject(theMesh);
	}

	//*************************************************************
	//	Actual RepXSerializer implementations for PxRigidStatic
	//*************************************************************
	PxRigidStatic* PxRigidStaticRepXSerializer::allocateObject( PxRepXInstantiationArgs& inArgs )
	{
		return inArgs.physics.createRigidStatic( PxTransform(PxIdentity) );
	}

	//*************************************************************
	//	Actual RepXSerializer implementations for PxRigidDynamic
	//*************************************************************
	PxRigidDynamic* PxRigidDynamicRepXSerializer::allocateObject( PxRepXInstantiationArgs& inArgs )
	{
		return inArgs.physics.createRigidDynamic( PxTransform(PxIdentity) );
	}

	//*************************************************************
	//	Actual RepXSerializer implementations for PxArticulation
	//*************************************************************
	void PxArticulationRepXSerializer::objectToFileImpl( const PxArticulation* inObj, PxCollection* inCollection, XmlWriter& inWriter, MemoryBuffer& inTempBuffer, PxRepXInstantiationArgs& /*inArgs*/)
	{
		TNameStack nameStack( inTempBuffer.mManager->mWrapper );
		Sn::TArticulationLinkLinkMap linkMap( inTempBuffer.mManager->mWrapper );
		RepXVisitorWriter<PxArticulation> writer( nameStack, inWriter, inObj, inTempBuffer, *inCollection, &linkMap );
		RepXPropertyFilter<RepXVisitorWriter<PxArticulation> > theOp( writer );
		visitAllProperties<PxArticulation>( theOp );
	}
	PxArticulation* PxArticulationRepXSerializer::allocateObject( PxRepXInstantiationArgs& inArgs ) { return inArgs.physics.createArticulation(); }

	//*************************************************************
	//	Actual RepXSerializer implementations for PxAggregate
	//*************************************************************
	void PxAggregateRepXSerializer::objectToFileImpl( const PxAggregate* data, PxCollection* inCollection, XmlWriter& inWriter, MemoryBuffer& inTempBuffer, PxRepXInstantiationArgs& /*inArgs*/)
	{
		PxArticulationLink *link = NULL;
		inWriter.addAndGotoChild( "Actors" );
		for(PxU32 i = 0; i < data->getNbActors(); ++i)
		{
			PxActor* actor;

			if(data->getActors(&actor, 1, i))
			{
				link = actor->is<PxArticulationLink>();
			}

			if(link && !link->getInboundJoint() )
			{
				writeProperty( inWriter, *inCollection, inTempBuffer, "PxArticulationRef",  &link->getArticulation());			
			}
			else if( !link )
			{
				PxSerialObjectId theId = 0;

				theId = inCollection->getId( *actor );
				if( theId == 0 )
					theId = static_cast<uint64_t>(reinterpret_cast<size_t>(actor));

				writeProperty( inWriter, *inCollection, inTempBuffer, "PxActorRef", theId );			
			}
		}

		inWriter.leaveChild( );

		writeProperty( inWriter, *inCollection, inTempBuffer, "NumActors", data->getNbActors() );
		writeProperty( inWriter, *inCollection, inTempBuffer, "MaxNbActors", data->getMaxNbActors() );
		writeProperty( inWriter, *inCollection, inTempBuffer, "SelfCollision", data->getSelfCollision() );

		writeAllProperties( data, inWriter, inTempBuffer, *inCollection );
	}

	PxRepXObject PxAggregateRepXSerializer::fileToObject( XmlReader& inReader, XmlMemoryAllocator& inAllocator, PxRepXInstantiationArgs& inArgs, PxCollection* inCollection )
	{
		PxU32 numActors;
		readProperty( inReader, "NumActors", numActors );
		PxU32 maxNbActors;
		readProperty( inReader, "MaxNbActors", maxNbActors );
		
		bool selfCollision;
		bool ret = readProperty( inReader, "SelfCollision", selfCollision );

		PxAggregate* theAggregate = inArgs.physics.createAggregate(maxNbActors, selfCollision);
		ret &= readAllProperties( inArgs, inReader, theAggregate, inAllocator, *inCollection );

		inReader.pushCurrentContext();
		if ( inReader.gotoChild( "Actors" ) )
		{
			inReader.pushCurrentContext();
			for( bool matSuccess = inReader.gotoFirstChild(); matSuccess;
				matSuccess = inReader.gotoNextSibling() )
			{
				const char* actorType = inReader.getCurrentItemName();
				if ( 0 == physx::shdfnd::stricmp( actorType, "PxActorRef" ) ) 
				{
					PxActor *actor = NULL;
					ret &= readReference<PxActor>( inReader, *inCollection, actor );

					if(actor)
					{
						PxScene *currScene = actor->getScene();
						if(currScene)
						{
							currScene->removeActor(*actor);
						}
						theAggregate->addActor(*actor);
					}
				}
				else if ( 0 == physx::shdfnd::stricmp( actorType, "PxArticulationRef" ) ) 
				{
					PxArticulation* articulation = NULL;
					ret &= readReference<PxArticulation>( inReader, *inCollection, articulation );
					if(articulation)
					{
						PxScene *currScene = articulation->getScene();
						if(currScene)
						{
							currScene->removeArticulation(*articulation);
						}
						theAggregate->addArticulation(*articulation);
					}
				}	
			}
			inReader.popCurrentContext();
			inReader.leaveChild();
		}
		inReader.popCurrentContext();

		return ret ? PxCreateRepXObject(theAggregate) : PxRepXObject();
	}
}
