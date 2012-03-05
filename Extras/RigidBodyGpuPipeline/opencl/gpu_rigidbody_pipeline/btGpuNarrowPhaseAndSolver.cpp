/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Erwin Coumans

#include "btGpuNarrowphaseAndSolver.h"

//#include "CustomConvexShape.h"
//#include "CustomConvexPairCollision.h"
#include "LinearMath/btQuickprof.h"


//#include "BulletDynamics/Dynamics/btRigidBody.h"

#include "Adl/Adl.h"
#include "../../dynamics/basic_demo/Stubs/AdlMath.h"
#include "../../dynamics/basic_demo/Stubs/AdlContact4.h"
#include "../../dynamics/basic_demo/Stubs/AdlQuaternion.h"
#include "../../dynamics/basic_demo/Stubs/ChNarrowPhase.h"
#include "../../dynamics/basic_demo/Stubs/Solver.h"
#include <AdlPrimitives/Sort/RadixSort32.h>

int gpuBatchContacts = 1;

int numPairsOut =0;
struct CPUSolveData
{
	u32 m_n[adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT];
	u32 m_offset[adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT];
};


struct ParallelSolveData
{
	adl::Buffer<u32>* m_numConstraints;
	adl::Buffer<u32>* m_offsets;
};

struct	CustomDispatchData
{
	adl::DeviceCL* m_deviceCL;
	adl::Device* m_deviceHost;
	ShapeDataType m_ShapeBuffer;
	adl::HostBuffer<ConvexHeightField*>* m_shapePointers;

	adl::HostBuffer<int2>* m_pBufPairsCPU;

	adl::Buffer<int2>* m_convexPairsOutGPU;
	adl::Buffer<int2>* m_planePairs;

	adl::Buffer<Contact4>* m_pBufContactOutGPU;
	adl::HostBuffer<Contact4>* m_pBufContactOutCPU;
	adl::ChNarrowphase<adl::TYPE_CL>::Data* m_Data;
	


	adl::HostBuffer<RigidBodyBase::Body>* m_bodyBufferCPU;
	adl::Buffer<RigidBodyBase::Body>* m_bodyBufferGPU;

	adl::Buffer<RigidBodyBase::Inertia>*	m_inertiaBufferCPU;
	adl::Buffer<RigidBodyBase::Inertia>*	m_inertiaBufferGPU;

	adl::Solver<adl::TYPE_CL>::Data* m_solverDataGPU;
	SolverData		m_contactCGPU;
	void*			m_frictionCGPU;

	int m_numAcceleratedShapes;
	int m_numAcceleratedRigidBodies;
};


btGpuNarrowphaseAndSolver::btGpuNarrowphaseAndSolver(adl::DeviceCL* deviceCL)
	:m_internalData(0) ,m_planeBodyIndex(-1)
{

	if (deviceCL)
	{
		m_internalData = new CustomDispatchData();
		memset(m_internalData,0,sizeof(CustomDispatchData));

		adl::DeviceUtils::Config cfg;
		m_internalData->m_deviceCL = deviceCL;


		m_internalData->m_deviceHost = adl::DeviceUtils::allocate( adl::TYPE_HOST, cfg );
		m_internalData->m_pBufPairsCPU = new adl::HostBuffer<int2>(m_internalData->m_deviceHost, MAX_BROADPHASE_COLLISION_CL);

		m_internalData->m_convexPairsOutGPU = new adl::Buffer<int2>(m_internalData->m_deviceCL,MAX_BROADPHASE_COLLISION_CL);
		m_internalData->m_planePairs = new adl::Buffer<int2>(m_internalData->m_deviceCL,MAX_BROADPHASE_COLLISION_CL);
		
		m_internalData->m_pBufContactOutCPU = new adl::HostBuffer<Contact4>(m_internalData->m_deviceHost, MAX_BROADPHASE_COLLISION_CL);
		m_internalData->m_bodyBufferCPU = new adl::HostBuffer<RigidBodyBase::Body>(m_internalData->m_deviceHost, MAX_CONVEX_BODIES_CL);

		m_internalData->m_inertiaBufferCPU = new adl::Buffer<RigidBodyBase::Inertia>(m_internalData->m_deviceHost,MAX_CONVEX_BODIES_CL);
		m_internalData->m_pBufContactOutGPU = new adl::Buffer<Contact4>(m_internalData->m_deviceCL, MAX_BROADPHASE_COLLISION_CL);
		m_internalData->m_inertiaBufferGPU = new adl::Buffer<RigidBodyBase::Inertia>(m_internalData->m_deviceCL,MAX_CONVEX_BODIES_CL);

		m_internalData->m_solverDataGPU = adl::Solver<adl::TYPE_CL>::allocate( m_internalData->m_deviceCL, MAX_BROADPHASE_COLLISION_CL);
		m_internalData->m_bodyBufferGPU = new adl::Buffer<RigidBodyBase::Body>(m_internalData->m_deviceCL, MAX_CONVEX_BODIES_CL);
		m_internalData->m_Data = adl::ChNarrowphase<adl::TYPE_CL>::allocate(m_internalData->m_deviceCL);
//		m_internalData->m_DataCPU = adl::ChNarrowphase<adl::TYPE_HOST>::allocate(m_internalData->m_deviceHost);
		

		m_internalData->m_ShapeBuffer = adl::ChNarrowphase<adl::TYPE_CL>::allocateShapeBuffer(m_internalData->m_deviceCL, MAX_CONVEX_SHAPES_CL);	

		m_internalData->m_shapePointers = new adl::HostBuffer<ConvexHeightField*>(m_internalData->m_deviceHost,MAX_CONVEX_SHAPES_CL);

		m_internalData->m_numAcceleratedShapes = 0;
		m_internalData->m_numAcceleratedRigidBodies = 0;

		m_internalData->m_contactCGPU = adl::Solver<adl::TYPE_CL>::allocateConstraint4( m_internalData->m_deviceCL, MAX_BROADPHASE_COLLISION_CL);
		m_internalData->m_frictionCGPU = adl::Solver<adl::TYPE_CL>::allocateFrictionConstraint( m_internalData->m_deviceCL, MAX_BROADPHASE_COLLISION_CL);

	}
}

int btGpuNarrowphaseAndSolver::registerShape(ConvexHeightField* convexShape)
{
	(*m_internalData->m_shapePointers)[m_internalData->m_numAcceleratedShapes] = convexShape;
	adl::ChNarrowphase<adl::TYPE_CL>::setShape(m_internalData->m_ShapeBuffer, convexShape, m_internalData->m_numAcceleratedShapes, 0.01f);
	return m_internalData->m_numAcceleratedShapes++;
}

cl_mem	btGpuNarrowphaseAndSolver::getBodiesGpu()
{
	return (cl_mem)m_internalData->m_bodyBufferGPU->m_ptr;
}

cl_mem	btGpuNarrowphaseAndSolver::getBodyInertiasGpu()
{
	return (cl_mem)m_internalData->m_inertiaBufferGPU->m_ptr;
}


int btGpuNarrowphaseAndSolver::registerRigidBody(int shapeIndex, float mass, const float* position, const float* orientation , bool writeToGpu)
{
	assert(m_internalData->m_numAcceleratedRigidBodies< (MAX_CONVEX_BODIES_CL-1));

	RigidBodyBase::Body& body = m_internalData->m_bodyBufferCPU->m_ptr[m_internalData->m_numAcceleratedRigidBodies];

	float friction = 1.f;
	float restitution = 0.f;

	body.m_frictionCoeff = friction;
	body.m_restituitionCoeff = restitution;
	body.m_angVel = make_float4(0.f);
	body.m_linVel = make_float4(0.f);
	body.m_pos = make_float4(position[0],position[1],position[2],0.f);
	body.m_quat = make_float4(orientation[0],orientation[1],orientation[2],orientation[3]);
	body.m_shapeIdx = shapeIndex;
	if (shapeIndex<0)
	{
		body.m_shapeType = CollisionShape::SHAPE_PLANE;
		m_planeBodyIndex = m_internalData->m_numAcceleratedRigidBodies;
	} else
	{
		body.m_shapeType = CollisionShape::SHAPE_CONVEX_HEIGHT_FIELD;
	}
	
	body.m_invMass = mass? 1.f/mass : 0.f;

	if (writeToGpu)
		m_internalData->m_bodyBufferGPU->write(&body,1,m_internalData->m_numAcceleratedRigidBodies);

	RigidBodyBase::Inertia& shapeInfo = m_internalData->m_inertiaBufferCPU->m_ptr[m_internalData->m_numAcceleratedRigidBodies];

	if (mass==0.f)
	{
		shapeInfo.m_initInvInertia = mtZero();
		shapeInfo.m_invInertia = mtZero();
	} else
	{

		assert(body.m_shapeIdx>=0);

		//approximate using the aabb of the shape

		Aabb aabb = (*m_internalData->m_shapePointers)[shapeIndex]->m_aabb;
		float4 halfExtents = (aabb.m_max - aabb.m_min);

		float4 localInertia;

		float lx=2.f*halfExtents.x;
		float ly=2.f*halfExtents.y;
		float lz=2.f*halfExtents.z;

		localInertia = make_float4( (mass/12.0f) * (ly*ly + lz*lz),
			(mass/12.0f) * (lx*lx + lz*lz),
			(mass/12.0f) * (lx*lx + ly*ly));

		float4 invLocalInertia;
		invLocalInertia.x = 1.f/localInertia.x;
		invLocalInertia.y = 1.f/localInertia.y;
		invLocalInertia.z = 1.f/localInertia.z;
		invLocalInertia.w = 0.f;

		shapeInfo.m_initInvInertia = mtZero();
		shapeInfo.m_initInvInertia.m_row[0].x = invLocalInertia.x;
		shapeInfo.m_initInvInertia.m_row[1].y = invLocalInertia.y;
		shapeInfo.m_initInvInertia.m_row[2].z = invLocalInertia.z;

		Matrix3x3 m = qtGetRotationMatrix( body.m_quat);
		Matrix3x3 mT = mtTranspose( m );
		shapeInfo.m_invInertia = mtMul( mtMul( m, shapeInfo.m_initInvInertia ), mT );

	}

	if (writeToGpu)
		m_internalData->m_inertiaBufferGPU->write(&shapeInfo,1,m_internalData->m_numAcceleratedRigidBodies);
	return m_internalData->m_numAcceleratedRigidBodies++;
}

void	btGpuNarrowphaseAndSolver::writeAllBodiesToGpu()
{
	m_internalData->m_bodyBufferGPU->write(m_internalData->m_bodyBufferCPU->m_ptr,m_internalData->m_numAcceleratedRigidBodies);
	m_internalData->m_inertiaBufferGPU->write(	m_internalData->m_inertiaBufferCPU->m_ptr,m_internalData->m_numAcceleratedRigidBodies);
}



btGpuNarrowphaseAndSolver::~btGpuNarrowphaseAndSolver(void)
{
	if (m_internalData)
	{
		delete m_internalData->m_pBufPairsCPU;
		delete m_internalData->m_convexPairsOutGPU;
		delete m_internalData->m_planePairs;
		delete m_internalData->m_pBufContactOutGPU;
		delete m_internalData->m_inertiaBufferGPU;
		delete m_internalData->m_pBufContactOutCPU;
		delete m_internalData->m_shapePointers;
		adl::ChNarrowphase<adl::TYPE_CL>::deallocateShapeBuffer(m_internalData->m_ShapeBuffer);
		delete m_internalData->m_inertiaBufferCPU;
		adl::Solver<adl::TYPE_CL>::deallocateConstraint4( m_internalData->m_contactCGPU );
		adl::Solver<adl::TYPE_CL>::deallocateFrictionConstraint( m_internalData->m_frictionCGPU );

		delete m_internalData->m_bodyBufferGPU;
		adl::Solver<adl::TYPE_CL>::deallocate(	m_internalData->m_solverDataGPU);
		delete m_internalData->m_bodyBufferCPU;
		adl::ChNarrowphase<adl::TYPE_CL>::deallocate(m_internalData->m_Data);

		

		adl::DeviceUtils::deallocate(m_internalData->m_deviceHost);
		
		delete m_internalData;
	}

}





void btGpuNarrowphaseAndSolver::computeContactsAndSolver(cl_mem broadphasePairs, int numBroadphasePairs) 
{

	BT_PROFILE("computeContactsAndSolver");
	bool bGPU = (m_internalData != 0);
	int maxBodyIndex = m_internalData->m_numAcceleratedRigidBodies;

	if (!maxBodyIndex)
		return;
	int numOfConvexRBodies = maxBodyIndex;

	adl::ChNarrowphaseBase::Config cfgNP;
	cfgNP.m_collisionMargin = 0.01f;
	int nContactOut = 0;
	//printf("convexPairsOut.m_size = %d\n",m_internalData->m_convexPairsOutGPU->m_size);


	adl::Buffer<int2> broadphasePairsGPU;
	broadphasePairsGPU.m_ptr = (int2*)broadphasePairs;
	broadphasePairsGPU.m_size = numBroadphasePairs;
	broadphasePairsGPU.m_device = m_internalData->m_deviceCL;


	bool useCulling = true;
	if (useCulling)
	{
		BT_PROFILE("ChNarrowphase::culling");
		adl::DeviceUtils::waitForCompletion(m_internalData->m_deviceCL);

		numPairsOut = adl::ChNarrowphase<adl::TYPE_CL>::culling(
			m_internalData->m_Data, 
			&broadphasePairsGPU, 
			numBroadphasePairs,
			m_internalData->m_bodyBufferGPU, m_internalData->m_ShapeBuffer,
			m_internalData->m_convexPairsOutGPU,
			cfgNP);
	}

	{
		BT_PROFILE("ChNarrowphase::execute");
		if (useCulling)
		{
		
			if (m_planeBodyIndex>=0)
			{
				BT_PROFILE("ChNarrowphase:: plane versus convex");
				//todo: get rid of this dynamic allocation
				int2* hostPairs = new int2[m_internalData->m_numAcceleratedRigidBodies-1];
				int index=0;
				for (int i=0;i<m_internalData->m_numAcceleratedRigidBodies;i++)
				{
					if (i!=m_planeBodyIndex)
					{
						hostPairs[index].x = m_planeBodyIndex;
						hostPairs[index].y = i;
						index++;
					}
				}
				assert(m_internalData->m_numAcceleratedRigidBodies-1 == index);
				m_internalData->m_planePairs->write(hostPairs,index);
				adl::DeviceUtils::waitForCompletion(m_internalData->m_deviceCL);
				delete[]hostPairs;
				//convex versus plane
				adl::ChNarrowphase<adl::TYPE_CL>::execute(m_internalData->m_Data, m_internalData->m_planePairs, index, m_internalData->m_bodyBufferGPU, m_internalData->m_ShapeBuffer, 
					0,0,m_internalData->m_pBufContactOutGPU, nContactOut, cfgNP);
			}
		
			//convex versus convex
			adl::ChNarrowphase<adl::TYPE_CL>::execute(m_internalData->m_Data, m_internalData->m_convexPairsOutGPU,numPairsOut, m_internalData->m_bodyBufferGPU, m_internalData->m_ShapeBuffer, m_internalData->m_pBufContactOutGPU, nContactOut, cfgNP);
		} else
		{
			adl::ChNarrowphase<adl::TYPE_CL>::execute(m_internalData->m_Data, &broadphasePairsGPU, numBroadphasePairs, m_internalData->m_bodyBufferGPU, m_internalData->m_ShapeBuffer, m_internalData->m_pBufContactOutGPU, nContactOut, cfgNP);
		}

		adl::DeviceUtils::waitForCompletion(m_internalData->m_deviceCL);
	}
	
	if (!nContactOut)
		return;
	
	
	bool useSolver = true;//true;//false;

	if (useSolver)
	{
		float dt=1./60.;
		adl::SolverBase::ConstraintCfg csCfg( dt );
		csCfg.m_enableParallelSolve = true;
		csCfg.m_averageExtent = 0.2f;//@TODO m_averageObjExtent;
		csCfg.m_staticIdx = m_planeBodyIndex;

		
		bool exposeInternalBatchImplementation=true;

		adl::Solver<adl::TYPE_HOST>::Data* cpuSolverData = 0;
		if (exposeInternalBatchImplementation)
		{
			BT_PROFILE("Batching");

			cpuSolverData = adl::Solver<adl::TYPE_HOST>::allocate( m_internalData->m_deviceHost, nContactOut);

			adl::Buffer<Contact4>* contactsIn = m_internalData->m_pBufContactOutGPU;
			const adl::Buffer<RigidBodyBase::Body>* bodyBuf = m_internalData->m_bodyBufferGPU;
			void* additionalData = m_internalData->m_frictionCGPU;
			const adl::Buffer<RigidBodyBase::Inertia>* shapeBuf = m_internalData->m_inertiaBufferGPU;
			SolverData contactCOut = m_internalData->m_contactCGPU;
			int nContacts = nContactOut;

			bool useCPU=false;

			if (useCPU)
			{
				BT_PROFILE("CPU batch");
				{
					BT_PROFILE("CPU sortContacts2");
					sortContacts2( cpuSolverData, bodyBuf, contactsIn, additionalData, nContacts, csCfg );
				}

				CPUSolveData* dataCPU = (CPUSolveData*)cpuSolverData->m_parallelSolveData;
				{
					BT_PROFILE("CPU batchContacts2");

					adl::Buffer<u32> n; n.setRawPtr( cpuSolverData->m_device, dataCPU->m_n, adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT );
					adl::Buffer<u32> offsets; offsets.setRawPtr( cpuSolverData->m_device, dataCPU->m_offset, adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT );
					batchContacts2( cpuSolverData, contactsIn, nContacts, &n, &offsets, csCfg.m_staticIdx );
				}

				{
					BT_PROFILE("CPU convertToConstraints2");
					convertToConstraints2( cpuSolverData, bodyBuf, shapeBuf, contactsIn, contactCOut, additionalData, nContacts, csCfg );
				}

				{
					BT_PROFILE("CPU -> GPU copy");
					ParallelSolveData* dataGPU = (ParallelSolveData*)m_internalData->m_solverDataGPU->m_parallelSolveData;
					dataGPU->m_numConstraints->write(dataCPU->m_n,adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT);
					dataGPU->m_offsets->write(dataCPU->m_offset,adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT);
					adl::DeviceUtils::waitForCompletion( m_internalData->m_deviceCL);
				}

			}
			else
			{
				BT_PROFILE("GPU batch");

				adl::Solver<adl::TYPE_CL>::Data* data = m_internalData->m_solverDataGPU;

				{
					if( data->m_contactBuffer )
					{
						if( data->m_contactBuffer->getSize() < nContacts )
						{
							BT_PROFILE("delete data->m_contactBuffer;");
							delete data->m_contactBuffer;
							data->m_contactBuffer = 0;
						}
					}
					if( data->m_contactBuffer == 0 )
					{
						data->m_contactBuffer = new adl::Buffer<Contact4>( data->m_device, nContacts );
					}

					adl::Buffer<Contact4>* contactNative  = contactsIn;

					ParallelSolveData* nativeSolveData = (ParallelSolveData*)data->m_parallelSolveData;

					{

						ADLASSERT( data->m_device->m_type == adl::TYPE_CL );
						adl::Buffer<RigidBodyBase::Body>* bodyNative = adl::BufferUtils::map<adl::TYPE_CL, true>( data->m_device, bodyBuf );
						adl::Buffer<Contact4>* contactNative = adl::BufferUtils::map<adl::TYPE_CL, true>( data->m_device, contactsIn );

						const int sortAlignment = 512; // todo. get this out of sort
						if( csCfg.m_enableParallelSolve )
						{
							ParallelSolveData* nativeSolveData = (ParallelSolveData*)data->m_parallelSolveData;

							int sortSize = NEXTMULTIPLEOF( nContacts, sortAlignment );

							adl::Buffer<u32>* countsNative = nativeSolveData->m_numConstraints;//BufferUtils::map<TYPE_CL, false>( data->m_device, &countsHost );
							adl::Buffer<u32>* offsetsNative = nativeSolveData->m_offsets;//BufferUtils::map<TYPE_CL, false>( data->m_device, &offsetsHost );

							{	//	2. set cell idx
								BT_PROFILE("GPU set cell idx");
								struct CB
								{
									int m_nContacts;
									int m_staticIdx;
									float m_scale;
									int m_nSplit;
								};

								ADLASSERT( sortSize%64 == 0 );
								CB cdata;
								cdata.m_nContacts = nContacts;
								cdata.m_staticIdx = csCfg.m_staticIdx;
								cdata.m_scale = 1.f/(adl::SolverBase::N_OBJ_PER_SPLIT*csCfg.m_averageExtent);
								cdata.m_nSplit = adl::SolverBase::N_SPLIT;

								adl::Buffer<CB> constBuffer( data->m_device, 1, adl::BufferBase::BUFFER_CONST );
								adl::Launcher::BufferInfo bInfo[] = { adl::Launcher::BufferInfo( contactNative ), adl::Launcher::BufferInfo( bodyNative ), adl::Launcher::BufferInfo( data->m_sortDataBuffer ) };
								adl::Launcher launcher( data->m_device, data->m_setSortDataKernel );
								launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(adl::Launcher::BufferInfo) );
								launcher.setConst( constBuffer, cdata );
								launcher.launch1D( sortSize, 64 );
							}
							bool gpuRadixSort=true;
							if (gpuRadixSort)
							{	//	3. sort by cell idx
								BT_PROFILE("gpuRadixSort");
								int n = adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT;
								int sortBit = 32;
								//if( n <= 0xffff ) sortBit = 16;
								//if( n <= 0xff ) sortBit = 8;
								//adl::RadixSort<adl::TYPE_CL>::execute( data->m_sort, *data->m_sortDataBuffer, sortSize );
								adl::RadixSort32<adl::TYPE_CL>::execute( data->m_sort32, *data->m_sortDataBuffer, sortSize );

							} else
							{
								BT_PROFILE("cpu RadixSort");
								adl::HostBuffer<adl::SortData> sortData(m_internalData->m_deviceHost,nContacts);
								data->m_sortDataBuffer->read(sortData.m_ptr,nContacts);
								adl::DeviceUtils::waitForCompletion( m_internalData->m_deviceCL);

								adl::RadixSort<adl::TYPE_HOST>::Data* sData = adl::RadixSort<adl::TYPE_HOST>::allocate( m_internalData->m_deviceHost, nContacts );
								adl::RadixSort<adl::TYPE_HOST>::execute( sData, sortData, nContacts );
								adl::RadixSort<adl::TYPE_HOST>::deallocate( sData );

								data->m_sortDataBuffer->write(sortData.m_ptr,nContacts);
								adl::DeviceUtils::waitForCompletion( m_internalData->m_deviceCL);
							}



							bool gpuBoundSearch=true;
							if (gpuBoundSearch)
							{	//	4. find entries
								BT_PROFILE("gpuBoundSearch");
								adl::BoundSearch<adl::TYPE_CL>::execute( data->m_search, *data->m_sortDataBuffer, nContacts, *countsNative, 
									adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT, adl::BoundSearchBase::COUNT );

								adl::PrefixScan<adl::TYPE_CL>::execute( data->m_scan, *countsNative, *offsetsNative, 
									adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT );
							} else
							{
								BT_PROFILE("cpuBoundSearch");
								adl::HostBuffer<adl::SortData> sortData(m_internalData->m_deviceHost,nContacts);
								data->m_sortDataBuffer->read(sortData.m_ptr,nContacts);
								adl::DeviceUtils::waitForCompletion( m_internalData->m_deviceCL);

								adl::HostBuffer<u32> n0( m_internalData->m_deviceHost, adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT );
								adl::HostBuffer<u32> offset0( m_internalData->m_deviceHost, adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT );
								for(int i=0; i<adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT; i++)
								{
									n0[i] = 0;
									offset0[i] = 0;
								}

								for(int i=0; i<nContacts; i++)
								{
									int idx = sortData[i].m_key;
									assert(idx>=0);
									assert(idx<adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT);
									n0[idx]++;
								}

								//	scan
								int sum = 0;
								for(int i=0; i<adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT; i++)
								{
									offset0[i] = sum;
									sum += n0[i];
								}

								countsNative->write(n0.m_ptr,adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT);
								offsetsNative->write(offset0.m_ptr,adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT);
								adl::DeviceUtils::waitForCompletion( data->m_device );

							}
							{	//	5. sort constraints by cellIdx
								{
									BT_PROFILE("gpu m_reorderContactKernel");
									adl::Buffer<int4> constBuffer( data->m_device, 1, adl::BufferBase::BUFFER_CONST );

									int4 cdata; cdata.x = nContacts;
									adl::Launcher::BufferInfo bInfo[] = { adl::Launcher::BufferInfo( contactNative ), adl::Launcher::BufferInfo( data->m_contactBuffer ), adl::Launcher::BufferInfo( data->m_sortDataBuffer ) };
									adl::Launcher launcher( data->m_device, data->m_reorderContactKernel );
									launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(adl::Launcher::BufferInfo) );
									launcher.setConst( constBuffer, cdata );
									launcher.launch1D( nContacts, 64 );
								}
							}

						}

						adl::BufferUtils::unmap<false>( bodyNative, bodyBuf );
						adl::BufferUtils::unmap<false>( contactNative, contactsIn );

					}

					adl::DeviceUtils::waitForCompletion( m_internalData->m_deviceCL);

					{
						BT_PROFILE("gpu m_copyConstraintKernel");
						adl::Buffer<int4> constBuffer( data->m_device, 1, adl::BufferBase::BUFFER_CONST );
						int4 cdata; cdata.x = nContacts;
						adl::Launcher::BufferInfo bInfo[] = { adl::Launcher::BufferInfo( data->m_contactBuffer ), adl::Launcher::BufferInfo( contactNative ) };
						adl::Launcher launcher( data->m_device, data->m_copyConstraintKernel );
						launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(adl::Launcher::BufferInfo) );
						launcher.setConst( constBuffer, cdata );
						launcher.launch1D( nContacts, 64 );
						adl::DeviceUtils::waitForCompletion( data->m_device );
					}
					
					bool compareGPU = false;
					if (gpuBatchContacts)
					{
						BT_PROFILE("gpu batchContacts");
						adl::Solver<adl::TYPE_CL>::batchContacts( data, contactNative, nContacts, nativeSolveData->m_numConstraints, nativeSolveData->m_offsets, csCfg.m_staticIdx );
					}
					else
					{
						BT_PROFILE("cpu batchContacts2");
						cpuSolverData->m_parallelSolveData = 0;//
						ParallelSolveData* dataGPU = (ParallelSolveData*)m_internalData->m_solverDataGPU->m_parallelSolveData;
						adl::Buffer<u32> numConstraints(cpuSolverData->m_device,adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT);
						adl::Buffer<u32> offsets(cpuSolverData->m_device,adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT);

						{
							BT_PROFILE("gpu->cpu read m_numConstraints");
							dataGPU->m_numConstraints->read(numConstraints.m_ptr,adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT);
							dataGPU->m_offsets->read(offsets.m_ptr,adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT);
							adl::DeviceUtils::waitForCompletion( data->m_device );
						}

						adl::Buffer<u32> gpunumConstraints(cpuSolverData->m_device,adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT);
						adl::Buffer<u32> gpuoffsets(cpuSolverData->m_device,adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT);

						if (compareGPU)
						{
							adl::Buffer<Contact4> contactNativeCopy (data->m_device,contactNative->getSize());
							contactNativeCopy.write(*contactNative,contactNative->getSize());
							adl::DeviceUtils::waitForCompletion( data->m_device );

							adl::Buffer<u32> tmpNumGPU(data->m_device,adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT);
							adl::Buffer<u32> tmpOffsetGPU(data->m_device,adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT);
							tmpNumGPU.write(numConstraints.m_ptr,adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT);
							tmpOffsetGPU.write(offsets.m_ptr,adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT);
							adl::DeviceUtils::waitForCompletion( data->m_device );

							BT_PROFILE("gpu batchContacts");
							//adl::Solver<adl::TYPE_CL>::batchContacts( data, contactNative, nContacts, nativeSolveData->m_numConstraints, nativeSolveData->m_offsets, csCfg.m_staticIdx );
							adl::Solver<adl::TYPE_CL>::batchContacts( data, &contactNativeCopy, nContacts, &tmpNumGPU, &tmpOffsetGPU, csCfg.m_staticIdx );


							adl::DeviceUtils::waitForCompletion( data->m_device );

							//compare now
							tmpNumGPU.read(gpunumConstraints,adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT);
							tmpOffsetGPU.read(gpuoffsets,adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT);
							adl::DeviceUtils::waitForCompletion( data->m_device );

						}

						CPUSolveData* dataCPU = (CPUSolveData*)cpuSolverData->m_parallelSolveData;

						{
							BT_PROFILE("cpu batchContacts2");
							batchContacts2( cpuSolverData, contactNative, nContacts, &numConstraints, &offsets, csCfg.m_staticIdx );
						}


						if (compareGPU)
						{
							adl::DeviceUtils::waitForCompletion( data->m_device );
							dataGPU->m_numConstraints->write(numConstraints.m_ptr,adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT);
							dataGPU->m_offsets->write(offsets.m_ptr,adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT);
							adl::DeviceUtils::waitForCompletion( data->m_device );


							for (int i=0;i<adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT;i++)
							{
								if (gpunumConstraints.m_ptr[i] != numConstraints.m_ptr[i])
								{
									printf("numConstraints error at %d, expected %d got %d\n",i,numConstraints.m_ptr[i],gpunumConstraints.m_ptr[i]);
								}

								if (gpuoffsets.m_ptr[i] != offsets.m_ptr[i])
								{
									printf("numConstraints error at %d, expected %d got %d\n",i,offsets.m_ptr[i],gpuoffsets.m_ptr[i]);
								}

							}

						}

					}
					if (1)
					{
						BT_PROFILE("gpu convertToConstraints");
						adl::Solver<adl::TYPE_CL>::convertToConstraints( data, bodyBuf, shapeBuf, contactNative, contactCOut, additionalData, nContacts, csCfg );
						adl::DeviceUtils::waitForCompletion( data->m_device );
					}
					if (compareGPU)
					{
						adl::Buffer<Contact4> contactNativeCPU(cpuSolverData->m_device,contactNative->getSize());
						contactNative->read(contactNativeCPU,nContacts);
						adl::DeviceUtils::waitForCompletion( data->m_device );
						for (int i=0;i<nContacts;i++)
						{
							//if (contactNativeCopyCPU.m_ptr[i].m_frictionCoeffCmp !=45874)// contactNativeCPU.m_ptr[i].m_batchIdx != contactNativeCopyCPU.m_ptr[i].m_batchIdx)
							{
								//if (.m_friction!=45874
								//printf("not matching at %d, expected %d, got %d\n",i,contactNativeCPU.m_ptr[i].m_batchIdx,contactNativeCopyCPU.m_ptr[i].m_batchIdx);
							}
						}
					}

				}
			}

		} else
		{
			BT_PROFILE("GPU reorderConvertToConstraints");
			adl::Solver<adl::TYPE_CL>::reorderConvertToConstraints( 
				m_internalData->m_solverDataGPU, 
				m_internalData->m_bodyBufferGPU, 
				m_internalData->m_inertiaBufferGPU, 
				m_internalData->m_pBufContactOutGPU,
				m_internalData->m_contactCGPU, 
				m_internalData->m_frictionCGPU, 
				nContactOut, 
				csCfg );
			adl::DeviceUtils::waitForCompletion( m_internalData->m_deviceCL );
		}


		if (1)
		{
			BT_PROFILE("GPU solveContactConstraint");
			m_internalData->m_solverDataGPU->m_nIterations = 5;

			adl::Solver<adl::TYPE_CL>::solveContactConstraint( m_internalData->m_solverDataGPU, 
				m_internalData->m_bodyBufferGPU, 
				m_internalData->m_inertiaBufferGPU, 
				m_internalData->m_contactCGPU,
				0, 
				nContactOut );

			adl::DeviceUtils::waitForCompletion( m_internalData->m_deviceCL );
		}

		if (cpuSolverData)
			adl::Solver<adl::TYPE_HOST>::deallocate( cpuSolverData );

		if (0)
		{
			BT_PROFILE("read body velocities back to CPU");
			//read body updated linear/angular velocities back to CPU
			m_internalData->m_bodyBufferGPU->read(
				m_internalData->m_bodyBufferCPU->m_ptr,numOfConvexRBodies);
			adl::DeviceUtils::waitForCompletion( m_internalData->m_deviceCL );
		}
	}

}
