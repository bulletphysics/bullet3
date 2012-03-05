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
//Originally written by Takahiro Harada


class SolverInl
{
public:
	typedef SolverBase::ConstraintData ConstraintData;


	static
	__forceinline
	void setLinearAndAngular(const MYF4& n, const MYF4& r0, const MYF4& r1,
							 MYF4& linear, MYF4& angular0, MYF4& angular1)
	{
		linear = -n;
		angular0 = -cross3(r0, n);
		angular1 = cross3(r1, n);
	}

	static
	__forceinline
	float calcJacCoeff(const MYF4& linear0, const MYF4& linear1, const MYF4& angular0, const MYF4& angular1,
					  float invMass0, const Matrix3x3& invInertia0, float invMass1, const Matrix3x3& invInertia1)
	{
		//	linear0,1 are normlized
		float jmj0 = invMass0;//dot3F4(linear0, linear0)*invMass0;
		float jmj1 = dot3F4(mtMul3(angular0,invInertia0), angular0);
		float jmj2 = invMass1;//dot3F4(linear1, linear1)*invMass1;
		float jmj3 = dot3F4(mtMul3(angular1,invInertia1), angular1);
		return -1.f/(jmj0+jmj1+jmj2+jmj3);
	}
	static
	__forceinline
	float calcRelVel(const MYF4& l0, const MYF4& l1, const MYF4& a0, const MYF4& a1, 
					 const MYF4& linVel0, const MYF4& angVel0, const MYF4& linVel1, const MYF4& angVel1)
	{
		return dot3F4(l0, linVel0) + dot3F4(a0, angVel0) + dot3F4(l1, linVel1) + dot3F4(a1, angVel1);
	}

	static
	__forceinline
	void setConstraint4( const MYF4& posA, const MYF4& linVelA, const MYF4& angVelA, float invMassA, const Matrix3x3& invInertiaA, 
		const MYF4& posB, const MYF4& linVelB, const MYF4& angVelB, float invMassB, const Matrix3x3& invInertiaB, 
		const Contact4& src, const SolverBase::ConstraintCfg& cfg, 
		Constraint4& dstC )
	{
		dstC.m_bodyA = (u32)src.m_bodyAPtr;
		dstC.m_bodyB = (u32)src.m_bodyBPtr;

		float dtInv = 1.f/cfg.m_dt;
		for(int ic=0; ic<4; ic++)
		{
			dstC.m_appliedRambdaDt[ic] = 0.f;
		}
		dstC.m_fJacCoeffInv[0] = dstC.m_fJacCoeffInv[1] = 0.f;


		const MYF4& n = src.m_worldNormal;
		dstC.m_linear = -n;
		dstC.setFrictionCoeff( src.getFrictionCoeff() );
		for(int ic=0; ic<4; ic++)
		{
			MYF4 r0 = src.m_worldPos[ic] - posA;
			MYF4 r1 = src.m_worldPos[ic] - posB;

			if( ic >= src.getNPoints() )
			{
				dstC.m_jacCoeffInv[ic] = 0.f;
				continue;
			}

			float relVelN;
			{
				MYF4 linear, angular0, angular1;
				setLinearAndAngular(n, r0, r1, linear, angular0, angular1);

				dstC.m_jacCoeffInv[ic] = calcJacCoeff(linear, -linear, angular0, angular1,
					invMassA, invInertiaA, invMassB, invInertiaB );

				relVelN = calcRelVel(linear, -linear, angular0, angular1,
					linVelA, angVelA, linVelB, angVelB);

				float e = src.getRestituitionCoeff();
				if( relVelN*relVelN < 0.004f ) e = 0.f;

				dstC.m_b[ic] = e*relVelN;
				dstC.m_b[ic] += (src.getPenetration(ic) + cfg.m_positionDrift)*cfg.m_positionConstraintCoeff*dtInv;
				dstC.m_appliedRambdaDt[ic] = 0.f;
			}
		}

		if( src.getNPoints() > 1 )
		{	//	prepare friction
			MYF4 center = MAKE_MYF4(0.f);
			for(int i=0; i<src.getNPoints(); i++) center += src.m_worldPos[i];
			center /= (float)src.getNPoints();

			MYF4 tangent[2];
			tangent[0] = cross3( src.m_worldNormal, src.m_worldPos[0]-center );
			tangent[1] = cross3( tangent[0], src.m_worldNormal );
			tangent[0] = normalize3( tangent[0] );
			tangent[1] = normalize3( tangent[1] );
			MYF4 r[2];
			r[0] = center - posA;
			r[1] = center - posB;

			for(int i=0; i<2; i++)
			{
				MYF4 linear, angular0, angular1;
				setLinearAndAngular(tangent[i], r[0], r[1], linear, angular0, angular1);

				dstC.m_fJacCoeffInv[i] = calcJacCoeff(linear, -linear, angular0, angular1,
					invMassA, invInertiaA, invMassB, invInertiaB );
				dstC.m_fAppliedRambdaDt[i] = 0.f;
			}
			dstC.m_center = center;
		}
		else
		{
			//	single point constraint
		}

		for(int i=0; i<4; i++)
		{
			if( i<src.getNPoints() )
			{
				dstC.m_worldPos[i] = src.m_worldPos[i];
			}
			else
			{
				dstC.m_worldPos[i] = MAKE_MYF4(0.f);
			}
		}
	}

/*
	struct Constraint4
	{
		float4 m_linear;			X
		float4 m_angular0[4];		X
		float4 m_angular1[4];		center
		float m_jacCoeffInv[4];		[0,1]
		float m_b[4];				X
		float m_appliedRambdaDt[4];	[0,1]

		void* m_bodyAPtr;			X
		void* m_bodyBPtr;			X
	};
*/
	static
	__inline
	void solveFriction(Constraint4& cs, 
		const MYF4& posA, MYF4& linVelA, MYF4& angVelA, float invMassA, const Matrix3x3& invInertiaA,
		const MYF4& posB, MYF4& linVelB, MYF4& angVelB, float invMassB, const Matrix3x3& invInertiaB, 
		float maxRambdaDt[4], float minRambdaDt[4])
	{
		if( cs.m_fJacCoeffInv[0] == 0 && cs.m_fJacCoeffInv[0] == 0 ) return;
		const MYF4& center = cs.m_center;

		MYF4 n = -cs.m_linear;

		MYF4 tangent[2];
		tangent[0] = cross3( n, cs.m_worldPos[0]-center );
		tangent[1] = cross3( tangent[0], n );
		tangent[0] = normalize3( tangent[0] );
		tangent[1] = normalize3( tangent[1] );

		MYF4 angular0, angular1, linear;
		MYF4 r0 = center - posA;
		MYF4 r1 = center - posB;
		for(int i=0; i<2; i++)
		{
			setLinearAndAngular( tangent[i], r0, r1, linear, angular0, angular1 );
			float rambdaDt = calcRelVel(linear, -linear, angular0, angular1,
				linVelA, angVelA, linVelB, angVelB );
			rambdaDt *= cs.m_fJacCoeffInv[i];

				{
					float prevSum = cs.m_fAppliedRambdaDt[i];
					float updated = prevSum;
					updated += rambdaDt;
					updated = max2( updated, minRambdaDt[i] );
					updated = min2( updated, maxRambdaDt[i] );
					rambdaDt = updated - prevSum;
					cs.m_fAppliedRambdaDt[i] = updated;
				}

			MYF4 linImp0 = invMassA*linear*rambdaDt;
			MYF4 linImp1 = invMassB*(-linear)*rambdaDt;
			MYF4 angImp0 = mtMul1(invInertiaA, angular0)*rambdaDt;
			MYF4 angImp1 = mtMul1(invInertiaB, angular1)*rambdaDt;

			linVelA += linImp0;
			angVelA += angImp0;
			linVelB += linImp1;
			angVelB += angImp1;
		}

		{	//	angular damping for point constraint
			MYF4 ab = normalize3( posB - posA );
			MYF4 ac = normalize3( center - posA );
			if( dot3F4( ab, ac ) > 0.95f || (invMassA == 0.f || invMassB == 0.f))
			{
				float angNA = dot3F4( n, angVelA );
				float angNB = dot3F4( n, angVelB );

				angVelA -= (angNA*0.1f)*n;
				angVelB -= (angNB*0.1f)*n;
			}
		}
	}

	template<bool JACOBI>
	static
	__inline
	void solveContact(Constraint4& cs, 
		const MYF4& posA, MYF4& linVelA, MYF4& angVelA, float invMassA, const Matrix3x3& invInertiaA,
		const MYF4& posB, MYF4& linVelB, MYF4& angVelB, float invMassB, const Matrix3x3& invInertiaB, 
		float maxRambdaDt[4], float minRambdaDt[4])
	{
		MYF4 dLinVelA = MAKE_MYF4(0.f);
		MYF4 dAngVelA = MAKE_MYF4(0.f);
		MYF4 dLinVelB = MAKE_MYF4(0.f);
		MYF4 dAngVelB = MAKE_MYF4(0.f);

		for(int ic=0; ic<4; ic++)
		{
			//	dont necessary because this makes change to 0
			if( cs.m_jacCoeffInv[ic] == 0.f ) continue;

			{
				MYF4 angular0, angular1, linear;
				MYF4 r0 = cs.m_worldPos[ic] - posA;
				MYF4 r1 = cs.m_worldPos[ic] - posB;
				setLinearAndAngular( -cs.m_linear, r0, r1, linear, angular0, angular1 );

				float rambdaDt = calcRelVel(cs.m_linear, -cs.m_linear, angular0, angular1,
					linVelA, angVelA, linVelB, angVelB ) + cs.m_b[ic];
				rambdaDt *= cs.m_jacCoeffInv[ic];

				{
					float prevSum = cs.m_appliedRambdaDt[ic];
					float updated = prevSum;
					updated += rambdaDt;
					updated = max2( updated, minRambdaDt[ic] );
					updated = min2( updated, maxRambdaDt[ic] );
					rambdaDt = updated - prevSum;
					cs.m_appliedRambdaDt[ic] = updated;
				}

				MYF4 linImp0 = invMassA*linear*rambdaDt;
				MYF4 linImp1 = invMassB*(-linear)*rambdaDt;
				MYF4 angImp0 = mtMul1(invInertiaA, angular0)*rambdaDt;
				MYF4 angImp1 = mtMul1(invInertiaB, angular1)*rambdaDt;

				if( JACOBI )
				{
					dLinVelA += linImp0;
					dAngVelA += angImp0;
					dLinVelB += linImp1;
					dAngVelB += angImp1;
				}
				else
				{
					linVelA += linImp0;
					angVelA += angImp0;
					linVelB += linImp1;
					angVelB += angImp1;
				}
			}
		}

		if( JACOBI )
		{
			linVelA += dLinVelA;
			angVelA += dAngVelA;
			linVelB += dLinVelB;
			angVelB += dAngVelB;
		}
	}

	enum
	{
		N_SPLIT = SolverBase::N_SPLIT,
	};

	//	for parallel solve
	struct ParallelSolveData
	{
		u32 m_n[N_SPLIT*N_SPLIT];
		u32 m_offset[N_SPLIT*N_SPLIT];
	};

	static
	__inline
	int sortConstraintByBatch(Contact4* cs, int n, int ignoreIdx, int simdWidth = -1)
	{
		SortData* sortData;
		{
			BT_PROFILE("new");
			sortData = new SortData[n];
		}

		u32* idxBuffer = new u32[n];
		u32* idxSrc = idxBuffer;
		u32* idxDst = idxBuffer;
		int nIdxSrc, nIdxDst;

		const int N_FLG = 256;
		const int FLG_MASK = N_FLG-1;
		u32 flg[N_FLG/32];
#if defined(_DEBUG)
		for(int i=0; i<n; i++) cs[i].getBatchIdx() = -1; 
#endif
		for(int i=0; i<n; i++) idxSrc[i] = i;
		nIdxSrc = n;

		int batchIdx = 0;

		{
			BT_PROFILE("batching");
			while( nIdxSrc )
			{
				nIdxDst = 0;
				int nCurrentBatch = 0;

				//	clear flag
				for(int i=0; i<N_FLG/32; i++) flg[i] = 0;

				for(int i=0; i<nIdxSrc; i++)
				{
					int idx = idxSrc[i];
					ADLASSERT( idx < n );
					//	check if it can go
					int aIdx = cs[idx].m_bodyAPtr & FLG_MASK;
					int bIdx = cs[idx].m_bodyBPtr & FLG_MASK;

					u32 aUnavailable = flg[ aIdx/32 ] & (1<<(aIdx&31));
					u32 bUnavailable = flg[ bIdx/32 ] & (1<<(bIdx&31));

					aUnavailable = (ignoreIdx==cs[idx].m_bodyAPtr)? 0:aUnavailable;
					bUnavailable = (ignoreIdx==cs[idx].m_bodyBPtr)? 0:bUnavailable;

					if( aUnavailable==0 && bUnavailable==0 ) // ok 
					{
						flg[ aIdx/32 ] |= (1<<(aIdx&31));
						flg[ bIdx/32 ] |= (1<<(bIdx&31));
						cs[idx].getBatchIdx() = batchIdx;
						sortData[idx].m_key = batchIdx;
						sortData[idx].m_value = idx;

						{
							nCurrentBatch++;
							if( nCurrentBatch == simdWidth )
							{
								nCurrentBatch = 0;
								for(int i=0; i<N_FLG/32; i++) flg[i] = 0;
							}
						}
					}
					else
					{
						idxDst[nIdxDst++] = idx;
					}
				}
				swap2( idxSrc, idxDst );
				swap2( nIdxSrc, nIdxDst );
				batchIdx ++;
			}
		}

		

		{
			BT_PROFILE("radix sort data");
			//	sort SortData
			Device::Config cfg;
			Device* deviceHost = DeviceUtils::allocate( TYPE_HOST, cfg );
			{
				Buffer<SortData> sortBuffer; sortBuffer.setRawPtr( deviceHost, sortData, n );
				RadixSort<TYPE_HOST>::Data* sort = RadixSort<TYPE_HOST>::allocate( deviceHost, n );

				RadixSort<TYPE_HOST>::execute( sort, sortBuffer, n );

				RadixSort<TYPE_HOST>::deallocate( sort );
			}
			DeviceUtils::deallocate( deviceHost );
		}

		{	
				BT_PROFILE("reorder");
			//	reorder
			Contact4* old = new Contact4[n];
			memcpy( old, cs, sizeof(Contact4)*n);
			for(int i=0; i<n; i++)
			{
				int idx = sortData[i].m_value;
				cs[i] = old[idx];
			}
			delete [] old;
		}

		{
			BT_PROFILE("delete");
			delete [] idxBuffer;
			delete [] sortData;
		}
#if defined(_DEBUG)
//		debugPrintf( "nBatches: %d\n", batchIdx );
		for(int i=0; i<n; i++) ADLASSERT( cs[i].getBatchIdx() != -1 );
#endif
		return batchIdx;
	}
};



enum
{
//	N_SPLIT = SOLVER_N_SPLIT,
//	MAX_TASKS_PER_BATCH = N_SPLIT*N_SPLIT/4,
};

struct SolveTask// : public ThreadPool::Task
{
	SolveTask(const Buffer<RigidBodyBase::Body>* bodies, const Buffer<RigidBodyBase::Inertia>* shapes, const Buffer<Constraint4>* constraints,
		int start, int nConstraints)
		: m_bodies( bodies ), m_shapes( shapes ), m_constraints( constraints ), m_start( start ), m_nConstraints( nConstraints ),
		m_solveFriction( true ){}

	u16 getType(){ return 0; }

	void run(int tIdx)
	{
		HostBuffer<RigidBodyBase::Body>& hBody = *(HostBuffer<RigidBodyBase::Body>*)m_bodies;
		HostBuffer<RigidBodyBase::Inertia>& hShape = *(HostBuffer<RigidBodyBase::Inertia>*)m_shapes;
		HostBuffer<Constraint4>& hc = *(HostBuffer<Constraint4>*)m_constraints;

		for(int ic=0; ic<m_nConstraints; ic++)
		{
			int i = m_start + ic;

			float frictionCoeff = hc[i].getFrictionCoeff();
			int aIdx = (int)hc[i].m_bodyA;
			int bIdx = (int)hc[i].m_bodyB;
			RigidBodyBase::Body& bodyA = hBody[aIdx];
			RigidBodyBase::Body& bodyB = hBody[bIdx];

			if( !m_solveFriction )
			{
				float maxRambdaDt[4] = {FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX};
				float minRambdaDt[4] = {0.f,0.f,0.f,0.f};

				SolverInl::solveContact<false>( hc[i], bodyA.m_pos, (MYF4&)bodyA.m_linVel, (MYF4&)bodyA.m_angVel, bodyA.m_invMass, hShape[aIdx].m_invInertia, 
					bodyB.m_pos, (MYF4&)bodyB.m_linVel, (MYF4&)bodyB.m_angVel, bodyB.m_invMass, hShape[bIdx].m_invInertia,
					maxRambdaDt, minRambdaDt );
			}
			else
			{
				float maxRambdaDt[4] = {FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX};
				float minRambdaDt[4] = {0.f,0.f,0.f,0.f};

				float sum = 0;
				for(int j=0; j<4; j++)
				{
					sum +=hc[i].m_appliedRambdaDt[j];
				}
				frictionCoeff = 0.7f;
				for(int j=0; j<4; j++)
				{
					maxRambdaDt[j] = frictionCoeff*sum;
					minRambdaDt[j] = -maxRambdaDt[j];
				}

				SolverInl::solveFriction( hc[i], bodyA.m_pos, (MYF4&)bodyA.m_linVel, (MYF4&)bodyA.m_angVel, bodyA.m_invMass, hShape[aIdx].m_invInertia, 
					bodyB.m_pos, (MYF4&)bodyB.m_linVel, (MYF4&)bodyB.m_angVel, bodyB.m_invMass, hShape[bIdx].m_invInertia,
					maxRambdaDt, minRambdaDt );
			}
		}
	}

	const Buffer<RigidBodyBase::Body>* m_bodies;
	const Buffer<RigidBodyBase::Inertia>* m_shapes;
	const Buffer<Constraint4>* m_constraints;
	int m_start;
	int m_nConstraints;
	bool m_solveFriction;
};


template<>
static Solver<adl::TYPE_HOST>::Data* Solver<adl::TYPE_HOST>::allocate( const Device* device, int pairCapacity )
{
	Solver<adl::TYPE_HOST>::Data* data = new Data;
	data->m_device = device;
	data->m_parallelSolveData = 0;

	return data;
}

template<>
static void Solver<adl::TYPE_HOST>::deallocate( Solver<TYPE_HOST>::Data* data )
{
	if( data->m_parallelSolveData ) delete (SolverInl::ParallelSolveData*)data->m_parallelSolveData;
	delete data;
}


void sortContacts2(  Solver<TYPE_HOST>::Data* data, const Buffer<RigidBodyBase::Body>* bodyBuf, 
			Buffer<Contact4>* contactsIn, void* additionalData, 
			int nContacts, const Solver<TYPE_HOST>::ConstraintCfg& cfg )
{
	ADLASSERT( data->m_device->m_type == TYPE_HOST );
	HostBuffer<RigidBodyBase::Body>* bodyNative 
		= (HostBuffer<RigidBodyBase::Body>*)BufferUtils::map<TYPE_HOST, true>( data->m_device, bodyBuf );
	HostBuffer<Contact4>* contactNative 
		= (HostBuffer<Contact4>*)BufferUtils::map<TYPE_HOST, true>( data->m_device, contactsIn);

	if( cfg.m_enableParallelSolve )
	{
		ADLASSERT( data->m_parallelSolveData == 0 );
		data->m_parallelSolveData = new SolverInl::ParallelSolveData;
		SolverInl::ParallelSolveData* solveData = (SolverInl::ParallelSolveData*)data->m_parallelSolveData;

		HostBuffer<SortData> sortData( data->m_device, nContacts );
		{	//	2. set cell idx
			float spacing = adl::SolverBase::N_OBJ_PER_SPLIT*cfg.m_averageExtent;
			float xScale = 1.f/spacing;
			for(int i=0; i<nContacts; i++)
			{
				int idx = ((*contactNative)[i].m_bodyAPtr==cfg.m_staticIdx)? (*contactNative)[i].m_bodyBPtr:(*contactNative)[i].m_bodyAPtr;
				float4& p = (*bodyNative)[idx].m_pos;
				int xIdx = (int)((p.x-((p.x<0.f)?1.f:0.f))*xScale)&(adl::SolverBase::N_SPLIT-1);
				int zIdx = (int)((p.z-((p.z<0.f)?1.f:0.f))*xScale)&(adl::SolverBase::N_SPLIT-1);
				ADLASSERT( xIdx >= 0 && xIdx < adl::SolverBase::N_SPLIT );
				ADLASSERT( zIdx >= 0 && zIdx < adl::SolverBase::N_SPLIT );
				sortData[i].m_key = (xIdx+zIdx*adl::SolverBase::N_SPLIT);
				sortData[i].m_value = i;
			}
		}

		{	//	3. sort by cell idx
			RadixSort<TYPE_HOST>::Data* sData = RadixSort<TYPE_HOST>::allocate( data->m_device, nContacts );

			RadixSort<TYPE_HOST>::execute( sData, sortData, nContacts );

			RadixSort<TYPE_HOST>::deallocate( sData );
		}

		{	//	4. find entries
			HostBuffer<u32> counts; counts.setRawPtr( data->m_device, solveData->m_n, adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT );
			HostBuffer<u32> offsets; offsets.setRawPtr( data->m_device, solveData->m_offset, adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT );
			{
				BoundSearch<TYPE_HOST>::Data* sData = BoundSearch<TYPE_HOST>::allocate( data->m_device );
				PrefixScan<TYPE_HOST>::Data* pData = PrefixScan<TYPE_HOST>::allocate( data->m_device, adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT );

				BoundSearch<TYPE_HOST>::execute( sData, sortData, nContacts, counts, adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT, BoundSearchBase::COUNT );

				PrefixScan<TYPE_HOST>::execute( pData, counts, offsets, adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT );
				
				BoundSearch<TYPE_HOST>::deallocate( sData );
				PrefixScan<TYPE_HOST>::deallocate( pData );
			}
#if defined(_DEBUG)
			{
				HostBuffer<u32> n0( data->m_device, adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT );
				HostBuffer<u32> offset0( data->m_device, adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT );
				for(int i=0; i<adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT; i++)
				{
					n0[i] = 0;
					offset0[i] = 0;
				}

				for(int i=0; i<nContacts; i++)
				{
					int idx = sortData[i].m_key;
					n0[idx]++;
				}

				//	scan
				int sum = 0;
				for(int i=0; i<adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT; i++)
				{
					offset0[i] = sum;
					sum += n0[i];
				}

				for(int i=0; i<adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT; i++)
				{
					ADLASSERT( n0[i] == counts[i] );
					ADLASSERT( offset0[i] == offsets[i] );
				}
			}
#endif
		}

		{	//	5. sort constraints by cellIdx
			Contact4* old = new Contact4[nContacts];
			memcpy( old, contactNative->m_ptr, sizeof(Contact4)*nContacts );
			for(int i=0; i<nContacts; i++)
			{
				int srcIdx = sortData[i].m_value;
				(*contactNative)[i] = old[srcIdx];
			}
			delete [] old;
		}
	}

	BufferUtils::unmap<false>( bodyNative, bodyBuf );
	BufferUtils::unmap<true>( contactNative, contactsIn );
}

static void reorderConvertToConstraints2( Solver<TYPE_HOST>::Data* data, const Buffer<RigidBodyBase::Body>* bodyBuf, 
	const Buffer<RigidBodyBase::Inertia>* shapeBuf,
	adl::Buffer<Contact4>* contactsIn, SolverData contactCOut, void* additionalData, 
	int nContacts, const Solver<TYPE_HOST>::ConstraintCfg& cfg )
{
	
	
	sortContacts2( data, bodyBuf, contactsIn, additionalData, nContacts, cfg );

	{
		SolverInl::ParallelSolveData* solveData = (SolverInl::ParallelSolveData*)data->m_parallelSolveData;
		Buffer<u32> n; n.setRawPtr( data->m_device, solveData->m_n, adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT );
		Buffer<u32> offsets; offsets.setRawPtr( data->m_device, solveData->m_offset, adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT );
		Solver<TYPE_HOST>::batchContacts( data, contactsIn, nContacts, &n, &offsets, cfg.m_staticIdx );
		printf("hello\n");
	}
	
	Solver<TYPE_HOST>::convertToConstraints( data, bodyBuf, shapeBuf, contactsIn, contactCOut, additionalData, nContacts, cfg );
}

template<DeviceType TYPE>
static void solveContactConstraint(  Solver<TYPE_HOST>::Data* data, const Buffer<RigidBodyBase::Body>* bodyBuf, const Buffer<RigidBodyBase::Inertia>* shapeBuf, 
			SolverData constraint, void* additionalData, int n )
{

	Buffer<RigidBodyBase::Body>* bodyNative
		= BufferUtils::map<TYPE_HOST, true>( data->m_device, bodyBuf );
	Buffer<RigidBodyBase::Inertia>* shapeNative
		= BufferUtils::map<TYPE_HOST, true>( data->m_device, shapeBuf );
	Buffer<Constraint4>* constraintNative
		= BufferUtils::map<TYPE_HOST, true>( data->m_device, (const Buffer<Constraint4>*)constraint );

	for(int iter=0; iter<data->m_nIterations; iter++)
	{
		SolveTask task( bodyNative, shapeNative, constraintNative, 0, n );
		task.m_solveFriction = false;
		task.run(0);
	}

	for(int iter=0; iter<data->m_nIterations; iter++)
	{
		SolveTask task( bodyNative, shapeNative, constraintNative, 0, n );
		task.m_solveFriction = true;
		task.run(0);
	}

	BufferUtils::unmap<true>( bodyNative, bodyBuf );
	BufferUtils::unmap<false>( shapeNative, shapeBuf );
	BufferUtils::unmap<false>( constraintNative, (const Buffer<Constraint4>*)constraint );
}

#if 0
static
int createSolveTasks( int batchIdx, Data* data, const Buffer<RigidBodyBase::Body>* bodyBuf, const Buffer<RigidBodyBase::Inertia>* shapeBuf, 
			SolverData constraint, int n, ThreadPool::Task* tasksOut[], int taskCapacity )
{
/*
	ADLASSERT( (N_SPLIT&1) == 0 );
	ADLASSERT( batchIdx < N_BATCHES );
	ADLASSERT( data->m_device->m_type == TYPE_HOST );
	ADLASSERT( data->m_parallelSolveData );

	SolverInl::ParallelSolveData* solveData = (SolverInl::ParallelSolveData*)data->m_parallelSolveData;
	data->m_batchIdx = 0;

	const int nx = N_SPLIT/2;

	int nTasksCreated = 0;

//	for(int ii=0; ii<2; ii++)
	for(batchIdx=0; batchIdx<4; batchIdx++)
	{
		int2 offset = make_int2( batchIdx&1, batchIdx>>1 );
		for(int ix=0; ix<nx; ix++) for(int iy=0; iy<nx; iy++)
		{
			int xIdx = ix*2 + offset.x;
			int yIdx = iy*2 + offset.y;
			int cellIdx = xIdx+yIdx*N_SPLIT;

			int n = solveData->m_n[cellIdx];
			int start = solveData->m_offset[cellIdx];

			if( n == 0 ) continue;

			SolveTask* task = new SolveTask( bodyBuf, shapeBuf, (const Buffer<Constraint4>*)constraint, start, n );
//			task->m_solveFriction = (ii==0)? false:true;
			tasksOut[nTasksCreated++] = task;
		}
	}

	return nTasksCreated;
*/
	ADLASSERT(0);
	return 0;
}
#endif



static void convertToConstraints2(  Solver<TYPE_HOST>::Data* data, const Buffer<RigidBodyBase::Body>* bodyBuf, 
	const Buffer<RigidBodyBase::Inertia>* shapeBuf, 
	Buffer<Contact4>* contactsIn, SolverData contactCOut, void* additionalData, 
	int nContacts, const Solver<TYPE_HOST>::ConstraintCfg& cfg )
{
	ADLASSERT( data->m_device->m_type == TYPE_HOST );

	HostBuffer<RigidBodyBase::Body>* bodyNative 
		= (HostBuffer<RigidBodyBase::Body>*)BufferUtils::map<TYPE_HOST, true>( data->m_device, bodyBuf );
	HostBuffer<RigidBodyBase::Inertia>* shapeNative 
		= (HostBuffer<RigidBodyBase::Inertia>*)BufferUtils::map<TYPE_HOST, true>( data->m_device, shapeBuf );
	HostBuffer<Contact4>* contactNative 
		= (HostBuffer<Contact4>*)BufferUtils::map<TYPE_HOST, true>( data->m_device, contactsIn );
	HostBuffer<Constraint4>* constraintNative 
		= (HostBuffer<Constraint4>*)BufferUtils::map<TYPE_HOST, false>( data->m_device, (Buffer<Constraint4>*)contactCOut );

	{
#if !defined(_DEBUG)
#pragma omp parallel for
#endif
		for(int i=0; i<nContacts; i++)
		{
//			new (constraintNative+i)Constraint4;
			Contact4& contact = (*contactNative)[i];

			if( contact.isInvalid() ) continue;

			int aIdx = (int)contact.m_bodyAPtr;
			int bIdx = (int)contact.m_bodyBPtr;

			{
				const RigidBodyBase::Body& bodyA = (*bodyNative)[aIdx];
				const RigidBodyBase::Body& bodyB = (*bodyNative)[bIdx];
				MYF4 posA( bodyA.m_pos );
				MYF4 linVelA( bodyA.m_linVel );
				MYF4 angVelA( bodyA.m_angVel );
				MYF4 posB( bodyB.m_pos );
				MYF4 linVelB( bodyB.m_linVel );
				MYF4 angVelB( bodyB.m_angVel );

				bool aIsInactive = ( isZero( linVelA ) && isZero( angVelA ) );
				bool bIsInactive = ( isZero( linVelB ) && isZero( angVelB ) );

				SolverInl::setConstraint4( posA, linVelA, angVelA, 
					//(*bodyNative)[aIdx].m_invMass, (*shapeNative)[aIdx].m_invInertia,
					(aIsInactive)? 0.f : (*bodyNative)[aIdx].m_invMass, (aIsInactive)? mtZero() : (*shapeNative)[aIdx].m_invInertia,
					posB, linVelB, angVelB, 
					//(*bodyNative)[bIdx].m_invMass, (*shapeNative)[bIdx].m_invInertia, 
					(bIsInactive)? 0.f : (*bodyNative)[bIdx].m_invMass, (bIsInactive)? mtZero() : (*shapeNative)[bIdx].m_invInertia, 
					contact, cfg, 
					(*constraintNative)[i] );
				(*constraintNative)[i].m_batchIdx = contact.getBatchIdx();
			}
		}
	}

	BufferUtils::unmap<false>( bodyNative, bodyBuf );
	BufferUtils::unmap<false>( shapeNative, shapeBuf );
	BufferUtils::unmap<false>( contactNative, contactsIn );
	BufferUtils::unmap<true>( constraintNative, (Buffer<Constraint4>*)contactCOut );
}





static void batchContacts2(  Solver<TYPE_HOST>::Data* data, Buffer<Contact4>* contacts, int nContacts, Buffer<u32>* n, Buffer<u32>* offsets, int staticIdx )
{
	ADLASSERT( data->m_device->m_type == TYPE_HOST );

	HostBuffer<Contact4>* contactNative =0;
	HostBuffer<u32>* nNative =0;
	HostBuffer<u32>* offsetsNative =0;

	int sz = sizeof(Contact4);
	int sz2 = sizeof(int2);
	{
		BT_PROFILE("BufferUtils::map");
		contactNative  = (HostBuffer<Contact4>*)BufferUtils::map<TYPE_HOST, true>( data->m_device, contacts, nContacts );
	}
	{
		BT_PROFILE("BufferUtils::map2");
		nNative = (HostBuffer<u32>*)BufferUtils::map<TYPE_HOST, true>( data->m_device, n );
		offsetsNative= (HostBuffer<u32>*)BufferUtils::map<TYPE_HOST, true>( data->m_device, offsets );
	}

	
	{
		BT_PROFILE("sortConstraintByBatch");
		int numNonzeroGrid=0;
		int maxNumBatches = 0;

		for(int i=0; i<adl::SolverBase::N_SPLIT*adl::SolverBase::N_SPLIT; i++)
		{
			int n = (*nNative)[i];
			int offset = (*offsetsNative)[i];

			if( n ) 
			{
				numNonzeroGrid++;
				int numBatches = SolverInl::sortConstraintByBatch( contactNative->m_ptr+offset, n, staticIdx,-1 );	//	on GPU
				maxNumBatches = max(numBatches,maxNumBatches);

	//			SolverInl::sortConstraintByBatch( contactNative->m_ptr+offset, n, staticIdx );	//	on CPU
			}
		}

		printf("maxNumBatches = %d\n", maxNumBatches);
	}

	{
		BT_PROFILE("BufferUtils::unmap");
		BufferUtils::unmap<true>( contactNative, contacts, nContacts );
	}
	{
		BT_PROFILE("BufferUtils::unmap2");
		BufferUtils::unmap<false>( nNative, n );
		BufferUtils::unmap<false>( offsetsNative, offsets );
	}


}


