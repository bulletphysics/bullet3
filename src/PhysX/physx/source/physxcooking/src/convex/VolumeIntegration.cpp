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


//#ifdef PX_COOKING

/*
*	This code computes volume integrals needed to compute mass properties of polyhedral bodies.
*	Based on public domain code by Brian Mirtich.
*/
#include "foundation/PxMemory.h"
#include "VolumeIntegration.h"
#include "PxSimpleTriangleMesh.h"
#include "PxConvexMeshDesc.h"
#include "GuConvexMeshData.h"
#include "PsUtilities.h"
#include "PsVecMath.h"


namespace physx
{

	using namespace Ps::aos;

namespace
{

	class VolumeIntegrator
	{
		public:
			VolumeIntegrator(PxSimpleTriangleMesh mesh, PxF64	mDensity);
			~VolumeIntegrator();
			bool	computeVolumeIntegrals(PxIntegrals& ir);
		private:
			struct Normal
				{
				PxVec3 normal;
				PxF32 w;
				};
			
			struct Face
				{
				PxF64 Norm[3];
				PxF64	w;
				PxU32	Verts[3];
				};
			
			// Data structures
			PxF64				mMass;					//!< Mass
			PxF64				mDensity;				//!< Density
			PxSimpleTriangleMesh mesh;	
			//Normal	*			faceNormals;			//!< temp face normal data structure
			
			
			
			
			unsigned int		mA;						//!< Alpha
			unsigned int		mB;						//!< Beta
			unsigned int		mC;						//!< Gamma
			
			// Projection integrals
			PxF64				mP1;
			PxF64				mPa;					//!< Pi Alpha
			PxF64				mPb;					//!< Pi Beta
			PxF64				mPaa;					//!< Pi Alpha^2
			PxF64				mPab;					//!< Pi AlphaBeta
			PxF64				mPbb;					//!< Pi Beta^2
			PxF64				mPaaa;					//!< Pi Alpha^3
			PxF64				mPaab;					//!< Pi Alpha^2Beta
			PxF64				mPabb;					//!< Pi AlphaBeta^2
			PxF64				mPbbb;					//!< Pi Beta^3
			
			// Face integrals
			PxF64				mFa;					//!< FAlpha
			PxF64				mFb;					//!< FBeta
			PxF64				mFc;					//!< FGamma
			PxF64				mFaa;					//!< FAlpha^2
			PxF64				mFbb;					//!< FBeta^2
			PxF64				mFcc;					//!< FGamma^2
			PxF64				mFaaa;					//!< FAlpha^3
			PxF64				mFbbb;					//!< FBeta^3
			PxF64				mFccc;					//!< FGamma^3
			PxF64				mFaab;					//!< FAlpha^2Beta
			PxF64				mFbbc;					//!< FBeta^2Gamma
			PxF64				mFcca;					//!< FGamma^2Alpha
			
			// The 10 volume integrals
			PxF64				mT0;					//!< ~Total mass
			PxF64				mT1[3];					//!< Location of the center of mass
			PxF64				mT2[3];					//!< Moments of inertia
			PxF64				mTP[3];					//!< Products of inertia
			
			// Internal methods
			//				bool				Init();
			PxVec3				computeCenterOfMass();
			void				computeInertiaTensor(PxF64* J);
			void				computeCOMInertiaTensor(PxF64* J);
			void				computeFaceNormal(Face & f, PxU32 * indices);
			
			void				computeProjectionIntegrals(const Face& f);
			void				computeFaceIntegrals(const Face& f);
	};

	#define X 0u
	#define Y 1u
	#define Z 2u

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	*	Constructor.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	VolumeIntegrator::VolumeIntegrator(PxSimpleTriangleMesh mesh_, PxF64	density)
	{
		mDensity	= density;
		mMass		= 0.0;
		this->mesh	= mesh_;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	*	Destructor.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	VolumeIntegrator::~VolumeIntegrator()
	{
	}

	void VolumeIntegrator::computeFaceNormal(Face & f, PxU32 * indices)
	{
		const PxU8 * vertPointer = reinterpret_cast<const PxU8*>(mesh.points.data);

		//two edges
		PxVec3 d1 = (*reinterpret_cast<const PxVec3 *>(vertPointer + mesh.points.stride * indices[1] )) - (*reinterpret_cast<const PxVec3 *>(vertPointer + mesh.points.stride * indices[0] ));
		PxVec3 d2 = (*reinterpret_cast<const PxVec3 *>(vertPointer + mesh.points.stride * indices[2] )) - (*reinterpret_cast<const PxVec3 *>(vertPointer + mesh.points.stride * indices[1] ));


		PxVec3 normal = d1.cross(d2);

		normal.normalize();

		f.w = - PxF64(normal.dot(		(*reinterpret_cast<const PxVec3 *>(vertPointer + mesh.points.stride * indices[0] ))	));

		f.Norm[0] = PxF64(normal.x);
		f.Norm[1] = PxF64(normal.y);
		f.Norm[2] = PxF64(normal.z);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	*	Computes volume integrals for a polyhedron by summing surface integrals over its faces.
	*	\param		ir	[out] a result structure.
	*	\return		true if success
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool VolumeIntegrator::computeVolumeIntegrals(PxIntegrals& ir)
	{
		// Clear all integrals
		mT0 = mT1[X] = mT1[Y] = mT1[Z] = mT2[X] = mT2[Y] = mT2[Z] = mTP[X] = mTP[Y] = mTP[Z] = 0;

		Face f;
		const PxU8 * trigPointer = reinterpret_cast<const PxU8*>(mesh.triangles.data);
		for(PxU32 i=0;i<mesh.triangles.count;i++, trigPointer += mesh.triangles.stride)
			{

			if (mesh.flags & PxMeshFlag::e16_BIT_INDICES)
				{
				f.Verts[0] = (reinterpret_cast<const PxU16 *>(trigPointer))[0];
				f.Verts[1] = (reinterpret_cast<const PxU16 *>(trigPointer))[1];
				f.Verts[2] = (reinterpret_cast<const PxU16 *>(trigPointer))[2];
				}
			else
				{
				f.Verts[0] = (reinterpret_cast<const PxU32 *>(trigPointer)[0]);
				f.Verts[1] = (reinterpret_cast<const PxU32 *>(trigPointer)[1]);
				f.Verts[2] = (reinterpret_cast<const PxU32 *>(trigPointer)[2]);
				}

			if (mesh.flags & PxMeshFlag::eFLIPNORMALS)
				{
				PxU32 t = f.Verts[1];
				f.Verts[1] = f.Verts[2];
				f.Verts[2] = t;
				}

			//compute face normal:
			computeFaceNormal(f,f.Verts);

			// Compute alpha/beta/gamma as the right-handed permutation of (x,y,z) that maximizes |n|
			PxF64 nx = fabs(f.Norm[X]);
			PxF64 ny = fabs(f.Norm[Y]);
			PxF64 nz = fabs(f.Norm[Z]);
			if (nx > ny && nx > nz) mC = X;
			else mC = (ny > nz) ? Y : Z;
			mA = (mC + 1) % 3;
			mB = (mA + 1) % 3;

			// Compute face contribution
			computeFaceIntegrals(f);

			// Update integrals
			mT0 += f.Norm[X] * ((mA == X) ? mFa : ((mB == X) ? mFb : mFc));

			mT1[mA] += f.Norm[mA] * mFaa;
			mT1[mB] += f.Norm[mB] * mFbb;
			mT1[mC] += f.Norm[mC] * mFcc;

			mT2[mA] += f.Norm[mA] * mFaaa;
			mT2[mB] += f.Norm[mB] * mFbbb;
			mT2[mC] += f.Norm[mC] * mFccc;

			mTP[mA] += f.Norm[mA] * mFaab;
			mTP[mB] += f.Norm[mB] * mFbbc;
			mTP[mC] += f.Norm[mC] * mFcca;
			}

		mT1[X] /= 2; mT1[Y] /= 2; mT1[Z] /= 2;
		mT2[X] /= 3; mT2[Y] /= 3; mT2[Z] /= 3;
		mTP[X] /= 2; mTP[Y] /= 2; mTP[Z] /= 2;

		// Fill result structure
		ir.COM = computeCenterOfMass();
		computeInertiaTensor(reinterpret_cast<PxF64*>(ir.inertiaTensor));
		computeCOMInertiaTensor(reinterpret_cast<PxF64*>(ir.COMInertiaTensor));
		ir.mass = mMass;
		return true;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	*	Computes the center of mass.
	*	\return		The center of mass.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	PxVec3 VolumeIntegrator::computeCenterOfMass()
	{
		// Compute center of mass
		PxVec3 COM(0.0f, 0.0f, 0.0f);
		if(mT0!=0.0)
			{
			COM.x = float(mT1[X] / mT0);
			COM.y = float(mT1[Y] / mT0);
			COM.z = float(mT1[Z] / mT0);
			}
		return COM;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	*	Setups the inertia tensor relative to the origin.
	*	\param		it	[out] the returned inertia tensor.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void VolumeIntegrator::computeInertiaTensor(PxF64* it)
	{
		PxF64 J[3][3];

		// Compute inertia tensor
		J[X][X] = mDensity * (mT2[Y] + mT2[Z]);
		J[Y][Y] = mDensity * (mT2[Z] + mT2[X]);
		J[Z][Z] = mDensity * (mT2[X] + mT2[Y]);

		J[X][Y] = J[Y][X] = - mDensity * mTP[X];
		J[Y][Z] = J[Z][Y] = - mDensity * mTP[Y];
		J[Z][X] = J[X][Z] = - mDensity * mTP[Z];

		PxMemCopy(it, J, 9*sizeof(PxF64));
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	*	Setups the inertia tensor relative to the COM.
	*	\param		it	[out] the returned inertia tensor.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void VolumeIntegrator::computeCOMInertiaTensor(PxF64* it)
	{
		PxF64 J[3][3];

		mMass = mDensity * mT0;

		const PxVec3 COM = computeCenterOfMass();
		const PxVec3 MassCOM(PxF32(mMass) * COM); 
		const PxVec3 MassCOM2(MassCOM.x * COM.x, MassCOM.y * COM.y, MassCOM.z * COM.z); 

		// Compute initial inertia tensor
		computeInertiaTensor(reinterpret_cast<PxF64*>(J));

		// Translate inertia tensor to center of mass
		// Huyghens' theorem:
		// Jx'x' = Jxx - m*(YG^2+ZG^2)
		// Jy'y' = Jyy - m*(ZG^2+XG^2)
		// Jz'z' = Jzz - m*(XG^2+YG^2)
		// XG, YG, ZG = new origin
		// YG^2+ZG^2 = dx^2
		J[X][X] -= PxF64(MassCOM2.y + MassCOM2.z);
		J[Y][Y] -= PxF64(MassCOM2.z + MassCOM2.x);
		J[Z][Z] -= PxF64(MassCOM2.x + MassCOM2.y);
		
		// Huyghens' theorem:
		// Jx'y' = Jxy - m*XG*YG
		// Jy'z' = Jyz - m*YG*ZG
		// Jz'x' = Jzx - m*ZG*XG
		// ### IS THE SIGN CORRECT ?
		J[X][Y] = J[Y][X] += PxF64(MassCOM.x * COM.y);
		J[Y][Z] = J[Z][Y] += PxF64(MassCOM.y * COM.z);
		J[Z][X] = J[X][Z] += PxF64(MassCOM.z * COM.x);

		PxMemCopy(it, J, 9*sizeof(PxF64));
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	*	Computes integrals over a face projection from the coordinates of the projections vertices.
	*	\param		f	[in] a face structure.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void VolumeIntegrator::computeProjectionIntegrals(const Face& f)
	{
		mP1 = mPa = mPb = mPaa = mPab = mPbb = mPaaa = mPaab = mPabb = mPbbb = 0.0;

		const PxU8* vertPointer = reinterpret_cast<const PxU8*>(mesh.points.data);
		for(PxU32 i=0;i<3;i++)
		{
			const PxVec3& p0 = *reinterpret_cast<const PxVec3 *>(vertPointer + mesh.points.stride * (f.Verts[i]) );
			const PxVec3& p1 = *reinterpret_cast<const PxVec3 *>(vertPointer + mesh.points.stride * (f.Verts[(i+1) % 3]) );


			PxF64 a0 = PxF64(p0[mA]);
			PxF64 b0 = PxF64(p0[mB]);
			PxF64 a1 = PxF64(p1[mA]);
			PxF64 b1 = PxF64(p1[mB]);

			PxF64 da = a1 - a0;				// DeltaA
			PxF64 db = b1 - b0;				// DeltaB

			PxF64 a0_2 = a0 * a0;				// Alpha0^2
			PxF64 a0_3 = a0_2 * a0;			// ...
			PxF64 a0_4 = a0_3 * a0;

			PxF64 b0_2 = b0 * b0;
			PxF64 b0_3 = b0_2 * b0;
			PxF64 b0_4 = b0_3 * b0;

			PxF64 a1_2 = a1 * a1;
			PxF64 a1_3 = a1_2 * a1; 

			PxF64 b1_2 = b1 * b1;
			PxF64 b1_3 = b1_2 * b1;

			PxF64 C1 = a1 + a0;

			PxF64 Ca = a1*C1 + a0_2;
			PxF64 Caa = a1*Ca + a0_3;
			PxF64 Caaa = a1*Caa + a0_4;

			PxF64 Cb = b1*(b1 + b0) + b0_2;
			PxF64 Cbb = b1*Cb + b0_3;
			PxF64 Cbbb = b1*Cbb + b0_4;

			PxF64 Cab = 3*a1_2 + 2*a1*a0 + a0_2;
			PxF64 Kab = a1_2 + 2*a1*a0 + 3*a0_2;

			PxF64 Caab = a0*Cab + 4*a1_3;
			PxF64 Kaab = a1*Kab + 4*a0_3;

			PxF64 Cabb = 4*b1_3 + 3*b1_2*b0 + 2*b1*b0_2 + b0_3;
			PxF64 Kabb = b1_3 + 2*b1_2*b0 + 3*b1*b0_2 + 4*b0_3;

			mP1		+= db*C1;
			mPa		+= db*Ca;
			mPaa	+= db*Caa;
			mPaaa	+= db*Caaa;
			mPb		+= da*Cb;
			mPbb	+= da*Cbb;
			mPbbb	+= da*Cbbb;
			mPab	+= db*(b1*Cab + b0*Kab);
			mPaab	+= db*(b1*Caab + b0*Kaab);
			mPabb	+= da*(a1*Cabb + a0*Kabb);
		}

		mP1		/= 2.0;
		mPa		/= 6.0;
		mPaa	/= 12.0;
		mPaaa	/= 20.0;
		mPb		/= -6.0;
		mPbb	/= -12.0;
		mPbbb	/= -20.0;
		mPab	/= 24.0;
		mPaab	/= 60.0;
		mPabb	/= -60.0;
	}

	#define		SQR(x)			((x)*(x))						//!< Returns x square
	#define		CUBE(x)			((x)*(x)*(x))					//!< Returns x cube

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	*	Computes surface integrals over a polyhedral face from the integrals over its projection.
	*	\param		f	[in] a face structure.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void VolumeIntegrator::computeFaceIntegrals(const Face& f)
	{
		computeProjectionIntegrals(f);

		PxF64 w = f.w;
		const PxF64* n = f.Norm;
		PxF64 k1 = 1 / n[mC];
		PxF64 k2 = k1 * k1;
		PxF64 k3 = k2 * k1;
		PxF64 k4 = k3 * k1;

		mFa = k1 * mPa;
		mFb = k1 * mPb;
		mFc = -k2 * (n[mA]*mPa + n[mB]*mPb + w*mP1);

		mFaa = k1 * mPaa;
		mFbb = k1 * mPbb;
		mFcc = k3 * (SQR(n[mA])*mPaa + 2*n[mA]*n[mB]*mPab + SQR(n[mB])*mPbb + w*(2*(n[mA]*mPa + n[mB]*mPb) + w*mP1));

		mFaaa = k1 * mPaaa;
		mFbbb = k1 * mPbbb;
		mFccc = -k4 * (CUBE(n[mA])*mPaaa + 3*SQR(n[mA])*n[mB]*mPaab 
			+ 3*n[mA]*SQR(n[mB])*mPabb + CUBE(n[mB])*mPbbb
			+ 3*w*(SQR(n[mA])*mPaa + 2*n[mA]*n[mB]*mPab + SQR(n[mB])*mPbb)
			+ w*w*(3*(n[mA]*mPa + n[mB]*mPb) + w*mP1));

		mFaab = k1 * mPaab;
		mFbbc = -k2 * (n[mA]*mPabb + n[mB]*mPbbb + w*mPbb);
		mFcca = k3 * (SQR(n[mA])*mPaaa + 2*n[mA]*n[mB]*mPaab + SQR(n[mB])*mPabb + w*(2*(n[mA]*mPaa + n[mB]*mPab) + w*mPa));
	}

	/*
	*	This code computes volume integrals needed to compute mass properties of polyhedral bodies.
	*	Based on public domain code by David Eberly.
	*/

	class VolumeIntegratorEberly 
	{
	public:		
		VolumeIntegratorEberly(const PxConvexMeshDesc& mesh, PxF64	mDensity);
		~VolumeIntegratorEberly();
		bool	computeVolumeIntegralsSIMD(PxIntegrals& ir, const PxVec3& origin);
		bool	computeVolumeIntegrals(PxIntegrals& ir, const PxVec3& origin);

	private:
		VolumeIntegratorEberly& operator=(const VolumeIntegratorEberly&);		
		const PxConvexMeshDesc&	mDesc;
		PxF64					mMass;
		PxReal					mMassR;
		PxF64					mDensity;			
	};

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	*	Constructor.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	VolumeIntegratorEberly::VolumeIntegratorEberly(const PxConvexMeshDesc& desc, PxF64	density)
		: mDesc(desc), mMass(0), mMassR(0), mDensity(density)
	{
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	*	Destructor.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	VolumeIntegratorEberly::~VolumeIntegratorEberly()
	{
	}

	PX_FORCE_INLINE void subexpressions(PxF64 w0, PxF64 w1, PxF64 w2, PxF64& f1, PxF64& f2, PxF64& f3, PxF64& g0, PxF64& g1, PxF64& g2)
	{
		PxF64 temp0 = w0 + w1;
		f1 = temp0 + w2;
		PxF64 temp1 = w0*w0;
		PxF64 temp2 = temp1 + w1*temp0;
		f2 = temp2 + w2*f1;
		f3 = w0*temp1 + w1*temp2 + w2*f2;
		g0 = f2 + w0*(f1 + w0);
		g1 = f2 + w1*(f1 + w1);
		g2 = f2 + w2*(f1 + w2);
	}

	PX_FORCE_INLINE void subexpressionsSIMD(const Vec4V& w0, const Vec4V& w1, const Vec4V& w2, 
		Vec4V& f1, Vec4V& f2, Vec4V& f3, Vec4V& g0, Vec4V& g1, Vec4V& g2)
	{
		const Vec4V temp0 = V4Add(w0, w1);
		f1 = V4Add(temp0, w2);
		const Vec4V temp1 = V4Mul(w0,w0);
		const Vec4V temp2 = V4MulAdd(w1, temp0, temp1);
		f2 = V4MulAdd(w2, f1, temp2);

		// f3 = w0.multiply(temp1) + w1.multiply(temp2) + w2.multiply(f2);
		const Vec4V ad0 = V4Mul(w0, temp1);
		const Vec4V ad1 = V4MulAdd(w1, temp2, ad0);
		f3 = V4MulAdd(w2, f2, ad1);

		g0 = V4MulAdd(w0, V4Add(f1, w0), f2); // f2 + w0.multiply(f1 + w0);
		g1 = V4MulAdd(w1, V4Add(f1, w1), f2); // f2 + w1.multiply(f1 + w1);
		g2 = V4MulAdd(w2, V4Add(f1, w2), f2); // f2 + w2.multiply(f1 + w2);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	*	Computes volume integrals for a polyhedron by summing surface integrals over its faces. SIMD version
	*	\param		ir	[out] a result structure.
	*	\param		origin [in] the origin of the mesh vertices. All vertices will be shifted accordingly prior to computing the volume integrals.
					Can improve accuracy, for example, if the centroid is used in the case of a convex mesh. Note: the returned inertia will not be relative to this origin but relative to (0,0,0).
	*	\return		true if success
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool VolumeIntegratorEberly::computeVolumeIntegralsSIMD(PxIntegrals& ir, const PxVec3& origin)
	{		
		FloatV	mult = FLoad(1.0f/6.0f);
		const Vec4V multV = V4Load(1.0f/24.0f);
		const Vec4V multV2 = V4Load(1.0f/60.0f);
		const Vec4V multVV = V4Load(1.0f/120.0f);

		// order: 1, x, y, z, x^2, y^2, z^2, xy, yz, zx
		FloatV intg = FLoad(0.0f);
		Vec4V intgV = V4Load(0.0f);
		Vec4V intgV2 = V4Load(0.0f);
		Vec4V intgVV = V4Load(0.0f);

		const Vec4V originV = Vec4V_From_PxVec3_WUndefined(origin);
		const FloatV zeroV = FLoad(0.0f); 

		const PxVec3* hullVerts = static_cast<const PxVec3*> (mDesc.points.data);
		const Gu::HullPolygonData* hullPolygons = static_cast<const Gu::HullPolygonData*> (mDesc.polygons.data); 

		for (PxU32 i = 0; i < mDesc.polygons.count; i++)
		{
			const Gu::HullPolygonData& polygon = hullPolygons[i];
			const PxU8* data = static_cast<const PxU8*>(mDesc.indices.data) + polygon.mVRef8;
			const PxU32 nbVerts = polygon.mNbVerts;

			PX_ASSERT(nbVerts > 2);

			const Vec4V normalV = V4LoadU(&polygon.mPlane.n.x);

			for (PxU32 j = 0; j < nbVerts - 2; j++)
			{
				// Should be safe to V4Load, we allocate one more vertex each time
				const Vec4V vertex0 = V4LoadU(&hullVerts[data[0]].x);
				const Vec4V vertex1 = V4LoadU(&hullVerts[data[j + 1]].x);
				const Vec4V vertex2 = V4LoadU(&hullVerts[data[j + 2]].x);

				const Vec4V p0 = V4Sub(vertex0, originV);
				Vec4V p1 = V4Sub(vertex1, originV);
				Vec4V p2 = V4Sub(vertex2, originV);

				const Vec4V p0YZX = V4PermYZXW(p0);
				const Vec4V p1YZX = V4PermYZXW(p1);
				const Vec4V p2YZX = V4PermYZXW(p2);

				// get edges and cross product of edges
				Vec4V d = V4Cross(V4Sub(p1, p0), V4Sub(p2, p0)); //  (p1 - p0).cross(p2 - p0);

				const FloatV dist = V4Dot3(d, normalV);
				//if(cp.dot(normalV) < 0)
				if(FAllGrtr(zeroV, dist))
				{
					d = V4Neg(d);
					Vec4V temp = p1;
					p1 = p2;
					p2 = temp;					
				}
				
				// compute integral terms
				Vec4V f1; Vec4V f2; Vec4V f3; Vec4V g0; Vec4V g1; Vec4V g2;

				subexpressionsSIMD(p0, p1, p2, f1, f2, f3, g0, g1, g2);

				// update integrals				
				intg = FScaleAdd(V4GetX(d), V4GetX(f1), intg); //intg += d.x*f1.x;

				intgV = V4MulAdd(d, f2, intgV); // intgV +=d.multiply(f2);
				intgV2 = V4MulAdd(d, f3, intgV2); // intgV2 += d.multiply(f3);

				const Vec4V ad0 = V4Mul(p0YZX, g0);
				const Vec4V ad1 = V4MulAdd(p1YZX, g1, ad0);
				const Vec4V ad2 = V4MulAdd(p2YZX, g2, ad1);
				intgVV = V4MulAdd(d, ad2, intgVV); //intgVV += d.multiply(p0YZX.multiply(g0) + p1YZX.multiply(g1) + p2YZX.multiply(g2));
			}
		}		

		intg = FMul(intg, mult); // intg *= mult;
		intgV = V4Mul(intgV, multV);
		intgV2 = V4Mul(intgV2, multV2);
		intgVV = V4Mul(intgVV, multVV);

		// center of mass ir.COM = intgV/mMassR;
		const Vec4V comV = V4ScaleInv(intgV, intg);
		// we rewrite the mass, but then we set it back 
		V4StoreU(comV, &ir.COM.x);		

		FStore(intg, &mMassR);
		ir.mass = PxF64(mMassR); // = intg;

		PxVec3 intg2;
		V3StoreU(Vec3V_From_Vec4V(intgV2), intg2);

		PxVec3 intVV;
		V3StoreU(Vec3V_From_Vec4V(intgVV), intVV);

		// inertia tensor relative to the provided origin parameter
		ir.inertiaTensor[0][0] = PxF64(intg2.y + intg2.z);
		ir.inertiaTensor[1][1] = PxF64(intg2.x + intg2.z);
		ir.inertiaTensor[2][2] = PxF64(intg2.x + intg2.y);
		ir.inertiaTensor[0][1] = ir.inertiaTensor[1][0] = PxF64(-intVV.x);
		ir.inertiaTensor[1][2] = ir.inertiaTensor[2][1] = PxF64(-intVV.y);
		ir.inertiaTensor[0][2] = ir.inertiaTensor[2][0] = PxF64(-intVV.z);

		// inertia tensor relative to center of mass
		ir.COMInertiaTensor[0][0] = ir.inertiaTensor[0][0] -PxF64(mMassR*(ir.COM.y*ir.COM.y+ir.COM.z*ir.COM.z));
		ir.COMInertiaTensor[1][1] = ir.inertiaTensor[1][1] -PxF64(mMassR*(ir.COM.z*ir.COM.z+ir.COM.x*ir.COM.x));
		ir.COMInertiaTensor[2][2] = ir.inertiaTensor[2][2] -PxF64(mMassR*(ir.COM.x*ir.COM.x+ir.COM.y*ir.COM.y));
		ir.COMInertiaTensor[0][1] = ir.COMInertiaTensor[1][0] = (ir.inertiaTensor[0][1] +PxF64(mMassR*ir.COM.x*ir.COM.y));
		ir.COMInertiaTensor[1][2] = ir.COMInertiaTensor[2][1] = (ir.inertiaTensor[1][2] +PxF64(mMassR*ir.COM.y*ir.COM.z));
		ir.COMInertiaTensor[0][2] = ir.COMInertiaTensor[2][0] = (ir.inertiaTensor[0][2] +PxF64(mMassR*ir.COM.z*ir.COM.x));

		// inertia tensor relative to (0,0,0)
		if (!origin.isZero())
		{
			PxVec3 sum = ir.COM + origin;
			ir.inertiaTensor[0][0] -= PxF64(mMassR*((ir.COM.y*ir.COM.y+ir.COM.z*ir.COM.z) - (sum.y*sum.y+sum.z*sum.z)));
			ir.inertiaTensor[1][1] -= PxF64(mMassR*((ir.COM.z*ir.COM.z+ir.COM.x*ir.COM.x) - (sum.z*sum.z+sum.x*sum.x)));
			ir.inertiaTensor[2][2] -= PxF64(mMassR*((ir.COM.x*ir.COM.x+ir.COM.y*ir.COM.y) - (sum.x*sum.x+sum.y*sum.y)));
			ir.inertiaTensor[0][1] = ir.inertiaTensor[1][0] = ir.inertiaTensor[0][1] + PxF64(mMassR*((ir.COM.x*ir.COM.y) - (sum.x*sum.y)));
			ir.inertiaTensor[1][2] = ir.inertiaTensor[2][1] = ir.inertiaTensor[1][2] + PxF64(mMassR*((ir.COM.y*ir.COM.z) - (sum.y*sum.z)));
			ir.inertiaTensor[0][2] = ir.inertiaTensor[2][0] = ir.inertiaTensor[0][2] + PxF64(mMassR*((ir.COM.z*ir.COM.x) - (sum.z*sum.x)));
			ir.COM = sum;
		}

		return true;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	*	Computes volume integrals for a polyhedron by summing surface integrals over its faces.
	*	\param		ir	[out] a result structure.
	*	\param		origin [in] the origin of the mesh vertices. All vertices will be shifted accordingly prior to computing the volume integrals.
					Can improve accuracy, for example, if the centroid is used in the case of a convex mesh. Note: the returned inertia will not be relative to this origin but relative to (0,0,0).
	*	\return		true if success
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool VolumeIntegratorEberly::computeVolumeIntegrals(PxIntegrals& ir, const PxVec3& origin)
	{
		const PxF64 mult[10] = {1.0/6.0,1.0/24.0,1.0/24.0,1.0/24.0,1.0/60.0,1.0/60.0,1.0/60.0,1.0/120.0,1.0/120.0,1.0/120.0};
		PxF64 intg[10] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // order: 1, x, y, z, x^2, y^2, z^2, xy, yz, zx
		const PxVec3* hullVerts = static_cast<const PxVec3*> (mDesc.points.data);

		for (PxU32 i = 0; i < mDesc.polygons.count; i++)
		{
			const Gu::HullPolygonData& polygon = (static_cast<const Gu::HullPolygonData*> (mDesc.polygons.data))[i];
			const PxU8* Data = static_cast<const PxU8*>(mDesc.indices.data) + polygon.mVRef8;
			const PxU32 NbVerts = polygon.mNbVerts;
			for (PxU32 j = 0; j < NbVerts - 2; j++)
			{
				const PxVec3 p0 = hullVerts[Data[0]] - origin;
				PxVec3 p1 = hullVerts[Data[(j + 1) % NbVerts]] - origin;
				PxVec3 p2 = hullVerts[Data[(j + 2) % NbVerts]] - origin;

				PxVec3 cp = (p1 - p0).cross(p2 - p0);

				if(cp.dot(polygon.mPlane.n) < 0)
				{
					cp = -cp;
					Ps::swap(p1,p2);
				}

				PxF64 x0 = PxF64(p0.x); PxF64 y0 = PxF64(p0.y); PxF64 z0 = PxF64(p0.z);
				PxF64 x1 = PxF64(p1.x); PxF64 y1 = PxF64(p1.y); PxF64 z1 = PxF64(p1.z);
				PxF64 x2 = PxF64(p2.x); PxF64 y2 = PxF64(p2.y); PxF64 z2 = PxF64(p2.z);

				// get edges and cross product of edges
				PxF64 d0 = PxF64(cp.x); PxF64 d1 = PxF64(cp.y); PxF64 d2 = PxF64(cp.z);

				// compute integral terms
				PxF64 f1x; PxF64 f2x; PxF64 f3x; PxF64 g0x; PxF64 g1x; PxF64 g2x;
				PxF64 f1y; PxF64 f2y; PxF64 f3y; PxF64 g0y; PxF64 g1y; PxF64 g2y;
				PxF64 f1z; PxF64 f2z; PxF64 f3z; PxF64 g0z; PxF64 g1z; PxF64 g2z;

				subexpressions(x0, x1, x2, f1x, f2x, f3x, g0x, g1x, g2x);
				subexpressions(y0, y1, y2, f1y, f2y, f3y, g0y, g1y, g2y);
				subexpressions(z0, z1, z2, f1z, f2z, f3z, g0z, g1z, g2z);

				// update integrals
				intg[0] += d0*f1x;
				intg[1] += d0*f2x; intg[2] += d1*f2y; intg[3] += d2*f2z;
				intg[4] += d0*f3x; intg[5] += d1*f3y; intg[6] += d2*f3z;
				intg[7] += d0*(y0*g0x + y1*g1x + y2*g2x);
				intg[8] += d1*(z0*g0y + z1*g1y + z2*g2y);
				intg[9] += d2*(x0*g0z + x1*g1z + x2*g2z);

			}
		}

		for (PxU32 i = 0; i < 10; i++)
		{
			intg[i] *= mult[i];
		}

		ir.mass = mMass = intg[0];
		// center of mass
		ir.COM.x = PxReal(intg[1]/mMass);
		ir.COM.y = PxReal(intg[2]/mMass);
		ir.COM.z = PxReal(intg[3]/mMass);

		// inertia tensor relative to the provided origin parameter
		ir.inertiaTensor[0][0] = intg[5]+intg[6];
		ir.inertiaTensor[1][1] = intg[4]+intg[6];
		ir.inertiaTensor[2][2] = intg[4]+intg[5];
		ir.inertiaTensor[0][1] = ir.inertiaTensor[1][0] = -intg[7];
		ir.inertiaTensor[1][2] = ir.inertiaTensor[2][1] = -intg[8];
		ir.inertiaTensor[0][2] = ir.inertiaTensor[2][0] = -intg[9];		

		// inertia tensor relative to center of mass
		ir.COMInertiaTensor[0][0] = ir.inertiaTensor[0][0] -mMass*PxF64((ir.COM.y*ir.COM.y+ir.COM.z*ir.COM.z));
		ir.COMInertiaTensor[1][1] = ir.inertiaTensor[1][1] -mMass*PxF64((ir.COM.z*ir.COM.z+ir.COM.x*ir.COM.x));
		ir.COMInertiaTensor[2][2] = ir.inertiaTensor[2][2] -mMass*PxF64((ir.COM.x*ir.COM.x+ir.COM.y*ir.COM.y));
		ir.COMInertiaTensor[0][1] = ir.COMInertiaTensor[1][0] = (ir.inertiaTensor[0][1] +mMass*PxF64(ir.COM.x*ir.COM.y));
		ir.COMInertiaTensor[1][2] = ir.COMInertiaTensor[2][1] = (ir.inertiaTensor[1][2] +mMass*PxF64(ir.COM.y*ir.COM.z));
		ir.COMInertiaTensor[0][2] = ir.COMInertiaTensor[2][0] = (ir.inertiaTensor[0][2] +mMass*PxF64(ir.COM.z*ir.COM.x));

		// inertia tensor relative to (0,0,0)
		if (!origin.isZero())
		{
			PxVec3 sum = ir.COM + origin;
			ir.inertiaTensor[0][0] -= mMass*PxF64((ir.COM.y*ir.COM.y+ir.COM.z*ir.COM.z) - (sum.y*sum.y+sum.z*sum.z));
			ir.inertiaTensor[1][1] -= mMass*PxF64((ir.COM.z*ir.COM.z+ir.COM.x*ir.COM.x) - (sum.z*sum.z+sum.x*sum.x));
			ir.inertiaTensor[2][2] -= mMass*PxF64((ir.COM.x*ir.COM.x+ir.COM.y*ir.COM.y) - (sum.x*sum.x+sum.y*sum.y));
			ir.inertiaTensor[0][1] = ir.inertiaTensor[1][0] = ir.inertiaTensor[0][1] + mMass*PxF64((ir.COM.x*ir.COM.y) - (sum.x*sum.y));
			ir.inertiaTensor[1][2] = ir.inertiaTensor[2][1] = ir.inertiaTensor[1][2] + mMass*PxF64((ir.COM.y*ir.COM.z) - (sum.y*sum.z));
			ir.inertiaTensor[0][2] = ir.inertiaTensor[2][0] = ir.inertiaTensor[0][2] + mMass*PxF64((ir.COM.z*ir.COM.x) - (sum.z*sum.x));
			ir.COM = sum;
		}

		return true;
	}
} // namespace

// Wrapper
bool computeVolumeIntegrals(const PxSimpleTriangleMesh& mesh, PxReal density, PxIntegrals& integrals)
{
	VolumeIntegrator v(mesh, PxF64(density));
	return v.computeVolumeIntegrals(integrals);
}

// Wrapper
bool computeVolumeIntegralsEberly(const PxConvexMeshDesc& mesh, PxReal density, PxIntegrals& integrals, const PxVec3& origin)
{
	VolumeIntegratorEberly v(mesh, PxF64(density));
	v.computeVolumeIntegrals(integrals, origin);
	return true;
}

// Wrapper
bool computeVolumeIntegralsEberlySIMD(const PxConvexMeshDesc& mesh, PxReal density, PxIntegrals& integrals, const PxVec3& origin)
{
	VolumeIntegratorEberly v(mesh, PxF64(density));
	v.computeVolumeIntegralsSIMD(integrals, origin);
	return true;
}

}

//#endif
