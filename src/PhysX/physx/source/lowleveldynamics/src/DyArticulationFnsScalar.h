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



#ifndef DY_ARTICULATION_SCALAR_FNS_H
#define DY_ARTICULATION_SCALAR_FNS_H

// Scalar helpers for articulations

#include "DyArticulationUtils.h"
#include "DyArticulationScalar.h"
#include "DySpatial.h"

namespace physx
{

namespace Dy
{

/*
namespace
{
	static void print(const PxMat33 &m)
	{
		printf("(%f, %f, %f)\n(%f, %f, %f)\n(%f, %f, %f)\n\n",
			m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2]);
	}

	static void print(const Cm::SpatialVector *v, PxU32 count)
	{
		for(PxU32 i=0;i<count;i++)
		{
			printf("(%f, %f, %f), (%f, %f, %f)\n", 
				v[i].linear.x, v[i].linear.y, v[i].linear.z,
				v[i].angular.x, v[i].angular.y, v[i].angular.z);
		}
	}
}
*/

class ArticulationDiagnostics
{
public:
static bool cholesky(const PxMat33& in, PxMat33& out)
{
	out = in;

	if(out[0][0]<=0)
		return false;

	out[0] /= PxSqrt(out[0][0]);
	out[1] -= out[0][1]*out[0];
	out[2] -= out[0][2]*out[0];

	if(out[1][1]<=0)
		return false;

	out[1] /= PxSqrt(out[1][1]);

	out[2] -= out[1][2]*out[1];
	if(out[2][2]<=0)
		return false;
	out[2] /= PxSqrt(out[2][2]);

	out[1][0] = out[2][0] = out[2][1] = 0;
	return true;
}

static bool isSymmetric(const PxMat33&a)
{
	return a[0][1] == a[1][0] && a[0][2] == a[2][0] && a[1][2] == a[2][1];
}

static bool isSymmetric(const Mat33V&a)
{
	PxMat33 m;
	PxMat33_From_Mat33V(a,m);
	return isSymmetric(m);
}

static bool isSymmetric(const SpInertia&a)
{
	return isSymmetric(a.mLL) && isSymmetric(a.mAA);
}


static bool isPositiveDefinite(const PxMat33& m)
{
	PxMat33 _;
	return cholesky(m, _);
}


static bool isPositiveDefinite(const SpInertia &s)
{
	// compute
	// (a 0)
	// (b c)

	PxMat33 a;
	if(!cholesky(s.mLL, a))
		return false;

	PxMat33 bt = a.getInverse() * s.mLA;
	PxMat33 x = s.mAA - bt.getTranspose()*bt;
	PxMat33 c;
	return cholesky(x, c);
}

};

class ArticulationFnsScalar
{
public:

	static PX_FORCE_INLINE Cm::SpatialVector translateMotion(const PxVec3& p, const Cm::SpatialVector& v)
	{
		return Cm::SpatialVector(v.linear + p.cross(v.angular), v.angular);
	}

	// translate a force resolved at position p to the origin

	static PX_FORCE_INLINE Cm::SpatialVector translateForce(const PxVec3& p, const Cm::SpatialVector& v)
	{
		return Cm::SpatialVector(v.linear, v.angular + p.cross(v.linear));
	}

	static PX_FORCE_INLINE PxMat33 invertSym33(const PxMat33& in)
	{
		PxVec3 v0 = in[1].cross(in[2]),
			   v1 = in[2].cross(in[0]),
			   v2 = in[0].cross(in[1]);

		PxReal det = v0.dot(in[0]);


		PX_ASSERT(det!=0);
		PxReal recipDet = 1.0f/det;

		return PxMat33(v0 * recipDet,
					   PxVec3(v0.y, v1.y, v1.z) * recipDet,
					   PxVec3(v0.z, v1.z, v2.z) * recipDet);
	}

	static PX_FORCE_INLINE SpInertia multiplySubtract(const SpInertia& I, const Cm::SpatialVector in0[3], const Cm::SpatialVector in1[3])
	{
		return I - SpInertia::dyad(in0[0], in1[0])
				 - SpInertia::dyad(in0[1], in1[1])
				 - SpInertia::dyad(in0[2], in1[2]);
	}

	static PX_FORCE_INLINE PxMat33 multiplySym(const Cm::SpatialVector* IS, const Cm::SpatialVector* S)
	{
	//		return PxMat33(axisDot(IS, S[0]), axisDot(IS, S[1]), axisDot(IS, S[2]));

		PxReal a00 = IS[0].dot(S[0]), a01 = IS[0].dot(S[1]), a02 = IS[0].dot(S[2]),
									  a11 = IS[1].dot(S[1]), a12 = IS[1].dot(S[2]),
															 a22 = IS[2].dot(S[2]);

		return PxMat33(PxVec3(a00, a01, a02),
					   PxVec3(a01, a11, a12),
					   PxVec3(a02, a12, a22));
	}

	static PX_FORCE_INLINE void multiply(Cm::SpatialVector out[3], const SpInertia& I, const Cm::SpatialVector in[3])
	{
		out[0] = I * in[0];
		out[1] = I * in[1];
		out[2] = I * in[2];
	}

	static PX_FORCE_INLINE void multiply(Cm::SpatialVector out[3], const Cm::SpatialVector in[3], const PxMat33& D)
	{
		out[0] = axisMultiply(in, D[0]);
		out[1] = axisMultiply(in, D[1]);
		out[2] = axisMultiply(in, D[2]);
	}

	static PxMat33 invSqrt(const PxMat33 &m)
	{
		// cholesky factor to 
		// (a 0 0)
		// (b c 0)
		// (d e f)
		// except that a,c,f are the reciprocal sqrts rather than sqrts

		PxVec3 v0 = m.column0, v1 = m.column1, v2 = m.column2;

		PxReal a = PxRecipSqrt(v0.x);
		PxReal b = v0.y*a;
		PxReal c = PxRecipSqrt(v1.y - b*b);
		PxReal d = v0.z*a;
		PxReal e = (v1.z-d*b) * c;
		PxReal f = PxRecipSqrt(v2.z - d*d - e*e);

		// invert 
		PxReal x = -b*a*c, y = (-e*x-d*a)*f, z = -e*c*f;

		PxMat33 r(PxVec3(a, 0,  0 ),
				  PxVec3(x,  c, 0 ),
				  PxVec3(y,  z,  f));

		return r;
	}


	static PX_FORCE_INLINE PxMat33 computeSIS(const Cm::SpatialVector S[3], const SpInertia& I)
	{
		Cm::SpatialVector IS[3];
		multiply(IS, I, S);
		return multiplySym(IS, S);
	}

	// translate from COM-centered world-aligned inertia matrix to a displaced frame
	static PX_INLINE SpInertia translate(const PxVec3& p, const SpInertia& i)
	{
		PxMat33 S = Ps::star(p), ST = S.getTranspose();
		PxMat33 sla = S * i.mLA, llst = i.mLL * ST;
//		return SpInertia(i.mLL, i.mLA + llst, i.mAA + sla + sla.getTranspose() + S * llst);

		// this yields a symmetric result
		PxMat33 t = sla+S*llst*0.5f;
		return SpInertia(i.mLL, i.mLA + llst, i.mAA + (t+t.getTranspose()));	}

	static PX_FORCE_INLINE Cm::SpatialVector axisMultiply(const Cm::SpatialVector* a, const PxVec3& v)
	{
		return a[0]*v[0]+a[1]*v[1]+a[2]*v[2];
	}

	static PX_FORCE_INLINE PxVec3 axisDot(const Cm::SpatialVector* a, const Cm::SpatialVector& v)
	{
		return PxVec3(a[0].dot(v), a[1].dot(v), a[2].dot(v));
	}

	static PX_FORCE_INLINE SpInertia invertInertia(const SpInertia& I)
	{
		PxMat33 aa = I.mAA, ll = I.mLL, la = I.mLA;

		aa = (aa + aa.getTranspose())*0.5f;
		ll = (ll + ll.getTranspose())*0.5f;

		PxMat33 AAInv = invertSym33(aa);

		PxMat33 z = -la * AAInv;
		PxMat33 S = ll + z * la.getTranspose();	// Schur complement of mAA
		
		PxMat33 LL = invertSym33(S);

		PxMat33 LA = LL * z;
		PxMat33 AA = AAInv + z.getTranspose() * LA;

		SpInertia result(LL, LA, AA);

		return result;
	}

	static SpInertia propagate(const SpInertia& I,
							   const Cm::SpatialVector S[3],
							   const PxMat33& load,
							   PxReal isf)
	{
		Cm::SpatialVector IS[3], ISD[3];
		multiply(IS, I, S);
		
		PxMat33 SIS = multiplySym(S, IS);

		// yields a symmetric result
		PxMat33 D = invSqrt(SIS+load*isf);
		multiply(ISD, IS, D);
		return multiplySubtract(I, ISD, ISD);
	}

	static PxMat33 computeDriveInertia(const SpInertia& I0, 
									   const SpInertia& I1, 
									   const Cm::SpatialVector S[3])
	{
		// this could be a lot more efficient, especially since it can be combined with
		// the inertia accumulation. Also it turns out to be symmetric in I0 and I1, which
		// isn't obvious from the formulation, so it's likely there's a more efficient formulation

		PxMat33 D = invertSym33(computeSIS(S,I0));
		Cm::SpatialVector IS[3], ISD[3];

		multiply(IS,I0,S);
		multiply(ISD, IS, D);

		SpInertia tot = multiplySubtract(I0+I1,ISD,IS);
		SpInertia invTot = invertInertia(tot);

		PxMat33 E = computeSIS(ISD,invTot);

		PxMat33 load = invertSym33(E+D);

		PX_ASSERT(load[0].isFinite() && load[1].isFinite() && load[2].isFinite());
		PX_ASSERT(ArticulationDiagnostics::isSymmetric(load) && ArticulationDiagnostics::isPositiveDefinite(load));
		return load;
	}

	static PX_INLINE Cm::SpatialVector propagateImpulse(const FsRow& row, 
														const FsJointVectors& jv,
													    PxVec3& SZ,
													    const Cm::SpatialVector& Z,
													    const FsRowAux& aux)
	{
		PX_UNUSED(aux);
		SZ = Z.angular + Z.linear.cross(getJointOffset(jv));
		Cm::SpatialVector result = translateForce(getParentOffset(jv), Z - axisMultiply(getDSI(row), SZ));

#if DY_ARTICULATION_DEBUG_VERIFY
	PxVec3 SZcheck;
	Cm::SpatialVector check = ArticulationRef::propagateImpulse(row, jv, SZcheck, Z, aux);
	PX_ASSERT((result-check).magnitude()<1e-5*PxMax(check.magnitude(), 1.0f));
	PX_ASSERT((SZ-SZcheck).magnitude()<1e-5*PxMax(SZcheck.magnitude(), 1.0f));
#endif
		return result;
	}

	static PX_INLINE Cm::SpatialVector propagateVelocity(const FsRow& row, 
														 const FsJointVectors& jv,
														 const PxVec3& SZ, 
														 const Cm::SpatialVector& v,
														 const FsRowAux& aux)
	{
		PX_UNUSED(aux);

		Cm::SpatialVector w = translateMotion(-getParentOffset(jv), v);
		PxVec3 DSZ = multiply(row.D, SZ);

		PxVec3 n = axisDot(getDSI(row), w) + DSZ;
		Cm::SpatialVector result = w - Cm::SpatialVector(getJointOffset(jv).cross(n),n);

#if DY_ARTICULATION_DEBUG_VERIFY
	Cm::SpatialVector check = ArticulationRef::propagateVelocity(row, jv, SZ, v, aux);
	PX_ASSERT((result-check).magnitude()<1e-5*PxMax(check.magnitude(), 1.0f));
#endif
		return result;
	}


	static PX_FORCE_INLINE PxVec3 multiply(const Mat33V& m, const PxVec3& v)
	{
		return reinterpret_cast<const PxVec3&>(m.col0) * v.x 
			 + reinterpret_cast<const PxVec3&>(m.col1) * v.y 
			 + reinterpret_cast<const PxVec3&>(m.col2) * v.z;
	}

	static PX_FORCE_INLINE PxVec3 multiplyTranspose(const Mat33V& m, const PxVec3& v)
	{
		return PxVec3(v.dot(reinterpret_cast<const PxVec3&>(m.col0)), 
					  v.dot(reinterpret_cast<const PxVec3&>(m.col1)), 
					  v.dot(reinterpret_cast<const PxVec3&>(m.col2)));
	}

	static Cm::SpatialVector multiply(const FsInertia& m, const Cm::SpatialVector& v)
	{
		return Cm::SpatialVector(multiply(m.ll,v.linear) + multiply(m.la,v.angular),
								 multiplyTranspose(m.la, v.linear) + multiply(m.aa, v.angular));
	}

	static PX_FORCE_INLINE Cm::SpatialVector getRootDeltaV(const FsData& matrix, const Cm::SpatialVector& Z)
	{
		return multiply(getRootInverseInertia(matrix), Z);
	}
};

}

}

#endif //DY_ARTICULATION_SCALAR_FNS_H
