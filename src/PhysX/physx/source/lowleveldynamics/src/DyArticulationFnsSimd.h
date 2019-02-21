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



#ifndef DY_ARTICULATION_SIMD_FNS_H
#define DY_ARTICULATION_SIMD_FNS_H

#include "DyArticulationUtils.h"

namespace physx
{
namespace Dy
{

template <typename T, PxU32 count>
class PodULike
{
	PxU8 space[sizeof(T)*count];
public:
	PX_FORCE_INLINE operator T*() { return reinterpret_cast<T*>(space); }
};

#define POD_U_LIKE(_T, _count, _alignment) PX_ALIGN_PREFIX(_alignment) PodULike<_T, _count> PX_ALIGN_SUFFIX(_alignment)

class ArticulationFnsSimdBase
{
public:

	static PX_FORCE_INLINE	FsInertia addInertia(const FsInertia& in1, const FsInertia& in2)
	{
		return FsInertia(M33Add(in1.ll, in2.ll),
							M33Add(in1.la, in2.la),
							M33Add(in1.aa, in2.aa));
	}

	static PX_FORCE_INLINE	FsInertia subtractInertia(const FsInertia& in1, const FsInertia& in2)
	{
		return FsInertia(M33Sub(in1.ll, in2.ll),
							M33Sub(in1.la, in2.la),
							M33Sub(in1.aa, in2.aa));
	}

	static PX_FORCE_INLINE Vec3V axisDot(const Cm::SpatialVectorV S[3], const Cm::SpatialVectorV &v)
	{
		return V3Merge(FAdd(V3Dot(S[0].linear,v.linear), V3Dot(S[0].angular,v.angular)),
					   FAdd(V3Dot(S[1].linear,v.linear), V3Dot(S[1].angular,v.angular)),
					   FAdd(V3Dot(S[2].linear,v.linear), V3Dot(S[2].angular,v.angular)));
	}

	static PX_FORCE_INLINE Cm::SpatialVectorV axisMultiply(const Cm::SpatialVectorV S[3], Vec3V v)
	{
		return Cm::SpatialVectorV(V3ScaleAdd(S[0].linear, V3GetX(v), V3ScaleAdd(S[1].linear, V3GetY(v), V3Scale(S[2].linear, V3GetZ(v)))),
							  V3ScaleAdd(S[0].angular, V3GetX(v), V3ScaleAdd(S[1].angular, V3GetY(v), V3Scale(S[2].angular, V3GetZ(v)))));
	}


	static PX_FORCE_INLINE Cm::SpatialVectorV subtract(const Cm::SpatialVectorV &a, const Cm::SpatialVectorV &b)
	{
		return Cm::SpatialVectorV(V3Sub(a.linear, b.linear), V3Sub(a.angular, b.angular));
	}

	static PX_FORCE_INLINE Cm::SpatialVectorV add(const Cm::SpatialVectorV &a, const Cm::SpatialVectorV &b)
	{
		return Cm::SpatialVectorV(V3Add(a.linear, b.linear), V3Add(a.angular, b.angular));
	}


	static PX_FORCE_INLINE Cm::SpatialVectorV multiply(const FsInertia &I, const Cm::SpatialVectorV &S)
	{
		return Cm::SpatialVectorV(V3Add(M33MulV3(I.ll,S.linear),	   M33MulV3(I.la,S.angular)),
							  V3Add(M33TrnspsMulV3(I.la,S.linear), M33MulV3(I.aa,S.angular)));
	}


	static PX_FORCE_INLINE Cm::SpatialVectorV translateMotion(const Vec3V& p, const Cm::SpatialVectorV& v)
	{
		return Cm::SpatialVectorV(V3Add(v.linear, V3Cross(p, v.angular)), v.angular);
	}

	// translate a force resolved at position p to the origin

	static PX_FORCE_INLINE Cm::SpatialVectorV translateForce(const Vec3V& p, const Cm::SpatialVectorV& v)
	{
		return Cm::SpatialVectorV(v.linear, V3Add(v.angular, V3Cross(p, v.linear)));
	}

	static PX_FORCE_INLINE Mat33V invertSym33(const Mat33V &m)
	{
		Vec3V a0 = V3Cross(m.col1, m.col2);
		Vec3V a1 = V3Cross(m.col2, m.col0);
		Vec3V a2 = V3Cross(m.col0, m.col1);
		FloatV det = V3Dot(a0, m.col0);
		FloatV recipDet = FRecip(det);

		a1 = V3SetX(a1, V3GetY(a0));
		a2 = V3Merge(V3GetZ(a0), V3GetZ(a1), V3GetZ(a2));		// make sure it's symmetric

		return Mat33V(V3Scale(a0, recipDet),
					  V3Scale(a1, recipDet),
					  V3Scale(a2, recipDet));
	}


	static PX_FORCE_INLINE FloatV safeInvSqrt(FloatV v)
	{
		return FSqrt(FMax(FZero(), FRecip(v)));
	}
	static PX_FORCE_INLINE Mat33V invSqrt(const Mat33V& m)
	{
		// cholesky factor to 
		// (a 0 0)
		// (b c 0)
		// (d e f)
		// except that a,c,f are the reciprocal sqrts rather than sqrts

		// PxVec3 v0 = m.column0, v1 = m.column1, v2 = m.column2;
		Vec3V v0 = m.col0, v1 = m.col1, v2 = m.col2;

		const FloatV x0 = V3GetX(v0), y1 = V3GetY(v1), z2 = V3GetZ(v2);
		
		FloatV a	= safeInvSqrt(x0);								// PxReal a = PxRecipSqrt(v0.x);
	
		Vec3V abd	= V3Scale(v0, a);								// PxReal b = v0.y*a;
		FloatV b	= V3GetY(abd);

		FloatV c2	= FNegScaleSub(b, b, y1);						// PxReal c = PxRecipSqrt(v1.y - b*b);
		FloatV c	= safeInvSqrt(c2);

		FloatV d	= V3GetZ(abd);									// PxReal d = v0.z*a;
		
		FloatV e	= FMul(FNegScaleSub(b, d, V3GetZ(v1)), c);		// PxReal e = (v1.z-d*b) * c;

		FloatV f2	= FNegScaleSub(d, d, FNegScaleSub(e, e, z2));	// PxReal f = PxRecipSqrt(v2.z - d*d - e*e);
		FloatV f	= safeInvSqrt(f2);

		// invert 
		FloatV x = FMul(FMul(b,a),c),								// x = -b*a*c
			   y = FMul((FNegScaleSub(d,a, FMul(e,x))), f),			// y = (-e*x-d*a)*f 
			   z = FMul(e, FMul(c,f));								// z = -e*c*f

		return Mat33V(V3Merge(a,		FZero(),		FZero()),
					  V3Merge(FNeg(x),	c,				FZero()),
					  V3Merge(y,		FNeg(z),		f));
	}


	static PX_FORCE_INLINE FsInertia invertInertia(const FsInertia &I)
	{
		Mat33V aa = M33Scale(M33Add(I.aa, M33Trnsps(I.aa)), FHalf());
		Mat33V ll = M33Scale(M33Add(I.ll, M33Trnsps(I.ll)), FHalf());

		Mat33V AAInv = invertSym33(aa);
		Mat33V z = M33MulM33(M33Neg(I.la), AAInv);
		Mat33V S = M33Add(ll, M33MulM33(z, M33Trnsps(I.la)));

		Mat33V LL = invertSym33(S);
		Mat33V LA = M33MulM33(LL, z);
		Mat33V AA = M33Add(AAInv, M33MulM33(M33Trnsps(z), LA));

		return FsInertia(LL, LA, AA);
	}

	static PX_NOINLINE Mat33V computeSIS(const FsInertia &I, const Cm::SpatialVectorV S[3], Cm::SpatialVectorV IS[3])
	{
		Vec3V S0l = S[0].linear, S0a = S[0].angular;
		Vec3V S1l = S[1].linear, S1a = S[1].angular;
		Vec3V S2l = S[2].linear, S2a = S[2].angular;

		Vec3V IS0l = V3Add(M33MulV3(I.ll,S0l), M33MulV3(I.la,S0a));
		Vec3V IS0a = V3Add(M33TrnspsMulV3(I.la,S0l), M33MulV3(I.aa,S0a));
		Vec3V IS1l = V3Add(M33MulV3(I.ll,S1l), M33MulV3(I.la,S1a));
		Vec3V IS1a = V3Add(M33TrnspsMulV3(I.la,S1l), M33MulV3(I.aa,S1a));
		Vec3V IS2l = V3Add(M33MulV3(I.ll,S2l), M33MulV3(I.la,S2a));
		Vec3V IS2a = V3Add(M33TrnspsMulV3(I.la,S2l), M33MulV3(I.aa,S2a));

		// compute SIS
		FloatV a00 = FAdd(V3Dot(S0l, IS0l), V3Dot(S0a, IS0a));
		FloatV a01 = FAdd(V3Dot(S0l, IS1l), V3Dot(S0a, IS1a));
		FloatV a02 = FAdd(V3Dot(S0l, IS2l), V3Dot(S0a, IS2a));
		FloatV a11 = FAdd(V3Dot(S1l, IS1l), V3Dot(S1a, IS1a));
		FloatV a12 = FAdd(V3Dot(S1l, IS2l), V3Dot(S1a, IS2a));
		FloatV a22 = FAdd(V3Dot(S2l, IS2l), V3Dot(S2a, IS2a));

		// write IS, a useful side-effect
		IS[0].linear = IS0l; IS[0].angular = IS0a;
		IS[1].linear = IS1l; IS[1].angular = IS1a;
		IS[2].linear = IS2l; IS[2].angular = IS2a;

		return Mat33V(V3Merge(a00, a01, a02),
					  V3Merge(a01, a11, a12),
					  V3Merge(a02, a12, a22));
	}


	static PX_FORCE_INLINE FsInertia multiplySubtract(const FsInertia &I, const Mat33V &D, const Cm::SpatialVectorV IS[3], Cm::SpatialVectorV DSI[3])
	{
		// cut'n'paste, how I love ya, how I love ya

		Vec3V IS0l = IS[0].linear, IS0a = IS[0].angular;
		Vec3V IS1l = IS[1].linear, IS1a = IS[1].angular;
		Vec3V IS2l = IS[2].linear, IS2a = IS[2].angular;

		Vec3V D0 = D.col0, D1 = D.col1, D2 = D.col2;

		// compute IDS
		Vec3V DSI0l = V3ScaleAdd(IS0l, V3GetX(D0), V3ScaleAdd(IS1l, V3GetY(D0), V3Scale(IS2l, V3GetZ(D0))));
		Vec3V DSI1l = V3ScaleAdd(IS0l, V3GetX(D1), V3ScaleAdd(IS1l, V3GetY(D1), V3Scale(IS2l, V3GetZ(D1))));
		Vec3V DSI2l = V3ScaleAdd(IS0l, V3GetX(D2), V3ScaleAdd(IS1l, V3GetY(D2), V3Scale(IS2l, V3GetZ(D2))));

		Vec3V DSI0a = V3ScaleAdd(IS0a, V3GetX(D0), V3ScaleAdd(IS1a, V3GetY(D0), V3Scale(IS2a, V3GetZ(D0))));
		Vec3V DSI1a = V3ScaleAdd(IS0a, V3GetX(D1), V3ScaleAdd(IS1a, V3GetY(D1), V3Scale(IS2a, V3GetZ(D1))));
		Vec3V DSI2a = V3ScaleAdd(IS0a, V3GetX(D2), V3ScaleAdd(IS1a, V3GetY(D2), V3Scale(IS2a, V3GetZ(D2))));

		// compute J = I - DSI' IS. Each row of DSI' IS generates an inertia dyad

		Vec3V ll0 = I.ll.col0, ll1 = I.ll.col1, ll2 = I.ll.col2;
		Vec3V la0 = I.la.col0, la1 = I.la.col1, la2 = I.la.col2;
		Vec3V aa0 = I.aa.col0, aa1 = I.aa.col1, aa2 = I.aa.col2;

#define SUBTRACT_DYAD(_a, _b) \
	ll0 = V3NegScaleSub(_b##l, V3GetX(_a##l), ll0);	la0 = V3NegScaleSub(_b##l, V3GetX(_a##a), la0);	aa0 = V3NegScaleSub(_b##a, V3GetX(_a##a), aa0); \
	ll1 = V3NegScaleSub(_b##l, V3GetY(_a##l), ll1);	la1 = V3NegScaleSub(_b##l, V3GetY(_a##a), la1);	aa1 = V3NegScaleSub(_b##a, V3GetY(_a##a), aa1); \
	ll2 = V3NegScaleSub(_b##l, V3GetZ(_a##l), ll2);	la2 = V3NegScaleSub(_b##l, V3GetZ(_a##a), la2);	aa2 = V3NegScaleSub(_b##a, V3GetZ(_a##a), aa2); 

		SUBTRACT_DYAD(IS0, DSI0);
		SUBTRACT_DYAD(IS1, DSI1);
		SUBTRACT_DYAD(IS2, DSI2);
#undef SUBTRACT_DYAD

		DSI[0].linear = DSI0l;	DSI[0].angular = DSI0a;
		DSI[1].linear = DSI1l;	DSI[1].angular = DSI1a;
		DSI[2].linear = DSI2l;	DSI[2].angular = DSI2a;

		return FsInertia(Mat33V(ll0, ll1, ll2), 
							Mat33V(la0, la1, la2),
							Mat33V(aa0, aa1, aa2));
	} 


	static PX_FORCE_INLINE FsInertia multiplySubtract(const FsInertia &I, const Cm::SpatialVectorV S[3])
	{
		// cut'n'paste, how I love ya, how I love ya

		const Vec3V S0l = S[0].linear, S0a = S[0].angular;
		const Vec3V S1l = S[1].linear, S1a = S[1].angular;
		const Vec3V S2l = S[2].linear, S2a = S[2].angular;

		// compute J = I - DSI' IS. Each row of DSI' IS generates an inertia dyad

		Vec3V ll0 = I.ll.col0, ll1 = I.ll.col1, ll2 = I.ll.col2;
		Vec3V la0 = I.la.col0, la1 = I.la.col1, la2 = I.la.col2;
		Vec3V aa0 = I.aa.col0, aa1 = I.aa.col1, aa2 = I.aa.col2;

#define SUBTRACT_DYAD(_a, _b) \
	ll0 = V3NegScaleSub(_b##l, V3GetX(_a##l), ll0);	la0 = V3NegScaleSub(_b##l, V3GetX(_a##a), la0);	aa0 = V3NegScaleSub(_b##a, V3GetX(_a##a), aa0); \
	ll1 = V3NegScaleSub(_b##l, V3GetY(_a##l), ll1);	la1 = V3NegScaleSub(_b##l, V3GetY(_a##a), la1);	aa1 = V3NegScaleSub(_b##a, V3GetY(_a##a), aa1); \
	ll2 = V3NegScaleSub(_b##l, V3GetZ(_a##l), ll2);	la2 = V3NegScaleSub(_b##l, V3GetZ(_a##a), la2);	aa2 = V3NegScaleSub(_b##a, V3GetZ(_a##a), aa2); 

	SUBTRACT_DYAD(S0, S0);
	SUBTRACT_DYAD(S1, S1);
	SUBTRACT_DYAD(S2, S2);
#undef SUBTRACT_DYAD

		return FsInertia(Mat33V(ll0, ll1, ll2), 
							Mat33V(la0, la1, la2),
							Mat33V(aa0, aa1, aa2));
	} 


	static PX_FORCE_INLINE FsInertia translateInertia(Vec3V a, const FsInertia &input)
	{
		Vec3V b = V3Neg(a);
		
		Vec3V la0 = input.la.col0, la1 = input.la.col1, la2 = input.la.col2;
		Vec3V ll0 = input.ll.col0, ll1 = input.ll.col1, ll2 = input.ll.col2;
		Vec3V aa0 = input.aa.col0, aa1 = input.aa.col1, aa2 = input.aa.col2;

		FloatV aX = V3GetX(a), aY = V3GetY(a), aZ = V3GetZ(a);
		FloatV bX = V3GetX(b), bY = V3GetY(b), bZ = V3GetZ(b);
		FloatV Z = FZero();

		// s - star matrix of a
		Vec3V s0 = V3Merge(Z, aZ, bY),
			  s1 = V3Merge(bZ, Z, aX),
			  s2 = V3Merge(aY, bX, Z);

		// s * la
		Vec3V sla0 = V3ScaleAdd(s0, V3GetX(la0), V3ScaleAdd(s1, V3GetY(la0), V3Scale(s2, V3GetZ(la0))));
		Vec3V sla1 = V3ScaleAdd(s0, V3GetX(la1), V3ScaleAdd(s1, V3GetY(la1), V3Scale(s2, V3GetZ(la1))));
		Vec3V sla2 = V3ScaleAdd(s0, V3GetX(la2), V3ScaleAdd(s1, V3GetY(la2), V3Scale(s2, V3GetZ(la2))));

		// ll * s.transpose() (ll is symmetric)
		Vec3V llst0 = V3ScaleAdd(ll2, aY, V3Scale(ll1, bZ)),
			  llst1 = V3ScaleAdd(ll0, aZ, V3Scale(ll2, bX)),
			  llst2 = V3ScaleAdd(ll1, aX, V3Scale(ll0, bY));

		// t = sla+S*llst*0.5f;

		Vec3V sllst0 = V3ScaleAdd(s2, V3GetZ(llst0), V3ScaleAdd(s1, V3GetY(llst0), V3Scale(s0, V3GetX(llst0))));
		Vec3V sllst1 = V3ScaleAdd(s2, V3GetZ(llst1), V3ScaleAdd(s1, V3GetY(llst1), V3Scale(s0, V3GetX(llst1))));
		Vec3V sllst2 = V3ScaleAdd(s2, V3GetZ(llst2), V3ScaleAdd(s1, V3GetY(llst2), V3Scale(s0, V3GetX(llst2))));

		Vec3V t0 = V3ScaleAdd(sllst0, FHalf(), sla0);
		Vec3V t1 = V3ScaleAdd(sllst1, FHalf(), sla1);
		Vec3V t2 = V3ScaleAdd(sllst2, FHalf(), sla2);
		
		// t+t.transpose()
		Vec3V r0 = V3Add(t0, V3Merge(V3GetX(t0), V3GetX(t1), V3GetX(t2))),
			  r1 = V3Add(t1, V3Merge(V3GetY(t0), V3GetY(t1), V3GetY(t2))),
			  r2 = V3Add(t2, V3Merge(V3GetZ(t0), V3GetZ(t1), V3GetZ(t2)));

		return FsInertia(Mat33V(ll0, ll1, ll2),

							Mat33V(V3Add(la0, llst0),
								   V3Add(la1, llst1),
								   V3Add(la2, llst2)),

							Mat33V(V3Add(aa0, r0),
								   V3Add(aa1, r1),
								   V3Add(aa2, r2)));
	}

};

template<class Base>
class ArticulationFnsSimd : public Base
{
	static PX_FORCE_INLINE void axisMultiplyLowerTriangular(Cm::SpatialVectorV ES[3], const Mat33V&E, const Cm::SpatialVectorV S[3])
	{
		const Vec3V l0 = S[0].linear,  l1 = S[1].linear,  l2 = S[2].linear;
		const Vec3V a0 = S[0].angular, a1 = S[1].angular, a2 = S[2].angular;
		ES[0] = Cm::SpatialVectorV(V3Scale(l0, V3GetX(E.col0)),
							   V3Scale(a0, V3GetX(E.col0)));
		ES[1] = Cm::SpatialVectorV(V3ScaleAdd(l0, V3GetX(E.col1), V3Scale(l1, V3GetY(E.col1))),
							   V3ScaleAdd(a0, V3GetX(E.col1), V3Scale(a1, V3GetY(E.col1))));
		ES[2] = Cm::SpatialVectorV(V3ScaleAdd(l0, V3GetX(E.col2), V3ScaleAdd(l1, V3GetY(E.col2), V3Scale(l2, V3GetZ(E.col2)))),
							   V3ScaleAdd(a0, V3GetX(E.col2), V3ScaleAdd(a1, V3GetY(E.col2), V3Scale(a2, V3GetZ(E.col2)))));
	}

public:
	static PX_FORCE_INLINE FsInertia propagate(const FsInertia &I,
												  const Cm::SpatialVectorV S[3],
												  const Mat33V &load,
												  const FloatV isf)
	{
		Cm::SpatialVectorV IS[3], ISE[3];
		Mat33V D = Base::computeSIS(I, S, IS);

		D.col0 = V3ScaleAdd(load.col0, isf, D.col0);
		D.col1 = V3ScaleAdd(load.col1, isf, D.col1);
		D.col2 = V3ScaleAdd(load.col2, isf, D.col2);

		axisMultiplyLowerTriangular(ISE, Base::invSqrt(D), IS);
		return Base::multiplySubtract(I, ISE);
	}



	static PX_INLINE Cm::SpatialVectorV propagateImpulse(const FsRow& row, 
													 const FsJointVectors& jv,
												     Vec3V& SZ,
												     const Cm::SpatialVectorV& Z,
												     const FsRowAux& aux)
	{
		PX_UNUSED(aux);

		SZ = V3Add(Z.angular, V3Cross(Z.linear, jv.jointOffset));
		return Base::translateForce(jv.parentOffset, Z - Base::axisMultiply(row.DSI, SZ));
	}

	static PX_INLINE Cm::SpatialVectorV propagateVelocity(const FsRow& row, 
													  const FsJointVectors& jv,
													  const Vec3V& SZ, 
													  const Cm::SpatialVectorV& v,
													  const FsRowAux& aux)
	{
		PX_UNUSED(aux);

		Cm::SpatialVectorV w = Base::translateMotion(V3Neg(jv.parentOffset), v);
		Vec3V DSZ = M33MulV3(row.D, SZ);

		Vec3V n = V3Add(Base::axisDot(row.DSI, w), DSZ);
		return w - Cm::SpatialVectorV(V3Cross(jv.jointOffset, n), n);
	}





	static PX_FORCE_INLINE Mat33V computeDriveInertia(const FsInertia &I0, 
													  const FsInertia &I1, 
													  const Cm::SpatialVectorV S[3])
	{
		POD_U_LIKE(Cm::SpatialVectorV, 3, 16) IS, ISD, dummy;		
		Mat33V D = Base::computeSIS(I0, S, IS);
		Mat33V DInv = Base::invertSym33(D);

		FsInertia tmp = Base::addInertia(I0, I1);
		tmp = Base::multiplySubtract(tmp, DInv, IS, ISD);
		FsInertia J = Base::invertInertia(tmp);

		Mat33V E = Base::computeSIS(J, ISD, dummy);
		return Base::invertSym33(M33Add(DInv,E));

	}
};

}
}

#endif //DY_ARTICULATION_SIMD_FNS_H
