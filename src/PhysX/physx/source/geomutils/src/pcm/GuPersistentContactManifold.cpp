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

#include "foundation/PxMemory.h"
#include "CmPhysXCommon.h"
#include "GuPersistentContactManifold.h"
#include "GuContactBuffer.h"
#include "PsAllocator.h"
#include "PsVecTransform.h"
#include "PsUtilities.h"
#include "GuGJKUtil.h"

using namespace physx;

namespace physx
{
namespace Gu
{

/*
	This local function is to avoid DLL call
*/
static Ps::aos::FloatV distancePointSegmentSquaredLocal(const Ps::aos::Vec3VArg a, const Ps::aos::Vec3VArg b, const Ps::aos::Vec3VArg p)
{
	using namespace Ps::aos;
	const FloatV zero = FZero();
	const FloatV one = FOne();

	const Vec3V ap = V3Sub(p, a);
	const Vec3V ab = V3Sub(b, a);
	const FloatV nom = V3Dot(ap, ab);
	
	const FloatV denom = V3Dot(ab, ab);
	const FloatV tValue = FClamp(FDiv(nom, denom), zero, one);

	const FloatV t = FSel(FIsEq(denom, zero), zero, tValue);
	const Vec3V v = V3NegScaleSub(ab, t, ap);
	return V3Dot(v, v);
}

/*
	This local function is to avoid DLL call
*/
static Ps::aos::FloatV distancePointTriangleSquaredLocal(	const Ps::aos::Vec3VArg p, 
													const Ps::aos::Vec3VArg a, 
													const Ps::aos::Vec3VArg b, 
													const Ps::aos::Vec3VArg c)
{
	using namespace Ps::aos;

	const FloatV zero = FZero();
	//const Vec3V zero = V3Zero();
	const Vec3V ab = V3Sub(b, a);
	const Vec3V ac = V3Sub(c, a);
	const Vec3V bc = V3Sub(c, b);
	const Vec3V ap = V3Sub(p, a);
	const Vec3V bp = V3Sub(p, b);
	const Vec3V cp = V3Sub(p, c);

	const FloatV d1 = V3Dot(ab, ap); //  snom
	const FloatV d2 = V3Dot(ac, ap); //  tnom
	const FloatV d3 = V3Dot(ab, bp); // -sdenom
	const FloatV d4 = V3Dot(ac, bp); //  unom = d4 - d3
	const FloatV d5 = V3Dot(ab, cp); //  udenom = d5 - d6
	const FloatV d6 = V3Dot(ac, cp); // -tdenom
	const FloatV unom = FSub(d4, d3);
	const FloatV udenom = FSub(d5, d6);
	
	//check if p in vertex region outside a
	const BoolV con00 = FIsGrtr(zero, d1); // snom <= 0
	const BoolV con01 = FIsGrtr(zero, d2); // tnom <= 0
	const BoolV con0 = BAnd(con00, con01); // vertex region a

	if(BAllEqTTTT(con0))
	{
		const Vec3V vv = V3Sub(p, a);
		return V3Dot(vv, vv);
	}

	//check if p in vertex region outside b
	const BoolV con10 = FIsGrtrOrEq(d3, zero);
	const BoolV con11 = FIsGrtrOrEq(d3, d4);
	const BoolV con1 = BAnd(con10, con11); // vertex region b
	if(BAllEqTTTT(con1))
	{
		const Vec3V vv = V3Sub(p, b);
		return V3Dot(vv, vv);
	}

	//check if p in vertex region outside c
	const BoolV con20 = FIsGrtrOrEq(d6, zero);
	const BoolV con21 = FIsGrtrOrEq(d6, d5); 
	const BoolV con2 = BAnd(con20, con21); // vertex region c
	if(BAllEqTTTT(con2))
	{
		const Vec3V vv = V3Sub(p, c);
		return V3Dot(vv, vv);
	}

	//check if p in edge region of AB
	const FloatV vc = FSub(FMul(d1, d4), FMul(d3, d2));
	
	const BoolV con30 = FIsGrtr(zero, vc);
	const BoolV con31 = FIsGrtrOrEq(d1, zero);
	const BoolV con32 = FIsGrtr(zero, d3);
	const BoolV con3 = BAnd(con30, BAnd(con31, con32));
	if(BAllEqTTTT(con3))
	{
		const FloatV sScale = FDiv(d1, FSub(d1, d3));
		const Vec3V closest3 = V3ScaleAdd(ab, sScale, a);//V3Add(a, V3Scale(ab, sScale));
		const Vec3V vv = V3Sub(p, closest3);
		return V3Dot(vv, vv);
	}

	//check if p in edge region of BC
	const FloatV va = FSub(FMul(d3, d6),FMul(d5, d4));
	const BoolV con40 = FIsGrtr(zero, va);
	const BoolV con41 = FIsGrtrOrEq(d4, d3);
	const BoolV con42 = FIsGrtrOrEq(d5, d6);
	const BoolV con4 = BAnd(con40, BAnd(con41, con42)); 
	if(BAllEqTTTT(con4))
	{
		const FloatV uScale = FDiv(unom, FAdd(unom, udenom));
		const Vec3V closest4 = V3ScaleAdd(bc, uScale, b);//V3Add(b, V3Scale(bc, uScale));
		const Vec3V vv = V3Sub(p, closest4);
		return V3Dot(vv, vv);
	}

	//check if p in edge region of AC
	const FloatV vb = FSub(FMul(d5, d2), FMul(d1, d6));
	const BoolV con50 = FIsGrtr(zero, vb);
	const BoolV con51 = FIsGrtrOrEq(d2, zero);
	const BoolV con52 = FIsGrtr(zero, d6);
	const BoolV con5 = BAnd(con50, BAnd(con51, con52));
	if(BAllEqTTTT(con5))
	{
		const FloatV tScale = FDiv(d2, FSub(d2, d6));
		const Vec3V closest5 = V3ScaleAdd(ac, tScale, a);//V3Add(a, V3Scale(ac, tScale));
		const Vec3V vv = V3Sub(p, closest5);
		return V3Dot(vv, vv);
	}

	//P must project inside face region. Compute Q using Barycentric coordinates
	const Vec3V n = V3Cross(ab, ac);
	const FloatV nn = V3Dot(n,n);
	const FloatV t = FSel(FIsGrtr(nn, zero),FDiv(V3Dot(n, V3Sub(a, p)), nn), zero);
	const Vec3V closest6 = V3Add(p, V3Scale(n, t));

	const Vec3V vv = V3Sub(p, closest6);

	return V3Dot(vv, vv);
}




//This is the translational threshold used in invalidate_BoxConvexHull. 0.5 is 50% of the object margin. we use different threshold between
//0 and 4 points. This threashold is a scale that is multiplied by the objects' margins.
const PxF32 invalidateThresholds[5] = {	0.5f, 0.125f, 0.25f, 0.375f, 0.375f	};

//This is the translational threshold used in invalidate_SphereCapsule. 0.5 is 50% of the object margin, we use different threshold between
//0 and 2 points. This threshold is a scale that is multiplied by the objects' margin

const PxF32 invalidateThresholds2[3] = { 0.5f, 0.1f, 0.75f	};

//This is the rotational threshold used in invalidate_BoxConvexHull. 0.9998 is a threshold for quat difference
//between previous and current frame
const PxF32 invalidateQuatThresholds[5] = {	0.9998f, 0.9999f, 0.9999f, 0.9999f, 0.9999f	};


//This is the rotational threshold used in invalidate_SphereCapsule. 0.9995f is a threshold for quat difference
//between previous and current frame
const PxF32 invalidateQuatThresholds2[3] = { 0.9995f, 0.9999f, 0.9997f	};

}
}


#if VISUALIZE_PERSISTENT_CONTACT
#include "CmRenderOutput.h"

static void drawManifoldPoint(const Gu::PersistentContact& manifold,  const Ps::aos::PsTransformV& trA, const Ps::aos::PsTransformV& trB, Cm::RenderOutput& out, PxU32 color=0xffffff)
{
	PX_UNUSED(color);

	using namespace Ps::aos;
	const Vec3V worldA = trA.transform(manifold.mLocalPointA);
	const Vec3V worldB = trB.transform(manifold.mLocalPointB);
	const Vec3V localNormal = Vec3V_From_Vec4V(manifold.mLocalNormalPen);
	const FloatV pen = V4GetW(manifold.mLocalNormalPen);
	
	const Vec3V worldNormal = trB.rotate(localNormal);
	PxVec3 a, b, v;
	V3StoreU(worldA, a);
	V3StoreU(worldB, b);
	V3StoreU(worldNormal, v);
	PxF32 dist;
	FStore(pen, &dist);
	PxVec3 e = a - v*dist;

	PxF32 size = 0.05f;
	const PxVec3 up(0.f, size, 0.f);
	const PxVec3 right(size, 0.f, 0.f);
	const PxVec3 forwards(0.f, 0.f, size);

	PxF32 size2 = 0.1f;
	const PxVec3 up2(0.f, size2, 0.f);
	const PxVec3 right2(size2, 0.f, 0.f);
	const PxVec3 forwards2(0.f, 0.f, size2);
	
	PxMat44 m = PxMat44(PxIdentity);
	
	out << 0xffff00ff << m << Cm::RenderOutput::LINES  << a << e;
	out << 0xff00ffff << m << Cm::RenderOutput::LINES << a + up << a - up;
	out << 0xff00ffff << m << Cm::RenderOutput::LINES << a + right << a - right;
	out << 0xff00ffff << m << Cm::RenderOutput::LINES << a + forwards << a - forwards;

	out << 0xffff0000 << m << Cm::RenderOutput::LINES << b + up2 << b - up2;
	out << 0xffff0000 << m << Cm::RenderOutput::LINES << b + right2 << b - right2;
	out << 0xffff0000 << m << Cm::RenderOutput::LINES << b + forwards2 << b - forwards2;

	out << 0xffff0000 << m << Cm::RenderOutput::LINES << a << b;

	PxVec3 c = a - v*10.f;
	out << 0xffff00ff << m << Cm::RenderOutput::LINES << a << c;

}

static void drawManifoldPoint(const Gu::PersistentContact& manifold,  const Ps::aos::PsTransformV& trA, const Ps::aos::PsTransformV& trB, const Ps::aos::FloatVArg radius, Cm::RenderOutput& out, PxU32 color=0xffffff)
{
	PX_UNUSED(color);

	using namespace Ps::aos;

	const Vec3V localNormal = Vec3V_From_Vec4V(manifold.mLocalNormalPen);
	const Vec3V worldNormal = trB.rotate(localNormal);
	const Vec3V worldA = V3NegScaleSub(worldNormal, radius, trA.transform(manifold.mLocalPointA));
	const Vec3V worldB = trB.transform(manifold.mLocalPointB);
	const FloatV pen = FSub(V4GetW(manifold.mLocalNormalPen), radius);
	
	PxVec3 a, b, v;
	V3StoreU(worldA, a);
	V3StoreU(worldB, b);
	V3StoreU(worldNormal, v);
	PxF32 dist;
	FStore(pen, &dist);
	PxVec3 e = a - v*dist;

	PxF32 size = 0.05f;
	const PxVec3 up(0.f, size, 0.f);
	const PxVec3 right(size, 0.f, 0.f);
	const PxVec3 forwards(0.f, 0.f, size);

	PxF32 size2 = 0.1f;
	const PxVec3 up2(0.f, size2, 0.f);
	const PxVec3 right2(size2, 0.f, 0.f);
	const PxVec3 forwards2(0.f, 0.f, size2);
	
	PxMat44 m = PxMat44(PxIdentity);
	
	out << 0xffff00ff << m << Cm::RenderOutput::LINES  << a << e;
	out << 0xff00ffff << m << Cm::RenderOutput::LINES << a + up << a - up;
	out << 0xff00ffff << m << Cm::RenderOutput::LINES << a + right << a - right;
	out << 0xff00ffff << m << Cm::RenderOutput::LINES << a + forwards << a - forwards;

	out << 0xffff0000 << m << Cm::RenderOutput::LINES << b + up2 << b - up2;
	out << 0xffff0000 << m << Cm::RenderOutput::LINES << b + right2 << b - right2;
	out << 0xffff0000 << m << Cm::RenderOutput::LINES << b + forwards2 << b - forwards2;

	out << 0xffff0000 << m << Cm::RenderOutput::LINES << a << b;

}


static PxU32 gColors[8] = { 0xff0000ff, 0xff00ff00, 0xffff0000,
							0xff00ffff, 0xffff00ff, 0xffffff00,
							0xff000080, 0xff008000};

#endif

/*
	SIMD version 
*/
Ps::aos::Mat33V Gu::findRotationMatrixFromZAxis(const Ps::aos::Vec3VArg to)
{
	using namespace Ps::aos;
	
	const FloatV one = FOne();
	const FloatV threshold = FLoad(0.9999f);

	const FloatV e = V3GetZ(to);
	const FloatV f = FAbs(e);

	if(FAllGrtr(threshold, f))
	{
		const FloatV vx = FNeg(V3GetY(to));
		const FloatV vy = V3GetX(to);
		const FloatV h = FRecip(FAdd(one, e)); 
		const FloatV hvx = FMul(h,vx);
		const FloatV hvxy = FMul(hvx, vy);

		const Vec3V col0 = V3Merge(FScaleAdd(hvx, vx, e),		hvxy,							      vy);
		const Vec3V col1 = V3Merge(hvxy,				   FScaleAdd(h, FMul(vy, vy), e),			FNeg(vx));
		const Vec3V col2 = V3Merge(FNeg(vy),					   vx,							  e);

		return Mat33V(col0, col1, col2);

	}
	else
	{

		const FloatV two = FLoad(2.f);
		const Vec3V from = V3UnitZ();
		const Vec3V absFrom = V3UnitY();

		const Vec3V u = V3Sub(absFrom, from);
		const Vec3V v = V3Sub(absFrom, to);

		const FloatV dotU = V3Dot(u, u);
		const FloatV dotV = V3Dot(v, v);
		const FloatV dotUV = V3Dot(u, v);

		const FloatV c1 = FNeg(FDiv(two, dotU));
		const FloatV c2 = FNeg(FDiv(two, dotV));
		const FloatV c3 = FMul(c1, FMul(c2, dotUV));

		const Vec3V c1u = V3Scale(u, c1);
		const Vec3V c2v = V3Scale(v, c2);
		const Vec3V c3v = V3Scale(v, c3);


		FloatV temp0 = V3GetX(c1u);
		FloatV temp1 = V3GetX(c2v);
		FloatV temp2 = V3GetX(c3v);

		Vec3V col0 = V3ScaleAdd(u, temp0, V3ScaleAdd(v, temp1, V3Scale(u, temp2)));
		col0 = V3SetX(col0, FAdd(V3GetX(col0), one));

		temp0 = V3GetY(c1u);
		temp1 = V3GetY(c2v);
		temp2 = V3GetY(c3v);

		Vec3V col1 = V3ScaleAdd(u, temp0, V3ScaleAdd(v, temp1, V3Scale(u, temp2)));
		col1 = V3SetY(col1, FAdd(V3GetY(col1), one));

		temp0 = V3GetZ(c1u);
		temp1 = V3GetZ(c2v);
		temp2 = V3GetZ(c3v);

		Vec3V col2 = V3ScaleAdd(u, temp0, V3ScaleAdd(v, temp1, V3Scale(u, temp2)));
		col2 = V3SetZ(col2, FAdd(V3GetZ(col2), one));

		return Mat33V(col0, col1, col2);

	}
}


void Gu::PersistentContactManifold::drawManifold( Cm::RenderOutput& out, const Ps::aos::PsTransformV& trA, const Ps::aos::PsTransformV& trB)
{
	using namespace Ps::aos;
#if VISUALIZE_PERSISTENT_CONTACT

	PxVec3 a, b;
	V3StoreU(trA.p, a);
	V3StoreU(trB.p, b);

	for(PxU32 i = 0; i< mNumContacts; ++i)
	{
		Gu::PersistentContact& m = mContactPoints[i];
		drawManifoldPoint(m, trA, trB, out, gColors[i]);
	}
#else
	PX_UNUSED(out);
	PX_UNUSED(trA);
	PX_UNUSED(trB);
#endif
}

void Gu::PersistentContactManifold::drawManifold( Cm::RenderOutput& out, const Ps::aos::PsTransformV& trA, const Ps::aos::PsTransformV& trB, const Ps::aos::FloatVArg radius)
{
	using namespace Ps::aos;
#if VISUALIZE_PERSISTENT_CONTACT

	PxVec3 a, b;
	V3StoreU(trA.p, a);
	V3StoreU(trB.p, b);

	for(PxU32 i = 0; i< mNumContacts; ++i)
	{
		Gu::PersistentContact& m = mContactPoints[i];
		drawManifoldPoint(m, trA, trB, radius, out, gColors[i]);
	}
#else
	PX_UNUSED(out);
	PX_UNUSED(trA);
	PX_UNUSED(trB);
	PX_UNUSED(radius);
#endif
}

void Gu::PersistentContactManifold::drawManifold(const Gu::PersistentContact& m, Cm::RenderOutput& out, const Ps::aos::PsTransformV& trA, const Ps::aos::PsTransformV& trB)
{	
#if VISUALIZE_PERSISTENT_CONTACT
	drawManifoldPoint(m, trA, trB, out, gColors[0]);
#else
	PX_UNUSED(out);
	PX_UNUSED(trA);
	PX_UNUSED(trB);
	PX_UNUSED(m);
#endif
}

void Gu::PersistentContactManifold::drawPoint(Cm::RenderOutput& out, const Ps::aos::Vec3VArg p, const PxF32 size, const PxU32 color)
{
	using namespace Ps::aos;
#if VISUALIZE_PERSISTENT_CONTACT
	const PxVec3 up(0.f, size, 0.f);
	const PxVec3 right(size, 0.f, 0.f);
	const PxVec3 forwards(0.f, 0.f, size);

	PxVec3 a;
	V3StoreU(p, a);

	PxMat44 m = PxMat44(PxIdentity);
	
	out << color << m << Cm::RenderOutput::LINES << a + up << a - up;
	out << color << m << Cm::RenderOutput::LINES << a + right << a - right;
	out << color << m << Cm::RenderOutput::LINES << a + forwards << a - forwards;
#else
	PX_UNUSED(out);
	PX_UNUSED(p);
	PX_UNUSED(size);
	PX_UNUSED(color);
#endif
}

void Gu::PersistentContactManifold::drawLine(Cm::RenderOutput& out, const Ps::aos::Vec3VArg p0, const Ps::aos::Vec3VArg p1, const PxU32 color)
{
	using namespace Ps::aos;
#if VISUALIZE_PERSISTENT_CONTACT
	PxVec3 a, b;
	V3StoreU(p0, a);
	V3StoreU(p1, b);

	PxMat44 m = PxMat44(PxIdentity);
	out << color << m << Cm::RenderOutput::LINES << a << b;
#else
	PX_UNUSED(out);
	PX_UNUSED(p0);
	PX_UNUSED(p1);
	PX_UNUSED(color);
#endif  
}

void Gu::PersistentContactManifold::drawTriangle(Cm::RenderOutput& out, const Ps::aos::Vec3VArg p0, const Ps::aos::Vec3VArg p1, const Ps::aos::Vec3VArg p2, const PxU32 color)
{
	using namespace Ps::aos;
#if VISUALIZE_PERSISTENT_CONTACT
	PxVec3 a, b, c;
	V3StoreU(p0, a);
	V3StoreU(p1, b);
	V3StoreU(p2, c);

	PxMat44 m = PxMat44(PxIdentity);
	out << color << m << Cm::RenderOutput::TRIANGLES << a << b << c;
#else
	PX_UNUSED(out);
	PX_UNUSED(p0);
	PX_UNUSED(p1);
	PX_UNUSED(p2);
	PX_UNUSED(color);
#endif
}

void Gu::PersistentContactManifold::drawPolygon( Cm::RenderOutput& out, const Ps::aos::PsTransformV& transform,  Ps::aos::Vec3V* points, const PxU32 numVerts, const PxU32 color)
{
	using namespace Ps::aos;
#if VISUALIZE_PERSISTENT_CONTACT
	for(PxU32 i=0; i<numVerts; ++i)
	{
		Vec3V tempV0 = points[i == 0 ? numVerts-1 : i-1];
		Vec3V tempV1 = points[i];
		
		drawLine(out, transform.transform(tempV0), transform.transform(tempV1), color);
	}
#else
	PX_UNUSED(out);
	PX_UNUSED(transform);
	PX_UNUSED(points);
	PX_UNUSED(numVerts);
	PX_UNUSED(color);
#endif

}

void Gu::PersistentContactManifold::drawPolygon( Cm::RenderOutput& out, const Ps::aos::PsMatTransformV& transform,  Ps::aos::Vec3V* points, const PxU32 numVerts, const PxU32 color)
{
	using namespace Ps::aos;
#if VISUALIZE_PERSISTENT_CONTACT
	for(PxU32 i=0; i<numVerts; ++i)
	{
		Vec3V tempV0 = points[i == 0 ? numVerts-1 : i-1];
		Vec3V tempV1 = points[i];
		
		drawLine(out, transform.transform(tempV0), transform.transform(tempV1), color);
	}
#else
	PX_UNUSED(out);
	PX_UNUSED(transform);
	PX_UNUSED(points);
	PX_UNUSED(numVerts);
	PX_UNUSED(color);
#endif

}

void Gu::PersistentContactManifold::recordWarmStart(PxU8* aIndices, PxU8* bIndices, PxU8& nbWarmStartPoints)
{
	nbWarmStartPoints = mNumWarmStartPoints;
	for (PxU8 i = 0; i < mNumWarmStartPoints; ++i)
	{
		aIndices[i] = mAIndice[i];
		bIndices[i] = mBIndice[i];
	}
}

void Gu::PersistentContactManifold::setWarmStart(const PxU8* aIndices, const PxU8* bIndices, const PxU8 nbWarmStartPoints)
{
	mNumWarmStartPoints = nbWarmStartPoints;
	for (PxU8 i = 0; i < nbWarmStartPoints; ++i)
	{
		mAIndice[i] = aIndices[i];
		mBIndice[i] = bIndices[i];
	}
}

/*
	If a new point and the exisitng point's distance are within some replace breaking threshold, we will replace the existing point with the new point. This is used for
	incremental manifold strategy.
*/
bool Gu::PersistentContactManifold::replaceManifoldPoint(const Ps::aos::Vec3VArg localPointA, const Ps::aos::Vec3VArg localPointB, const Ps::aos::Vec4VArg localNormalPen
														, const Ps::aos::FloatVArg replaceBreakingThreshold)
{
	using namespace Ps::aos;

	const FloatV shortestDist =  FMul(replaceBreakingThreshold, replaceBreakingThreshold); 
	
	for( PxU32 i = 0; i < mNumContacts; ++i )
	{
		const PersistentContact& mp = mContactPoints[i];

		const Vec3V diffB =  V3Sub(mp.mLocalPointB, localPointB);
		const FloatV sqDifB = V3Dot(diffB, diffB);
		const Vec3V diffA = V3Sub(mp.mLocalPointA, localPointA);
		const FloatV sqDifA = V3Dot(diffA, diffA);
		const FloatV minSqDif = FMin(sqDifB, sqDifA);

		if (FAllGrtr(shortestDist, minSqDif))
		{
			mContactPoints[i].mLocalPointA = localPointA;
			mContactPoints[i].mLocalPointB = localPointB;
			mContactPoints[i].mLocalNormalPen = localNormalPen;
			return true;
		}

	}

	return false;
}



PxU32 Gu::PersistentContactManifold::reduceContactSegment(const Ps::aos::Vec3VArg localPointA, const Ps::aos::Vec3VArg localPointB, const Ps::aos::Vec4VArg localNormalPen)
{
	using namespace Ps::aos;
	const Vec3V p  = localPointB;
	const Vec3V p0 = mContactPoints[0].mLocalPointB;
	const Vec3V p1 = mContactPoints[1].mLocalPointB;
	const Vec3V v0 = V3Sub(p0, p);
	const Vec3V v1 = V3Sub(p1, p);
	const FloatV dist0 = V3Dot(v0, v0);
	const FloatV dist1 = V3Dot(v1, v1);
	if(FAllGrtr(dist0, dist1))
	{
		mContactPoints[1].mLocalPointA = localPointA;
		mContactPoints[1].mLocalPointB = localPointB;
		mContactPoints[1].mLocalNormalPen = localNormalPen;
	}
	else
	{
		mContactPoints[0].mLocalPointA = localPointA;
		mContactPoints[0].mLocalPointB = localPointB;
		mContactPoints[0].mLocalNormalPen = localNormalPen;
	}

	return 0;
}



PxU32 Gu::PersistentContactManifold::reduceContactsForPCM(const Ps::aos::Vec3VArg localPointA, const Ps::aos::Vec3VArg localPointB, const Ps::aos::Vec4VArg localNormalPen)
{
	using namespace Ps::aos;


	bool chosen[5];
	physx::PxMemZero(chosen, sizeof(bool)*5);
	const FloatV negMax = FNeg(FMax());
	PersistentContact tempContacts[5];
	
	for(PxU32 i=0; i<4; ++i)
	{
		tempContacts[i] = mContactPoints[i];
	}
	tempContacts[4].mLocalPointA = localPointA;
	tempContacts[4].mLocalPointB = localPointB;
	tempContacts[4].mLocalNormalPen = localNormalPen;

	//ML: we set the start point to be the 4th point
	FloatV maxDist =V4GetW(localNormalPen);
	PxI32 index = 4;
	//Choose deepest point
	for(PxI32 i=0; i<4; ++i)
	{
		const FloatV pen = V4GetW(tempContacts[i].mLocalNormalPen);
		if(FAllGrtr(maxDist, pen))
		{
			maxDist = pen;
			index = i;
		}
	}

	chosen[index] = true;
	mContactPoints[0] = tempContacts[index];

	//ML: we set the start point to be the 0th point 
	Vec3V dir= V3Sub(tempContacts[0].mLocalPointB, mContactPoints[0].mLocalPointB);
	maxDist = V3Dot(dir, dir);
	index = 0;

	for(PxI32 i=1; i<5; ++i)
	{
		if(!chosen[i])
		{
			dir = V3Sub(tempContacts[i].mLocalPointB, mContactPoints[0].mLocalPointB);
			const FloatV d = V3Dot(dir, dir);
			if(FAllGrtr(d, maxDist))
			{
				maxDist = d;
				index = i;
			}
		}
	}

	//PX_ASSERT(chosen[index] == false);
	chosen[index] = true;
	mContactPoints[1] = tempContacts[index];

	maxDist = negMax;	
	for(PxI32 i=0; i<5; ++i)
	{
		if(!chosen[i])
		{
			const FloatV sqDif = distancePointSegmentSquaredLocal(mContactPoints[0].mLocalPointB, mContactPoints[1].mLocalPointB, tempContacts[i].mLocalPointB);
			if(FAllGrtr(sqDif, maxDist))
			{
				maxDist = sqDif;
				index = i;
			}
		}
	}
	//PX_ASSERT(chosen[index] == false);
	chosen[index] = true;
	mContactPoints[2]=tempContacts[index];

	//Find point farthest away from segment tempContactPoints[0] - tempContactPoints[1]
	maxDist = negMax;
	for(PxI32 i=0; i<5; ++i)
	{
		if(!chosen[i])
		{
			const FloatV sqDif = distancePointTriangleSquaredLocal(	tempContacts[i].mLocalPointB, mContactPoints[0].mLocalPointB, mContactPoints[1].mLocalPointB, mContactPoints[2].mLocalPointB); 
			if(FAllGrtr(sqDif, maxDist))
			{
				maxDist= sqDif;
				index = i;
			}
		}
	}

	//PX_ASSERT(chosen[index] == false);
	if(chosen[index] == true)
	{
		//if we don't have any new contacts, which means the leftover contacts are inside the triangles
		mNumContacts = 3;
		return 0;
	}
	else
	{
		chosen[index] = true;
		mContactPoints[3]=tempContacts[index];
	}

	//Final pass, we work out the index that we didn't choose and bind it to its closest point. We then consider whether we want to swap the point if the
	//point we were about to discard is deeper...

	PxU32 notChosenIndex = 0;
	for(PxU32 a = 0; a < 5; ++a)
	{
		if(!chosen[a])
		{
			notChosenIndex = a;
			break;
		}
	}

	FloatV closest = FMax();
	index = 0;
	for(PxI32 a = 0; a < 4; ++a)
	{
		Vec3V dif = V3Sub(mContactPoints[a].mLocalPointA, tempContacts[notChosenIndex].mLocalPointA);
		const FloatV d2 = V3Dot(dif, dif);
		if(FAllGrtr(closest, d2))
		{
			closest = d2;
			index = a;
		}
	}

	if(FAllGrtr(V4GetW(mContactPoints[index].mLocalNormalPen), V4GetW(tempContacts[notChosenIndex].mLocalNormalPen)))
	{
		//Swap
		mContactPoints[index] = tempContacts[notChosenIndex];
	}


	return 0;
}


/*
	This function is for box/convexHull vs box/convexHull.
*/
void Gu::PersistentContactManifold::addManifoldContactsToContactBuffer(Gu::ContactBuffer& contactBuffer, const Ps::aos::Vec3VArg normal, const Ps::aos::PsTransformV& transf1, const Ps::aos::FloatVArg contactOffset)
{
	using namespace Ps::aos;
	//add the manifold contacts;
	PxU32 contactCount = 0;//contactBuffer.count;
	for(PxU32 i=0; (i< mNumContacts) & (contactCount < Gu::ContactBuffer::MAX_CONTACTS); ++i)
	{
		PersistentContact& p = getContactPoint(i);
		
		const FloatV dist = V4GetW(p.mLocalNormalPen);

		//Either the newly created points or the cache points might have dist/penetration larger than contactOffset
		if(FAllGrtrOrEq(contactOffset, dist))
		{
			const Vec3V worldP =transf1.transform(p.mLocalPointB);

			Gu::ContactPoint& contact = contactBuffer.contacts[contactCount++];
			//Fast allign store
			V4StoreA(Vec4V_From_Vec3V(normal), reinterpret_cast<PxF32*>(&contact.normal.x));
			V4StoreA(Vec4V_From_Vec3V(worldP), reinterpret_cast<PxF32*>(&contact.point.x));
			FStore(dist, &contact.separation);

			PX_ASSERT(contact.point.isFinite());
			PX_ASSERT(contact.normal.isFinite());
			PX_ASSERT(PxIsFinite(contact.separation));

			contact.internalFaceIndex1 = PXC_CONTACT_NO_FACE_INDEX;
		}
	
	}

	contactBuffer.count = contactCount;
}

/*
	This function is for direct implementation for box vs box. We don't need to discard the contacts based on whether the contact offset is larger than penetration/dist because
	the direct implementation will guarantee the penetration/dist won't be larger than the contact offset. Also, we won't have points from the cache if the direct implementation
	of box vs box is called.
*/
void Gu::PersistentContactManifold::addManifoldContactsToContactBuffer(Gu::ContactBuffer& contactBuffer, const Ps::aos::Vec3VArg normal, const Ps::aos::PsMatTransformV& transf1)
{
	using namespace Ps::aos;

	//add the manifold contacts;
	PxU32 contactCount = 0;//contactBuffer.count;
	for(PxU32 i=0; (i< mNumContacts) & (contactCount < Gu::ContactBuffer::MAX_CONTACTS); ++i)
	{
	
		PersistentContact& p = getContactPoint(i);
		
		const Vec3V worldP =transf1.transform(p.mLocalPointB);
		const FloatV dist = V4GetW(p.mLocalNormalPen);

		Gu::ContactPoint& contact = contactBuffer.contacts[contactCount++];
		//Fast allign store
		V4StoreA(Vec4V_From_Vec3V(normal), reinterpret_cast<PxF32*>(&contact.normal.x));
		V4StoreA(Vec4V_From_Vec3V(worldP), reinterpret_cast<PxF32*>(&contact.point.x));
		FStore(dist, &contact.separation);
		PX_ASSERT(PxIsFinite(contact.point.x));
		PX_ASSERT(PxIsFinite(contact.point.y));
		PX_ASSERT(PxIsFinite(contact.point.z));

		//PX_ASSERT(contact.separation > -0.2f);

		contact.internalFaceIndex1 = PXC_CONTACT_NO_FACE_INDEX;
	}

	contactBuffer.count = contactCount;
}

/*
	This function is for sphere/capsule vs other primitives. We treat sphere as a point and capsule as a segment in the contact gen and store the sphere center or a point in the segment for capsule
	in the manifold. 
*/
void Gu::PersistentContactManifold::addManifoldContactsToContactBuffer(Gu::ContactBuffer& contactBuffer, const Ps::aos::Vec3VArg normal, const Ps::aos::Vec3VArg projectionNormal,
	const Ps::aos::PsTransformV& transf0, const Ps::aos::FloatVArg radius, const Ps::aos::FloatVArg contactOffset)
{
	using namespace Ps::aos;

	//add the manifold contacts;
	PxU32 contactCount = 0;//contactBuffer.count;
	for(PxU32 i=0; (i< mNumContacts) & (contactCount < Gu::ContactBuffer::MAX_CONTACTS); ++i)
	{
	
		PersistentContact& p = getContactPoint(i);
		const FloatV dist = FSub(V4GetW(p.mLocalNormalPen), radius);

		//The newly created points should have a dist < contactOffset. However, points might come from the PCM contact cache so we might still have points which are
		//larger than contactOffset. The reason why we don't want to discard the contacts in the contact cache whose dist > contactOffset is because the contacts may 
		//only temporarily become separated. The points still project onto roughly the same spot but so, if the bodies move together again, the contacts may be valid again.
		//This is important when simulating at large time-steps because GJK can only generate 1 point of contact and missing contacts for just a single frame with large time-steps
		//may introduce noticeable instability.
		if(FAllGrtrOrEq(contactOffset, dist))
		{
			const Vec3V worldP =V3NegScaleSub(projectionNormal, radius, transf0.transform(p.mLocalPointA));

			Gu::ContactPoint& contact = contactBuffer.contacts[contactCount++];
			//Fast allign store
			V4StoreA(Vec4V_From_Vec3V(normal), reinterpret_cast<PxF32*>(&contact.normal.x));
			V4StoreA(Vec4V_From_Vec3V(worldP), reinterpret_cast<PxF32*>(&contact.point.x));
			FStore(dist, &contact.separation);
			//PX_ASSERT(PxAbs(contact.separation) < 2.f);

			contact.internalFaceIndex1 = PXC_CONTACT_NO_FACE_INDEX;
		}
	}

	contactBuffer.count = contactCount;
}


/*
	This function is used in the box/convexhull full manifold contact genenation. We will pass in a list of manifold contacts. If the number of contacts are more than
	GU_MANIFOLD_CACHE_SIZE, we will need to do contact reduction while we are storing the chosen manifold contacts from the manifold contact list to the manifold contact
	buffer.
*/
void Gu::PersistentContactManifold::addBatchManifoldContacts(const PersistentContact* manifoldContacts, 
	const PxU32 numPoints, const PxReal toleranceLength)
{

	if(numPoints <= GU_MANIFOLD_CACHE_SIZE)
	{
		for(PxU32 i=0; i<numPoints; ++i)
		{
			mContactPoints[i].mLocalPointA =  manifoldContacts[i].mLocalPointA;
			mContactPoints[i].mLocalPointB =  manifoldContacts[i].mLocalPointB;
			mContactPoints[i].mLocalNormalPen =  manifoldContacts[i].mLocalNormalPen;
		}
		mNumContacts = Ps::to8(numPoints);
	}
	else
	{
		
		reduceBatchContacts(manifoldContacts, numPoints, toleranceLength);

		mNumContacts = GU_MANIFOLD_CACHE_SIZE;
	}
}  

/*
	This function is for the plane and box contact gen. If the number of points passed in is more than 4, we need to do contact reduction 
*/
void Gu::PersistentContactManifold::addBatchManifoldContactsCluster(const PersistentContact* manifoldContacts, const PxU32 numPoints)
{

	if(numPoints <= GU_MANIFOLD_CACHE_SIZE)
	{
		for(PxU32 i=0; i<numPoints; ++i)
		{
			mContactPoints[i].mLocalPointA =  manifoldContacts[i].mLocalPointA;
			mContactPoints[i].mLocalPointB =  manifoldContacts[i].mLocalPointB;
			mContactPoints[i].mLocalNormalPen =  manifoldContacts[i].mLocalNormalPen;
		}
		mNumContacts = Ps::to8(numPoints);
	}
	else
	{
		reduceBatchContactsCluster(manifoldContacts, numPoints);
		mNumContacts = GU_MANIFOLD_CACHE_SIZE;
	}
}


/*
	This function is called by addBatchManifoldContactCluster. The logic in this funtion is:
	(1)get the furthest away point from origin and store in mContactPoints[0]
	(2)get the furthest away point from mContactPoints[0] and store in mContactPoints[1]
	(3)calculate the min and max distance point away the segment (mContactPoints[0] and mContactPoints[1]) and store the max distance point to mContactPoints[2]
	(4)if the min and max distance on the same side of the segment, we need to chose the min distance point again and store this point to mContactPoints[3]; 
	(5)cluster around that 4 points and chose the deepest points
	(6)reassign contact points
*/
void Gu::PersistentContactManifold::reduceBatchContactsCluster(const PersistentContact* manifoldPoints, const PxU32 numPoints)
{
	using namespace Ps::aos;
	//get the deepest points

	bool chosen[64];
	physx::PxMemZero(chosen, sizeof(bool)*numPoints);
	const FloatV max = FMax();
	const FloatV nmax = FNeg(max);
	FloatV maxDist = nmax;
	PxU32 index = 0;

	PxU32 indices[4];
	
	//get the furthest away point from itself
	for(PxU32 i=0; i<numPoints; ++i)
	{
		const FloatV dist = V3Dot(manifoldPoints[i].mLocalPointB, manifoldPoints[i].mLocalPointB);
		if(FAllGrtr(dist, maxDist))
		{
			maxDist = dist;
			index = i;
		}
	}

	//keep the furthest points in the first position
	mContactPoints[0] =  manifoldPoints[index];
	chosen[index] = true;
	indices[0] = index;


	//calculate the furthest away points from mContactPoints[0]
	Vec3V v = V3Sub(manifoldPoints[0].mLocalPointB, mContactPoints[0].mLocalPointB);
	maxDist = V3Dot(v, v);
	index = 0;

	for(PxU32 i=1; i<numPoints; ++i)
	{
		v = V3Sub(manifoldPoints[i].mLocalPointB, mContactPoints[0].mLocalPointB);
		const FloatV d = V3Dot(v, v);
		if(FAllGrtr(d, maxDist))
		{
			maxDist = d;
			index = i;
		}
	}

	//PX_ASSERT(chosen[index] == false);
	mContactPoints[1] =  manifoldPoints[index];
	chosen[index] = true;
	indices[1] = index;


	maxDist = nmax;
	index = GU_MANIFOLD_INVALID_INDEX;
	v = V3Sub(mContactPoints[1].mLocalPointB, mContactPoints[0].mLocalPointB);

	const Vec3V cn0 = Vec3V_From_Vec4V(mContactPoints[0].mLocalNormalPen);
	Vec3V norm = V3Cross(v, cn0);
	const FloatV sqLen = V3Dot(norm, norm);
	norm = V3Sel(FIsGrtr(sqLen, FZero()), V3ScaleInv(norm, FSqrt(sqLen)), cn0);


	FloatV minDist = max;
	PxU32 index1 = GU_MANIFOLD_INVALID_INDEX;
	

	//calculate the min and max point away from the segment
	for(PxU32 i=0; i<numPoints; ++i)
	{
		if(!chosen[i])
		{
			v = V3Sub(manifoldPoints[i].mLocalPointB, mContactPoints[0].mLocalPointB);
			const FloatV d = V3Dot(v, norm);
			if(FAllGrtr(d, maxDist))
			{
				maxDist = d;
				index = i;
			}

			if(FAllGrtr(minDist, d))
			{
				minDist = d;
				index1 = i;
			}
		}
	}

	//PX_ASSERT(chosen[index] == false && chosen[index1] == false);
	mContactPoints[2] =  manifoldPoints[index];
	chosen[index] = true;
	indices[2] = index;

	//if min and max in the same side, chose again
	const FloatV temp = FMul(minDist, maxDist);
	if(FAllGrtr(temp, FZero()))
	{
		//chose again
		maxDist = nmax;
		for(PxU32 i=0; i<numPoints; ++i)
		{
			if(!chosen[i])
			{
				v = V3Sub(manifoldPoints[i].mLocalPointB, mContactPoints[0].mLocalPointB);
				const FloatV d = V3Dot(v, norm);
				if(FAllGrtr(d, maxDist))
				{
					maxDist = d;
					index1 = i;
				}
			}
		}
	}

	mContactPoints[3] = manifoldPoints[index1];
	chosen[index1] = true;
	indices[3] = index1;

	//cluster around that 4 points and chose the deepest points
	for(PxU32 i=0; i<numPoints; ++i)
	{
		if(!chosen[i])
		{
			maxDist = max;
			const FloatV pen = V4GetW(manifoldPoints[i].mLocalNormalPen);
			index = 0;
			for(PxU32 j=0; j<4; ++j)
			{
				const Vec3V v1 = V3Sub(manifoldPoints[i].mLocalPointB, mContactPoints[j].mLocalPointB);
				const FloatV dist = V3Dot(v1, v1);
				if(FAllGrtr(maxDist, dist))
				{
					maxDist = dist;
					index = j;
				}
			}

			//check to see whether the penetration is deeper than the point in mContactPoints
			const FloatV tempPen = V4GetW(manifoldPoints[indices[index]].mLocalNormalPen);
			if(FAllGrtr(tempPen, pen))
			{
				//swap the indices
				indices[index] = i;
			}
		}
	}

	mContactPoints[0] = manifoldPoints[indices[0]];
	mContactPoints[1] = manifoldPoints[indices[1]];
	mContactPoints[2] = manifoldPoints[indices[2]];
	mContactPoints[3] = manifoldPoints[indices[3]];
	
				
}

/*
	This function is for box/convexhull full contact generation. If the numPoints > 4, we will reduce the contact points to 4
*/
void Gu::PersistentContactManifold::reduceBatchContacts(const PersistentContact* manifoldPoints, 
	const PxU32 numPoints, const PxReal tolereanceLength)
{
	using namespace Ps::aos;

	PxU8 chosenIndices[4];
	PxU8 candidates[64];
	
	const FloatV zero = FZero();
	const FloatV max = FMax();
	const FloatV nmax = FNeg(max);
	FloatV maxPen = V4GetW(manifoldPoints[0].mLocalNormalPen);
	FloatV minPen = nmax;
	PxU32 index = 0;
	candidates[0] = 0;
	PxU32 candidateIndex = 0;

	PxU32 nbCandiates = numPoints;
	//keep the deepest point, candidateIndex will be the same as index
	for (PxU32 i = 1; i<numPoints; ++i)
	{
		//at the begining candidates and indices will be the same
		candidates[i] = PxU8(i);
	
		const FloatV pen = V4GetW(manifoldPoints[i].mLocalNormalPen);
		minPen = FMax(minPen, pen);
		if (FAllGrtr(maxPen, pen))
		{
			maxPen = pen;
			index = i;
			candidateIndex = i;
		}
	}

	//keep the deepest points in the first position
	chosenIndices[0] = PxU8(index);

	//move the chosen indices out of the candidates indices
	nbCandiates = nbCandiates - 1;
	candidates[candidateIndex] = candidates[nbCandiates];
	//indices[index] = nbCandiates;

	//calculate the furthest away points
	Vec3V v = V3Sub(manifoldPoints[candidates[0]].mLocalPointB, manifoldPoints[chosenIndices[0]].mLocalPointB);
	FloatV maxDist = V3Dot(v, v);
	index = candidates[0];
	candidateIndex = 0;

	for (PxU32 i = 1; i<nbCandiates; ++i)
	{
		v = V3Sub(manifoldPoints[candidates[i]].mLocalPointB, manifoldPoints[chosenIndices[0]].mLocalPointB);
		const FloatV d = V3Dot(v, v);
		if (FAllGrtr(d, maxDist))
		{
			maxDist = d;
			index = candidates[i];
			candidateIndex = i;
		}
	}

	chosenIndices[1] = PxU8(index);
	//move the chosen indices out of the candidates indices
	nbCandiates = nbCandiates - 1;
	candidates[candidateIndex] = candidates[nbCandiates];

	v = V3Sub(manifoldPoints[chosenIndices[1]].mLocalPointB, manifoldPoints[chosenIndices[0]].mLocalPointB);
	const Vec3V cn0 = Vec3V_From_Vec4V(manifoldPoints[chosenIndices[0]].mLocalNormalPen);
	Vec3V norm = V3Cross(v, cn0);
	const FloatV sqLen = V3Dot(norm, norm);
	norm = V3Sel(FIsGrtr(sqLen, zero), V3ScaleInv(norm, FSqrt(sqLen)), cn0);

	//reset maxDist and index
	maxDist = nmax;
	index = GU_MANIFOLD_INVALID_INDEX;
	candidateIndex = GU_MANIFOLD_INVALID_INDEX;
	FloatV minDist = max;
	PxU32 index1 = GU_MANIFOLD_INVALID_INDEX;
	PxU32 candidateIndex1 = GU_MANIFOLD_INVALID_INDEX;


	//calculate the min and max point away from the segment
	for (PxU32 i = 0; i<nbCandiates; ++i)
	{
		v = V3Sub(manifoldPoints[candidates[i]].mLocalPointB, manifoldPoints[chosenIndices[0]].mLocalPointB);
		const FloatV d = V3Dot(v, norm);
		if (FAllGrtr(d, maxDist))
		{
			maxDist = d;
			index = candidates[i];
			candidateIndex = i;
		}

		if (FAllGrtr(minDist, d))
		{
			minDist = d;
			index1 = candidates[i];
			candidateIndex1 = i;
		}
		
	}


	chosenIndices[2] = PxU8(index);
	//move the chosen indices out of the candidates indices
	nbCandiates = nbCandiates - 1;
	candidates[candidateIndex] = candidates[nbCandiates];
	if (nbCandiates == candidateIndex1)
		candidateIndex1 = candidateIndex;

	//if min and max in the same side, chose again
	const FloatV temp = FMul(minDist, maxDist);
	if (FAllGrtr(temp, zero))
	{
		//chose again
		maxDist = nmax;
		for (PxU32 i = 0; i < nbCandiates; ++i)
		{
			v = V3Sub(manifoldPoints[candidates[i]].mLocalPointB, manifoldPoints[chosenIndices[0]].mLocalPointB);
			const FloatV d = V3Dot(v, norm);
			if (FAllGrtr(d, maxDist))
			{
				maxDist = d;
				index1 = candidates[i];
				candidateIndex1 = i;
			}
		}
		
	}

	chosenIndices[3] = PxU8(index1);
	//move the chosen indices out of the candidates indices
	nbCandiates = nbCandiates - 1;
	candidates[candidateIndex1] = candidates[nbCandiates];

	const FloatV eps = FLoad(tolereanceLength * 0.02f);
	const BoolV con = BAnd(FIsGrtr(eps, maxPen), FIsGrtr(minPen, eps));
	if (BAllEqTTTT(con) )
	{
		//post process 
		for (PxU32 i = 0; i < 4; ++i)
		{
			FloatV pen = V4GetW(manifoldPoints[chosenIndices[i]].mLocalNormalPen);

			if (FAllGrtr(pen, eps))
			{
				candidateIndex = GU_MANIFOLD_INVALID_INDEX;
				for (PxU32 j = 0; j < nbCandiates; ++j)
				{
					const FloatV pen1 = V4GetW(manifoldPoints[candidates[j]].mLocalNormalPen);
					if (FAllGrtr(pen, pen1) && FAllGrtr(eps, pen1))
					{
						pen = pen1;
						candidateIndex = j;
					}
				}
				if (candidateIndex < nbCandiates)
				{
					const PxU8 originalIndex = chosenIndices[i];
					chosenIndices[i] = candidates[candidateIndex];
					candidates[candidateIndex] = originalIndex;
				}
			}
			mContactPoints[i] = manifoldPoints[chosenIndices[i]];
		}
	}
	else
	{
		for (PxU32 i = 0; i < 4; ++i)
		{
			mContactPoints[i] = manifoldPoints[chosenIndices[i]];
		}
	}
}
/*
	This function is for capsule full contact generation. If the numPoints > 2, we will reduce the contact points to 2
*/
void Gu::PersistentContactManifold::reduceBatchContacts2(const PersistentContact* manifoldPoints, const PxU32 numPoints)  
{
	using namespace Ps::aos;
	
	PX_ASSERT(numPoints < 64);
	bool chosen[64];
	physx::PxMemZero(chosen, sizeof(bool)*numPoints);
	FloatV maxDis = V4GetW(manifoldPoints[0].mLocalNormalPen);
	PxI32 index = 0;
	//keep the deepest point
	for(PxU32 i=1; i<numPoints; ++i)
	{
		const FloatV pen = V4GetW(manifoldPoints[i].mLocalNormalPen);
		if(FAllGrtr(maxDis, pen))
		{  
			maxDis = pen;
			index = PxI32(i);
		}
	}
	//keep the deepest points in the first position
	mContactPoints[0] =  manifoldPoints[index];
	chosen[index] = true;


	//calculate the furthest away points
	Vec3V v = V3Sub(manifoldPoints[0].mLocalPointB, mContactPoints[0].mLocalPointB);
	maxDis = V3Dot(v, v);
	index = 0;

	for(PxU32 i=1; i<numPoints; ++i)
	{
		v = V3Sub(manifoldPoints[i].mLocalPointB, mContactPoints[0].mLocalPointB);
		const FloatV d = V3Dot(v, v);
		if(FAllGrtr(d, maxDis))
		{
			maxDis = d;
			index = PxI32(i);
		}
	}

	//PX_ASSERT(chosen[index] == false);
	mContactPoints[1] =  manifoldPoints[index];
	chosen[index] = true;

	PxI32 secondIndex = index;
	FloatV maxDepth = V4GetW(manifoldPoints[index].mLocalNormalPen);
	for(PxU32 i = 0; i < numPoints; ++i)
	{
		if(!chosen[i])
		{
			Vec3V d0 = V3Sub(mContactPoints[0].mLocalPointB, manifoldPoints[i].mLocalPointB);
			Vec3V d1 = V3Sub(mContactPoints[1].mLocalPointB, manifoldPoints[i].mLocalPointB);
			const FloatV dd0 = V3Dot(d0, d0);
			const FloatV dd1 = V3Dot(d1, d1);

			if(FAllGrtr(dd0, dd1))
			{
				//This clusters to point 1
				if(FAllGrtr(maxDepth, V4GetW(manifoldPoints[i].mLocalNormalPen)))
				{
					secondIndex = PxI32(i);
				}
			}
		}
	}

	if(secondIndex != index)
	{
		mContactPoints[1] = manifoldPoints[secondIndex];
	}	
}

PxU32 Gu::PersistentContactManifold::addManifoldPoint(const Ps::aos::Vec3VArg localPointA, const Ps::aos::Vec3VArg localPointB, const Ps::aos::Vec4VArg localNormalPen
													 , const Ps::aos::FloatVArg replaceBreakingThreshold)
{
	using namespace Ps::aos;
	
	if(replaceManifoldPoint(localPointA, localPointB, localNormalPen, replaceBreakingThreshold)) //replace the new point with the old one
		return 0;

	switch(mNumContacts)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		mContactPoints[mNumContacts].mLocalPointA = localPointA;
		mContactPoints[mNumContacts].mLocalPointB = localPointB;
		mContactPoints[mNumContacts++].mLocalNormalPen = localNormalPen;

		return 1;
	default:
		return reduceContactsForPCM(localPointA, localPointB, localNormalPen);//should be always return zero
	};

}


/*
	This function is for capsule vs other primitives. If the manifold originally has contacts and we can incrementally add a point at a time, we will
	use this function to add a point to manifold. If the number of contacts inside the manifold is more than 2, we will reduce contacts to 2 points.
*/
PxU32 Gu::PersistentContactManifold::addManifoldPoint2(const Ps::aos::Vec3VArg localPointA, const Ps::aos::Vec3VArg localPointB, const Ps::aos::Vec4VArg localNormalPen
													  , const Ps::aos::FloatVArg replaceBreakingThreshold)
{
	using namespace Ps::aos;
	
	if(replaceManifoldPoint(localPointA, localPointB, localNormalPen, replaceBreakingThreshold)) //replace the new point with the old one
		return 0;

	switch(mNumContacts)
	{
	case 0:
	case 1:      
		mContactPoints[mNumContacts].mLocalPointA = localPointA;
		mContactPoints[mNumContacts].mLocalPointB = localPointB;
		mContactPoints[mNumContacts++].mLocalNormalPen = localNormalPen;
		return 1;
	case 2:
		return reduceContactSegment(localPointA, localPointB, localNormalPen);
	default:
		PX_ASSERT(0);
	};
	return 0;

}

/*
	This function is used in the capsule full manifold contact genenation. We will pass in a list of manifold contacts. If the number of contacts are more than
	2, we will need to do contact reduction while we are storing the chosen manifold contacts from the manifold contact list to the manifold contact
	buffer.
*/
void Gu::PersistentContactManifold::addBatchManifoldContacts2(const PersistentContact* manifoldContacts, const PxU32 numPoints)
{
	using namespace Ps::aos;
	
	if(numPoints <= 2)
	{
		for(PxU32 i=0; i<numPoints; ++i)
		{
			mContactPoints[i].mLocalPointA =  manifoldContacts[i].mLocalPointA;
			mContactPoints[i].mLocalPointB =  manifoldContacts[i].mLocalPointB;
			mContactPoints[i].mLocalNormalPen =  manifoldContacts[i].mLocalNormalPen;
		}

		mNumContacts = Ps::to8(numPoints);
	}
	else
	{
		reduceBatchContacts2(manifoldContacts, numPoints);
		mNumContacts = 2;
	}

}


/*
	If the patch total number of manifold contacts are less than or equal to GU_SINGLE_MANIFOLD_CACHE_SIZE, we will add the manifold contacts in the contact stream to the manifold contact buffer
	which is associated to this single persistent contact manifold. Otherwise, we will reduce the manifold contacts to GU_SINGLE_MANIFOLD_CACHE_SIZE.
*/
Ps::aos::FloatV Gu::SinglePersistentContactManifold::addBatchManifoldContactsConvex(const MeshPersistentContact* manifoldContact, const PxU32 numContactExt, PCMContactPatch& patch, const Ps::aos::FloatVArg replaceBreakingThreshold)
{
	PX_UNUSED(replaceBreakingThreshold);

	using namespace Ps::aos;
	
	if(patch.mTotalSize <= GU_SINGLE_MANIFOLD_CACHE_SIZE)
	{
		PCMContactPatch* currentPatch = &patch;

		//this is because we already add the manifold contacts into manifoldContact array
		PxU32 tempNumContacts = 0;
		while(currentPatch)
		{
			for(PxU32 j=currentPatch->mStartIndex; j<currentPatch->mEndIndex; ++j)
			{
				mContactPoints[tempNumContacts++] =  manifoldContact[j];
			}
			currentPatch = currentPatch->mNextPatch;
		} 
		mNumContacts = tempNumContacts;
		return patch.mPatchMaxPen;
	}
	else
	{
		//contact reduction
		const FloatV maxPen = reduceBatchContactsConvex(manifoldContact, numContactExt, patch);
		mNumContacts = GU_SINGLE_MANIFOLD_CACHE_SIZE;
		return maxPen;
	}
}

Ps::aos::FloatV Gu::SinglePersistentContactManifold::addBatchManifoldContactsSphere(const MeshPersistentContact* manifoldContact, const PxU32 numContactExt, PCMContactPatch& patch, const Ps::aos::FloatVArg replaceBreakingThreshold)
{
	PX_UNUSED(replaceBreakingThreshold);

	using namespace Ps::aos;

	const FloatV maxPen = reduceBatchContactsSphere(manifoldContact, numContactExt, patch);
	mNumContacts = 1;
	return maxPen;
}

Ps::aos::FloatV Gu::SinglePersistentContactManifold::addBatchManifoldContactsCapsule(const MeshPersistentContact* manifoldContact, const PxU32 numContactExt, PCMContactPatch& patch, const Ps::aos::FloatVArg replaceBreakingThreshold)
{
	PX_UNUSED(replaceBreakingThreshold);

	using namespace Ps::aos;

	if(patch.mTotalSize <=GU_CAPSULE_MANIFOLD_CACHE_SIZE)
	{
		PCMContactPatch* currentPatch = &patch;

		//this is because we already add the manifold contacts into manifoldContact array
		PxU32 tempNumContacts = 0;
		while(currentPatch)
		{
			for(PxU32 j=currentPatch->mStartIndex; j<currentPatch->mEndIndex; ++j)
			{
				mContactPoints[tempNumContacts++] = manifoldContact[j];
			}
			currentPatch = currentPatch->mNextPatch;
		} 
		mNumContacts = tempNumContacts;
		return patch.mPatchMaxPen;
	}
	else
	{
		
		const FloatV maxPen = reduceBatchContactsCapsule(manifoldContact, numContactExt, patch);
		mNumContacts = GU_CAPSULE_MANIFOLD_CACHE_SIZE;
		return maxPen;
	}

}


Ps::aos::FloatV Gu::SinglePersistentContactManifold::reduceBatchContactsSphere(const MeshPersistentContact* manifoldContactExt, const PxU32 numContacts, PCMContactPatch& patch)
{
	PX_UNUSED(numContacts);

	using namespace Ps::aos;
	FloatV max = FMax();
	FloatV maxDist = max;
	PxI32 index = -1;

	
	PCMContactPatch* currentPatch = &patch;
	while(currentPatch)
	{
		for(PxU32 i=currentPatch->mStartIndex; i<currentPatch->mEndIndex; ++i)
		{
			const FloatV pen = V4GetW(manifoldContactExt[i].mLocalNormalPen);
			if(FAllGrtr(maxDist, pen))
			{
				maxDist = pen;
				index = PxI32(i);
			}
		}
		currentPatch = currentPatch->mNextPatch;
	}

	PX_ASSERT(index!=-1);
	mContactPoints[0] = manifoldContactExt[index];

	return maxDist;

}

Ps::aos::FloatV Gu::SinglePersistentContactManifold::reduceBatchContactsCapsule(const MeshPersistentContact* manifoldContactExt, const PxU32 numContacts, PCMContactPatch& patch)
{
	using namespace Ps::aos;

	bool* chosen = reinterpret_cast<bool*>(PxAlloca(sizeof(bool)*numContacts));
	physx::PxMemZero(chosen, sizeof(bool)*numContacts);
	const FloatV max = FMax();
	FloatV maxDis = max;
	PxI32 index = -1;

	FloatV maxPen = max;


	PCMContactPatch* currentPatch = &patch;
	while(currentPatch)
	{
		for(PxU32 i=currentPatch->mStartIndex; i<currentPatch->mEndIndex; ++i)
		{
			const FloatV pen = V4GetW(manifoldContactExt[i].mLocalNormalPen);
			if(FAllGrtr(maxDis, pen))
			{
				maxDis = pen;
				index = PxI32(i);
			}
		}
		currentPatch = currentPatch->mNextPatch;
	}

	chosen[index] = true;
	mContactPoints[0] = manifoldContactExt[index];
	maxPen = FMin(maxPen, V4GetW(manifoldContactExt[index].mLocalNormalPen));


	//calculate the furthest away points
	Vec3V v = V3Sub(manifoldContactExt[patch.mStartIndex].mLocalPointB, mContactPoints[0].mLocalPointB);
	maxDis = V3Dot(v, v);
	index = PxI32(patch.mStartIndex);

	currentPatch = &patch;
	while(currentPatch)
	{
		for(PxU32 i=currentPatch->mStartIndex; i<currentPatch->mEndIndex; ++i)
		{
			v = V3Sub(manifoldContactExt[i].mLocalPointB, mContactPoints[0].mLocalPointB);
			const FloatV d = V3Dot(v, v);
			if(FAllGrtr(d, maxDis))
			{
				maxDis = d;
				index = PxI32(i);
			}
		}
		currentPatch = currentPatch->mNextPatch;
	}

	//PX_ASSERT(chosen[index] == false);
	chosen[index] = true;
	mContactPoints[1] = manifoldContactExt[index];
	maxPen = FMin(maxPen, V4GetW(manifoldContactExt[index].mLocalNormalPen));

	//keep the second deepest point
	maxDis = max;
	currentPatch = &patch;
	while(currentPatch)
	{
		for(PxU32 i=currentPatch->mStartIndex; i<currentPatch->mEndIndex; ++i)
		{
			if(!chosen[i])
			{
				const FloatV pen = V4GetW(manifoldContactExt[i].mLocalNormalPen);
				//const FloatV v = V3Dot(manifoldContactExt[i].mLocalPointB, manifoldContactExt[i].mLocalPointB);
				if(FAllGrtr(maxDis, pen))
				{
					maxDis = pen;
					index = PxI32(i);
				}
			}
		}
		currentPatch = currentPatch->mNextPatch;
	}
	//PX_ASSERT(chosen[index] == false);
	chosen[index] = true;
	mContactPoints[2] = manifoldContactExt[index];
	maxPen = FMin(maxPen, V4GetW(manifoldContactExt[index].mLocalNormalPen));

	return maxPen;
}

Ps::aos::FloatV Gu::SinglePersistentContactManifold::reduceBatchContactsConvex(const MeshPersistentContact* manifoldContactExt, const PxU32 numContacts, PCMContactPatch& patch)
{
	using namespace Ps::aos;

	bool* chosen = reinterpret_cast<bool*>(PxAlloca(sizeof(bool)*numContacts));
	physx::PxMemZero(chosen, sizeof(bool)*numContacts);
	const FloatV max = FMax();
	const FloatV nmax = FNeg(max);
	FloatV maxDis = nmax;
	PxU32 index = GU_MANIFOLD_INVALID_INDEX;

	PCMContactPatch* currentPatch = &patch;
	while(currentPatch)
	{
		for(PxU32 i=currentPatch->mStartIndex; i<currentPatch->mEndIndex; ++i)
		{
			//const FloatV pen = V4GetW(manifoldContactExt[i].localNormalPen);
			const FloatV v = V3Dot(manifoldContactExt[i].mLocalPointB, manifoldContactExt[i].mLocalPointB);
			if(FAllGrtr(v, maxDis))
			{
				maxDis = v;
				index = i;
			}
		}
		currentPatch = currentPatch->mNextPatch;
	}

	chosen[index] = true;

	const Vec3V contact0 = manifoldContactExt[index].mLocalPointB;
	mContactPoints[0] = manifoldContactExt[index];

	FloatV maxPen = V4GetW(manifoldContactExt[index].mLocalNormalPen);

	//calculate the furthest away points
	Vec3V v = V3Sub(manifoldContactExt[patch.mStartIndex].mLocalPointB, contact0);
	maxDis = V3Dot(v, v);
	index = patch.mStartIndex;

	currentPatch = &patch;
	while(currentPatch)
	{
		for(PxU32 i=currentPatch->mStartIndex; i<currentPatch->mEndIndex; ++i)
		{
			v = V3Sub(manifoldContactExt[i].mLocalPointB, contact0);
			const FloatV d = V3Dot(v, v);
			if(FAllGrtr(d, maxDis))
			{
				maxDis = d;
				index = i;
			}
		}
		currentPatch = currentPatch->mNextPatch;
	}

	//PX_ASSERT(chosen[index] == false);
	chosen[index] = true;
	const Vec3V contact1 = manifoldContactExt[index].mLocalPointB;
	mContactPoints[1] = manifoldContactExt[index];

	maxPen = FMin(maxPen, V4GetW(manifoldContactExt[index].mLocalNormalPen));

	maxDis = nmax;
	index = GU_MANIFOLD_INVALID_INDEX;
	v = V3Sub(contact1, contact0);
	const Vec3V cn0 = Vec3V_From_Vec4V(mContactPoints[0].mLocalNormalPen);
	Vec3V norm = V3Cross(v, cn0);
	const FloatV sqLen = V3Dot(norm, norm);
	norm = V3Sel(FIsGrtr(sqLen, FZero()), V3ScaleInv(norm, FSqrt(sqLen)), cn0);
	FloatV minDis = max;
	PxU32 index1 = GU_MANIFOLD_INVALID_INDEX;
	

	//calculate the point furthest way to the segment
	currentPatch = &patch;
	
	while(currentPatch)
	{
		for(PxU32 i=currentPatch->mStartIndex; i<currentPatch->mEndIndex; ++i)
		{
			if(!chosen[i])
			{
				v = V3Sub(manifoldContactExt[i].mLocalPointB, contact0);
				const FloatV d = V3Dot(v, norm);
				if(FAllGrtr(d, maxDis))
				{
					maxDis = d;
					index = i;
				}

				if(FAllGrtr(minDis, d))
				{
					minDis = d;
					index1 = i;
				}
			}
		}
		currentPatch = currentPatch->mNextPatch;
	}

	//PX_ASSERT(chosen[index] == false);

	chosen[index] = true;
	mContactPoints[2] = manifoldContactExt[index];
	maxPen = FMin(maxPen, V4GetW(manifoldContactExt[index].mLocalNormalPen));

	//if min and max in the same side, choose again
	const FloatV temp = FMul(minDis, maxDis);
	if(FAllGrtr(temp, FZero()))
	{
		//choose again
		maxDis = nmax;
		//calculate the point furthest way to the segment
		currentPatch = &patch;
		
		while(currentPatch)
		{
			for(PxU32 i=currentPatch->mStartIndex; i<currentPatch->mEndIndex; ++i)
			{
				if(!chosen[i])
				{
					v = V3Sub(manifoldContactExt[i].mLocalPointB, contact0);
					const FloatV d = V3Dot(v, norm);
					if(FAllGrtr(d, maxDis))
					{
						maxDis = d;
						index1 = i;
					}
				}
			}
			currentPatch = currentPatch->mNextPatch;
		}
	}

	//PX_ASSERT(chosen[index1] == false);

	chosen[index1] = true;
	mContactPoints[3] = manifoldContactExt[index1];
	maxPen = FMin(maxPen, V4GetW(manifoldContactExt[index1].mLocalNormalPen));

	const PxU32 NB_TO_ADD = GU_SINGLE_MANIFOLD_CACHE_SIZE - 4;

	FloatV pens[NB_TO_ADD];
	PxU32 inds[NB_TO_ADD];
	for (PxU32 a = 0; a < NB_TO_ADD; ++a)
	{
		pens[a] = FMax();
		inds[a] = 0;
	}
	{
		currentPatch = &patch;

		while (currentPatch)
		{
			for (PxU32 i = currentPatch->mStartIndex; i < currentPatch->mEndIndex; ++i)
			{
				if (!chosen[i])
				{
					const FloatV pen = V4GetW(manifoldContactExt[i].mLocalNormalPen);
					for (PxU32 a = 0; a < NB_TO_ADD; ++a)
					{
						if (FAllGrtr(pens[a], pen))
						{
							for (PxU32 b = a + 1; b < NB_TO_ADD; ++b)
							{
								pens[b] = pens[b - 1];
								inds[b] = inds[b - 1];
							}
							pens[a] = pen;
							inds[a] = i;
							break;
						}
					}
				}
			}
			currentPatch = currentPatch->mNextPatch;
		}
		for (PxU32 i = 0; i < NB_TO_ADD; ++i)
		{
			mContactPoints[i + 4] = manifoldContactExt[inds[i]];
			maxPen = FMin(maxPen, pens[i]);
		}
	}
	return maxPen;
}

PxU32 Gu::SinglePersistentContactManifold::reduceContacts(MeshPersistentContact* manifoldPoints, PxU32 numPoints)
{
	using namespace Ps::aos;

	PxU8 chosenIndices[GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE];
	PxU8* candidates = reinterpret_cast<PxU8*>(PxAlloca(sizeof(PxU8) * numPoints));

	const FloatV max = FMax();
	const FloatV nmax = FNeg(max);
	const FloatV zero = FZero();
	FloatV maxPen = V4GetW(manifoldPoints[0].mLocalNormalPen);
	PxU32 index = 0;
	candidates[0] = 0;
	PxU32 candidateIndex = 0;
	PxU32 nbCandiates = numPoints;

	MeshPersistentContact newManifold[GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE];

	//keep the deepest point
	for (PxU32 i = 1; i<numPoints; ++i)
	{
		//at the begining candidates and indices will be the same
		candidates[i] = PxU8(i);

		const FloatV pen = V4GetW(manifoldPoints[i].mLocalNormalPen);
		if (FAllGrtr(maxPen, pen))
		{
			maxPen = pen;
			index = i;
			candidateIndex = i;
		}
	}

	//keep the deepest points in the first position
	chosenIndices[0] = PxU8(index);

	//move the chosen indices out of the candidates indices
	nbCandiates = nbCandiates - 1;
	candidates[candidateIndex] = candidates[nbCandiates];

	//keep the deepest points in the first position
	newManifold[0] = manifoldPoints[chosenIndices[0]];

	//calculate the furthest away points
	Vec3V v = V3Sub(manifoldPoints[candidates[0]].mLocalPointB, newManifold[0].mLocalPointB);
	maxPen = V3Dot(v, v);
	index = candidates[0];
	candidateIndex = 0;

	for (PxU32 i = 1; i<nbCandiates; ++i)
	{
		v = V3Sub(manifoldPoints[candidates[i]].mLocalPointB, manifoldPoints[chosenIndices[0]].mLocalPointB);
		const FloatV d = V3Dot(v, v);
		if (FAllGrtr(d, maxPen))
		{
			maxPen = d;
			index = candidates[i];
			candidateIndex = i;
		}
	}


	chosenIndices[1] = PxU8(index);
	//move the chosen indices out of the candidates indices
	nbCandiates = nbCandiates - 1;
	candidates[candidateIndex] = candidates[nbCandiates];

	newManifold[1] = manifoldPoints[chosenIndices[1]];


	v = V3Sub(newManifold[1].mLocalPointB, newManifold[0].mLocalPointB);
	const Vec3V cn0 = Vec3V_From_Vec4V(newManifold[0].mLocalNormalPen);
	Vec3V norm = V3Cross(v, cn0);
	const FloatV sqLen = V3Dot(norm, norm);
	norm = V3Sel(FIsGrtr(sqLen, zero), V3ScaleInv(norm, FSqrt(sqLen)), cn0);

	FloatV maxDist = nmax;
	FloatV minDist = max;
	index = GU_MANIFOLD_INVALID_INDEX;
	PxU32 index1 = GU_MANIFOLD_INVALID_INDEX;
	PxU32 candidateIndex1 = GU_MANIFOLD_INVALID_INDEX;

	//calculate the point furthest way to the segment
	for (PxU32 i = 0; i<nbCandiates; ++i)
	{
		v = V3Sub(manifoldPoints[candidates[i]].mLocalPointB, newManifold[0].mLocalPointB);
		const FloatV d = V3Dot(v, norm);
		if (FAllGrtr(d, maxDist))
		{
			maxDist = d;
			index = candidates[i];
			candidateIndex = i;
		}

		if (FAllGrtr(minDist, d))
		{
			minDist = d;
			index1 = candidates[i];
			candidateIndex1 = i;
		}

	}

	chosenIndices[2] = PxU8(index);
	//move the chosen indices out of the candidates indices
	nbCandiates = nbCandiates - 1;
	candidates[candidateIndex] = candidates[nbCandiates];

	newManifold[2] = manifoldPoints[chosenIndices[2]];

	if (nbCandiates == candidateIndex1)
		candidateIndex1 = candidateIndex;

	const FloatV temp = FMul(minDist, maxDist);
	if (FAllGrtr(temp, FZero()))
	{
		//chose again
		maxDist = nmax;
		for (PxU32 i = 0; i < nbCandiates; ++i)
		{
			v = V3Sub(manifoldPoints[candidates[i]].mLocalPointB, newManifold[0].mLocalPointB);
			const FloatV d = V3Dot(v, norm);
			if (FAllGrtr(d, maxDist))
			{
				maxDist = d;
				index1 = candidates[i];
				candidateIndex1 = i;
			}
		}
	}

	chosenIndices[3] = PxU8(index1);
	//move the chosen indices out of the candidates indices
	nbCandiates = nbCandiates - 1;
	candidates[candidateIndex1] = candidates[nbCandiates];

	newManifold[3] = manifoldPoints[chosenIndices[3]];

	maxDist = max;
	index = GU_MANIFOLD_INVALID_INDEX;
	candidateIndex = GU_MANIFOLD_INVALID_INDEX;
	//choose the 5 point, second deepest in the left overlap point
	for (PxU32 i = 0; i < nbCandiates; ++i)
	{
		const FloatV pen = V4GetW(manifoldPoints[candidates[i]].mLocalNormalPen);
		if (FAllGrtr(maxDist, pen))
		{
			maxDist = pen;
			index = candidates[i];
			candidateIndex = i;
		}

	}

	chosenIndices[4] = PxU8(index);
	//move the chosen indices out of the candidates indices
	nbCandiates = nbCandiates - 1;
	candidates[candidateIndex] = candidates[nbCandiates];

	newManifold[4] = manifoldPoints[chosenIndices[4]];


	//copy the new manifold back
	for (PxU32 i = 0; i<GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE; ++i)
	{
		manifoldPoints[i] = newManifold[i];
	}

	return GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE;
}


Ps::aos::FloatV Gu::SinglePersistentContactManifold::refreshContactPoints(const Ps::aos::PsMatTransformV& aToB, const Ps::aos::FloatVArg projectBreakingThreshold, const Ps::aos::FloatVArg /*contactOffset*/)
{
	using namespace Ps::aos;
	const FloatV sqProjectBreakingThreshold =  FMul(projectBreakingThreshold, projectBreakingThreshold); 

	FloatV maxPen = FZero();
	// first refresh worldspace positions and distance
	for (PxU32 i=mNumContacts; i > 0; --i)
	{
		MeshPersistentContact& manifoldPoint = mContactPoints[i-1];
		const Vec3V localAInB = aToB.transform( manifoldPoint.mLocalPointA ); // from a to b
		const Vec3V localBInB = manifoldPoint.mLocalPointB;
		const Vec3V v = V3Sub(localAInB, localBInB); 

		const Vec3V localNormal = Vec3V_From_Vec4V(manifoldPoint.mLocalNormalPen); // normal in b space
		const FloatV dist= V3Dot(v, localNormal);

		const Vec3V projectedPoint = V3NegScaleSub(localNormal,  dist, localAInB);//manifoldPoint.worldPointA - manifoldPoint.worldPointB * manifoldPoint.m_distance1;
		const Vec3V projectedDifference = V3Sub(localBInB, projectedPoint);

		const FloatV distance2d = V3Dot(projectedDifference, projectedDifference);
		//const BoolV con = BOr(FIsGrtr(dist, contactOffset), FIsGrtr(distance2d, sqProjectBreakingThreshold));
		const BoolV con = FIsGrtr(distance2d, sqProjectBreakingThreshold);
		if(BAllEqTTTT(con))
		{
			removeContactPoint(i-1);
		} 
		else
		{
			manifoldPoint.mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(localNormal), dist);
			maxPen = FMin(maxPen, dist);
		}
	}

	return maxPen;
}


void Gu::SinglePersistentContactManifold::drawManifold( Cm::RenderOutput& out, const Ps::aos::PsTransformV& trA, const Ps::aos::PsTransformV& trB)
{
	using namespace Ps::aos;
#if VISUALIZE_PERSISTENT_CONTACT

	PxVec3 a, b;
	V3StoreU(trA.p, a);
	V3StoreU(trB.p, b);

	for(PxU32 i = 0; i< mNumContacts; ++i)
	{
		Gu::MeshPersistentContact& m = mContactPoints[i];
		drawManifoldPoint(m, trA, trB, out, gColors[i]);
	}
#else
	PX_UNUSED(out);
	PX_UNUSED(trA);
	PX_UNUSED(trB);
#endif
}

void Gu::MultiplePersistentContactManifold::drawManifold( Cm::RenderOutput& out, const Ps::aos::PsTransformV& trA, const Ps::aos::PsTransformV& trB)
{
	for(PxU32 i=0; i<mNumManifolds; ++i)
	{
		SinglePersistentContactManifold* manifold = getManifold(i);
		manifold->drawManifold(out, trA, trB);
	}
}

void Gu::MultiplePersistentContactManifold::drawLine(Cm::RenderOutput& out, const Ps::aos::Vec3VArg p0, const Ps::aos::Vec3VArg p1, const PxU32 color)
{
	using namespace Ps::aos;
#if VISUALIZE_PERSISTENT_CONTACT
	PxVec3 a, b;
	V3StoreU(p0, a);
	V3StoreU(p1, b);

	PxMat44 m = PxMat44(PxIdentity);
	out << color << m << Cm::RenderOutput::LINES << a << b;
#else
	PX_UNUSED(out);
	PX_UNUSED(p0);
	PX_UNUSED(p1);
	PX_UNUSED(color);

#endif
}

void Gu::MultiplePersistentContactManifold::drawLine(Cm::RenderOutput& out, const PxVec3 p0, const PxVec3 p1, const PxU32 color)
{
	using namespace Ps::aos;
#if VISUALIZE_PERSISTENT_CONTACT

	PxMat44 m = PxMat44(PxIdentity);
	out << color << m << Cm::RenderOutput::LINES << p0 << p1;
#else
	PX_UNUSED(out);
	PX_UNUSED(p0);
	PX_UNUSED(p1);
	PX_UNUSED(color);
#endif
}

void Gu::MultiplePersistentContactManifold::drawPoint(Cm::RenderOutput& out, const Ps::aos::Vec3VArg p, const PxF32 size, const PxU32 color)
{
	using namespace Ps::aos;
#if VISUALIZE_PERSISTENT_CONTACT
	const PxVec3 up(0.f, size, 0.f);
	const PxVec3 right(size, 0.f, 0.f);
	const PxVec3 forwards(0.f, 0.f, size);

	PxVec3 a;
	V3StoreU(p, a);

	PxMat44 m = PxMat44(PxIdentity);
	
	out << color << m << Cm::RenderOutput::LINES << a + up << a - up;
	out << color << m << Cm::RenderOutput::LINES << a + right << a - right;
	out << color << m << Cm::RenderOutput::LINES << a + forwards << a - forwards;
#else
	PX_UNUSED(out);
	PX_UNUSED(p);
	PX_UNUSED(size);
	PX_UNUSED(color);
#endif
}


void Gu::MultiplePersistentContactManifold::drawPolygon( Cm::RenderOutput& out, const Ps::aos::PsTransformV& transform,  Ps::aos::Vec3V* points, const PxU32 numVerts, const PxU32 color)
{
	using namespace Ps::aos;
#if VISUALIZE_PERSISTENT_CONTACT
	for(PxU32 i=0; i<numVerts; ++i)
	{
		Vec3V tempV0 = points[i == 0 ? numVerts-1 : i-1];
		Vec3V tempV1 = points[i];
		
		drawLine(out, transform.transform(tempV0), transform.transform(tempV1), color);
	}
#else
	PX_UNUSED(out);
	PX_UNUSED(transform);
	PX_UNUSED(points);
	PX_UNUSED(numVerts);
	PX_UNUSED(color);
#endif

}

static Ps::aos::FloatV addBatchManifoldContactsToSingleManifold(Gu::SinglePersistentContactManifold* manifold, Gu::MeshPersistentContact* manifoldContact, PxU32 numManifoldContacts, Gu::PCMContactPatch* patch,  const Ps::aos::FloatVArg sqReplaceBreakingThreshold, PxU8 maxContactsPerManifold)
{
	using namespace Ps::aos;
	switch(maxContactsPerManifold)
	{
	case GU_SPHERE_MANIFOLD_CACHE_SIZE://sphere
		return manifold->addBatchManifoldContactsSphere(manifoldContact, numManifoldContacts, *patch,  sqReplaceBreakingThreshold);
	case GU_CAPSULE_MANIFOLD_CACHE_SIZE://capsule, need to implement keep two deepest
		return manifold->addBatchManifoldContactsCapsule(manifoldContact, numManifoldContacts, *patch, sqReplaceBreakingThreshold);
	default://cache size GU_SINGLE_MANIFOLD_CACHE_SIZE
		return manifold->addBatchManifoldContactsConvex(manifoldContact, numManifoldContacts, *patch, sqReplaceBreakingThreshold);
	};
}

/*
	This function adds the manifold contacts with different patches into the corresponding single persistent contact manifold
*/
void Gu::MultiplePersistentContactManifold::addManifoldContactPoints(MeshPersistentContact* manifoldContact, PxU32 numManifoldContacts, PCMContactPatch** contactPatch, const PxU32 numPatch, const Ps::aos::FloatVArg sqReplaceBreakingThreshold, const Ps::aos::FloatVArg acceptanceEpsilon, PxU8 maxContactsPerManifold)
{
	using namespace Ps::aos;
	
	if(mNumManifolds == 0)
	{
		for(PxU32 i=0; i<numPatch; ++i)
		{
			PCMContactPatch* patch = contactPatch[i];
			//this mean the patch hasn't been add to the manifold
			if(patch->mRoot == patch)
			{
				
				SinglePersistentContactManifold* manifold = getEmptyManifold();
				if(manifold)
				{
					const FloatV _maxPen = addBatchManifoldContactsToSingleManifold(manifold, manifoldContact, numManifoldContacts, patch, sqReplaceBreakingThreshold, maxContactsPerManifold);
					FStore(_maxPen, &mMaxPen[mManifoldIndices[mNumManifolds]]);
					mNumManifolds++;
				}
				else
				{
					//We already pre-sorted the patches so we know we can return here
					return;
				}
				
			}
		}
	
		
	}
	else
	{
		//we do processContacts() when the number of contacts are more than 16, such that, we might call processContacts() multiple times when we have very detailed mesh
		//or very large contact offset/objects. In this case, the mNumManifolds will be large than 0. 
		PCMContactPatch tempPatch;
		for(PxU32 i=0; i<numPatch; ++i)
		{
			bool found = false;
			PCMContactPatch* patch = contactPatch[i];
			//this mean the patch has't been add to the manifold
			if(patch->mRoot == patch)
			{
				PX_ASSERT(mNumManifolds <= GU_MAX_MANIFOLD_SIZE);
				for(PxU32 j=0; j<mNumManifolds; ++j)
				{
					PX_ASSERT(mManifoldIndices[j] < GU_MAX_MANIFOLD_SIZE);
					SinglePersistentContactManifold& manifold = *getManifold(j);

					const Vec3V pNor = manifold.getLocalNormal();
					const FloatV d = V3Dot(patch->mPatchNormal, pNor);

					if(FAllGrtrOrEq(d, acceptanceEpsilon))
					{
						//appending the existing contacts to the manifold contact stream
						for(PxU32 k=0; k< manifold.mNumContacts; ++k)
						{
							PxU32 index = k + numManifoldContacts;
							//not duplicate point
							PX_ASSERT(index < 64);
							manifoldContact[index] = manifold.mContactPoints[k];
						}

						//create a new patch for the exiting manifold
						tempPatch.mStartIndex = numManifoldContacts;
						tempPatch.mEndIndex = numManifoldContacts + manifold.mNumContacts;
						tempPatch.mPatchNormal = pNor;//manifold.getLocalNormal();
						tempPatch.mRoot = patch;
						tempPatch.mNextPatch = NULL;
						
						patch->mEndPatch->mNextPatch = &tempPatch;
				
						//numManifoldContacts += manifold.numContacts;
						patch->mTotalSize += manifold.mNumContacts;
						patch->mPatchMaxPen = FMin(patch->mPatchMaxPen, FLoad(mMaxPen[mManifoldIndices[j]]));
					
						//manifold.numContacts = 0;

						PX_ASSERT((numManifoldContacts+manifold.mNumContacts) <= 64);
						const FloatV _maxPen = addBatchManifoldContactsToSingleManifold(&manifold, manifoldContact, numManifoldContacts+manifold.mNumContacts, patch, sqReplaceBreakingThreshold, maxContactsPerManifold);
						FStore(_maxPen, &mMaxPen[mManifoldIndices[j]]);
						found = true;
						break;
					}
				}

				if(!found)// && numManifolds < 4)
				{
					SinglePersistentContactManifold* manifold = getEmptyManifold();
					//we still have slot to create a new manifold
					if(manifold)
					{
						const FloatV _maxPen = addBatchManifoldContactsToSingleManifold(manifold, manifoldContact, numManifoldContacts, patch, sqReplaceBreakingThreshold, maxContactsPerManifold);
						FStore(_maxPen, &mMaxPen[mManifoldIndices[mNumManifolds]]);
						mNumManifolds++;
					}	
					else
					{
						//we can't allocate a new manifold  and no existing manifold has the same normal as this patch, we need to find the shallowest penetration manifold. If this manifold is shallower than
						//the current patch, replace the manifold with the current patch
						PxU32 index = 0;
						for(PxU32 j=1; j<mNumManifolds; ++j)
						{
							//if(FAllGrtr(mMaxPen[mManifoldIndices[i]], mMaxPen[mManifoldIndices[index]]))
							if(mMaxPen[mManifoldIndices[j]] > mMaxPen[mManifoldIndices[index]])
							{
								index = j;
							}
						}

						if(FAllGrtr(FLoad(mMaxPen[mManifoldIndices[index]]), patch->mPatchMaxPen))
						{
							manifold = getManifold(index);
							manifold->mNumContacts = 0;
							const FloatV _maxPen = addBatchManifoldContactsToSingleManifold(manifold, manifoldContact, numManifoldContacts, patch, sqReplaceBreakingThreshold, maxContactsPerManifold);
							FStore(_maxPen, &mMaxPen[mManifoldIndices[index]]);

						}
						return;
					}
				}
			}
		}
	}
}

//This function adds the multi manifold contacts to the contact buffer for box/convexhull
bool Gu::MultiplePersistentContactManifold::addManifoldContactsToContactBuffer(Gu::ContactBuffer& contactBuffer, const Ps::aos::PsTransformV& meshTransform)
{
	using namespace Ps::aos;
	PxU32 contactCount = 0;//contactBuffer.count;
	PxU32 numContacts = 0;
	mNumTotalContacts = 0;
	//drawManifold(*gRenderOutPut, convexTransform, meshTransform);
	for(PxU32 i=0; i < mNumManifolds; ++i)
	{
		Gu::SinglePersistentContactManifold& manifold = *getManifold(i);
		numContacts = manifold.getNumContacts();
		PX_ASSERT(mNumTotalContacts + numContacts <= 0xFF);
		mNumTotalContacts += Ps::to8(numContacts);
		const Vec3V normal = manifold.getWorldNormal(meshTransform);
		
		for(PxU32 j=0; (j< numContacts) & (contactCount < ContactBuffer::MAX_CONTACTS); ++j)
		{
			Gu::MeshPersistentContact& p = manifold.getContactPoint(j);
			
			const Vec3V worldP =meshTransform.transform(p.mLocalPointB);
			const FloatV dist = V4GetW(p.mLocalNormalPen);
			
			Gu::ContactPoint& contact = contactBuffer.contacts[contactCount++];
			//Fast allign store
			V4StoreA(Vec4V_From_Vec3V(normal), reinterpret_cast<PxF32*>(&contact.normal.x));
			V4StoreA(Vec4V_From_Vec3V(worldP), reinterpret_cast<PxF32*>(&contact.point.x));
			FStore(dist, &contact.separation);

			contact.internalFaceIndex1 = p.mFaceIndex;

		}
	}

	PX_ASSERT(contactCount <= 64);
	contactBuffer.count = contactCount;
	return contactCount > 0;
}

//This function adds the multi manifold contacts to the contact buffer for sphere and capsule, radius is corresponding to the sphere radius/capsule radius
bool Gu::MultiplePersistentContactManifold::addManifoldContactsToContactBuffer(Gu::ContactBuffer& contactBuffer, const Ps::aos::PsTransformV& trA, const Ps::aos::PsTransformV& trB, const Ps::aos::FloatVArg radius)
{
	using namespace Ps::aos;
	PxU32 contactCount = 0;
	PxU32 numContacts = 0;
	mNumTotalContacts = 0;

	for(PxU32 i=0; i < mNumManifolds; ++i)
	{
		//get a single manifold
		Gu::SinglePersistentContactManifold& manifold = *getManifold(i);
		numContacts = manifold.getNumContacts();
		PX_ASSERT(mNumTotalContacts + numContacts <= 0xFF);
		mNumTotalContacts += Ps::to8(numContacts);
		const Vec3V normal = manifold.getWorldNormal(trB);
		
		//iterate all the contacts in this single manifold and add contacts to the contact buffer
		//each manifold contact point store two points which are in each other's local space. In this
		//case, mLocalPointA is stored as the center of sphere/a point in the segment for capsule
		for(PxU32 j=0; (j< numContacts) & (contactCount < ContactBuffer::MAX_CONTACTS); ++j)
		{
			Gu::MeshPersistentContact& p = manifold.getContactPoint(j);
			
			const Vec3V worldP =V3NegScaleSub(normal, radius, trA.transform(p.mLocalPointA));
			const FloatV dist = FSub(V4GetW(p.mLocalNormalPen), radius);
			
			Gu::ContactPoint& contact = contactBuffer.contacts[contactCount++];
			//Fast allign store
			V4StoreA(Vec4V_From_Vec3V(normal), reinterpret_cast<PxF32*>(&contact.normal.x));
			V4StoreA(Vec4V_From_Vec3V(worldP), reinterpret_cast<PxF32*>(&contact.point.x));
			FStore(dist, &contact.separation);

			contact.internalFaceIndex1 = p.mFaceIndex;

		}
	}

	PX_ASSERT(contactCount <= 64);
	contactBuffer.count = contactCount;
	return contactCount > 0;
}


/*
	This function copies the mesh persistent contact from compress buffer(NpCacheStreamPair in the PxcNpThreadContext) to the multiple manifold
*/
// PT: function moved to cpp to go around a compiler bug on PS4
void physx::Gu::MultiplePersistentContactManifold::fromBuffer(PxU8* PX_RESTRICT buffer)
{
	using namespace Ps::aos;
	PxU32 numManifolds = 0;
	if (buffer != NULL)
	{
		PX_ASSERT(((uintptr_t(buffer)) & 0xF) == 0);
		PxU8* PX_RESTRICT buff = buffer;
		MultiPersistentManifoldHeader* PX_RESTRICT header = reinterpret_cast<MultiPersistentManifoldHeader*>(buff);
		buff += sizeof(MultiPersistentManifoldHeader);

		numManifolds = header->mNumManifolds;

		PX_ASSERT(numManifolds <= GU_MAX_MANIFOLD_SIZE);
		mRelativeTransform = header->mRelativeTransform;

		for (PxU32 a = 0; a < numManifolds; ++a)
		{
			mManifoldIndices[a] = PxU8(a);
			SingleManifoldHeader* PX_RESTRICT manHeader = reinterpret_cast<SingleManifoldHeader*>(buff);
			buff += sizeof(SingleManifoldHeader);
			PxU32 numContacts = manHeader->mNumContacts;
			PX_ASSERT(numContacts <= GU_SINGLE_MANIFOLD_CACHE_SIZE);
			SinglePersistentContactManifold& manifold = mManifolds[a];
			manifold.mNumContacts = numContacts;
			PX_ASSERT((uintptr_t(buff) & 0xf) == 0);
			CachedMeshPersistentContact* contacts = reinterpret_cast<CachedMeshPersistentContact*>(buff);
			for (PxU32 b = 0; b < manifold.mNumContacts; ++b)
			{
				manifold.mContactPoints[b].mLocalPointA = Vec3V_From_Vec4V(V4LoadA(&contacts[b].mLocalPointA.x));
				manifold.mContactPoints[b].mLocalPointB = Vec3V_From_Vec4V(V4LoadA(&contacts[b].mLocalPointB.x));
				manifold.mContactPoints[b].mLocalNormalPen = V4LoadA(&contacts[b].mLocalNormal.x);
				manifold.mContactPoints[b].mFaceIndex = contacts[b].mFaceIndex;
			}
			buff += sizeof(Gu::CachedMeshPersistentContact) * numContacts;
		}
	}
	else
	{
		mRelativeTransform.Invalidate();
	}
	mNumManifolds = PxU8(numManifolds);
	for (PxU32 a = numManifolds; a < GU_MAX_MANIFOLD_SIZE; ++a)
	{
		mManifoldIndices[a] = PxU8(a);
	}
}

void Gu::addManifoldPoint(Gu::PersistentContact* manifoldContacts, Gu::PersistentContactManifold& manifold, GjkOutput& output,
	const Ps::aos::PsMatTransformV& aToB, const Ps::aos::FloatV replaceBreakingThreshold)
{
	using namespace Ps::aos;

	const Vec3V localPointA = aToB.transformInv(output.closestA);
	const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(output.normal), output.penDep);

	//Add contact to contact stream
	manifoldContacts[0].mLocalPointA = localPointA;
	manifoldContacts[0].mLocalPointB = output.closestB;
	manifoldContacts[0].mLocalNormalPen = localNormalPen;

	//Add contact to manifold
	manifold.addManifoldPoint(localPointA, output.closestB, localNormalPen, replaceBreakingThreshold);
}
