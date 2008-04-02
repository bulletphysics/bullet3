/*
 *	OPCODE - Optimized Collision Detection
 * http://www.codercorner.com/Opcode.htm
 * 
 * Copyright (c) 2001-2008 Pierre Terdiman,  pierre@codercorner.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//! if OPC_TRITRI_EPSILON_TEST is true then we do a check (if |dv|<EPSILON then dv=0.0;) else no check is done (which is less robust, but faster)
#define LOCAL_EPSILON 0.000001f

//! sort so that a<=b
#define SORT(a,b)			\
	if(a>b)					\
	{						\
		const float c=a;	\
		a=b;				\
		b=c;				\
	}

//! Edge to edge test based on Franlin Antonio's gem: "Faster Line Segment Intersection", in Graphics Gems III, pp. 199-202
#define EDGE_EDGE_TEST(V0, U0, U1)						\
	Bx = U0[i0] - U1[i0];								\
	By = U0[i1] - U1[i1];								\
	Cx = V0[i0] - U0[i0];								\
	Cy = V0[i1] - U0[i1];								\
	f  = Ay*Bx - Ax*By;									\
	d  = By*Cx - Bx*Cy;									\
	if((f>0.0f && d>=0.0f && d<=f) || (f<0.0f && d<=0.0f && d>=f))	\
	{													\
		const float e=Ax*Cy - Ay*Cx;					\
		if(f>0.0f)										\
		{												\
			if(e>=0.0f && e<=f) return TRUE;			\
		}												\
		else											\
		{												\
			if(e<=0.0f && e>=f) return TRUE;			\
		}												\
	}

//! TO BE DOCUMENTED
#define EDGE_AGAINST_TRI_EDGES(V0, V1, U0, U1, U2)		\
{														\
	float Bx,By,Cx,Cy,d,f;								\
	const float Ax = V1[i0] - V0[i0];					\
	const float Ay = V1[i1] - V0[i1];					\
	/* test edge U0,U1 against V0,V1 */					\
	EDGE_EDGE_TEST(V0, U0, U1);							\
	/* test edge U1,U2 against V0,V1 */					\
	EDGE_EDGE_TEST(V0, U1, U2);							\
	/* test edge U2,U1 against V0,V1 */					\
	EDGE_EDGE_TEST(V0, U2, U0);							\
}

//! TO BE DOCUMENTED
#define POINT_IN_TRI(V0, U0, U1, U2)					\
{														\
	/* is T1 completly inside T2? */					\
	/* check if V0 is inside tri(U0,U1,U2) */			\
	float a  = U1[i1] - U0[i1];							\
	float b  = -(U1[i0] - U0[i0]);						\
	float c  = -a*U0[i0] - b*U0[i1];					\
	float d0 = a*V0[i0] + b*V0[i1] + c;					\
														\
	a  = U2[i1] - U1[i1];								\
	b  = -(U2[i0] - U1[i0]);							\
	c  = -a*U1[i0] - b*U1[i1];							\
	const float d1 = a*V0[i0] + b*V0[i1] + c;			\
														\
	a  = U0[i1] - U2[i1];								\
	b  = -(U0[i0] - U2[i0]);							\
	c  = -a*U2[i0] - b*U2[i1];							\
	const float d2 = a*V0[i0] + b*V0[i1] + c;			\
	if(d0*d1>0.0f)										\
	{													\
		if(d0*d2>0.0f) return TRUE;						\
	}													\
}

//! TO BE DOCUMENTED
BOOL CoplanarTriTri(const Point& n, const Point& v0, const Point& v1, const Point& v2, const Point& u0, const Point& u1, const Point& u2)
{
	float A[3];
	short i0,i1;
	/* first project onto an axis-aligned plane, that maximizes the area */
	/* of the triangles, compute indices: i0,i1. */
	A[0] = fabsf(n[0]);
	A[1] = fabsf(n[1]);
	A[2] = fabsf(n[2]);
	if(A[0]>A[1])
	{
		if(A[0]>A[2])
		{
			i0=1;      /* A[0] is greatest */
			i1=2;
		}
		else
		{
			i0=0;      /* A[2] is greatest */
			i1=1;
		}
	}
	else   /* A[0]<=A[1] */
	{
		if(A[2]>A[1])
		{
			i0=0;      /* A[2] is greatest */
			i1=1;
		}
		else
		{
			i0=0;      /* A[1] is greatest */
			i1=2;
		}
	}

	/* test all edges of triangle 1 against the edges of triangle 2 */
	EDGE_AGAINST_TRI_EDGES(v0, v1, u0, u1, u2);
	EDGE_AGAINST_TRI_EDGES(v1, v2, u0, u1, u2);
	EDGE_AGAINST_TRI_EDGES(v2, v0, u0, u1, u2);

	/* finally, test if tri1 is totally contained in tri2 or vice versa */
	POINT_IN_TRI(v0, u0, u1, u2);
	POINT_IN_TRI(u0, v0, v1, v2);

	return FALSE;
}

//! TO BE DOCUMENTED
#define NEWCOMPUTE_INTERVALS(VV0, VV1, VV2, D0, D1, D2, D0D1, D0D2, A, B, C, X0, X1)	\
{																						\
	if(D0D1>0.0f)																		\
	{																					\
		/* here we know that D0D2<=0.0 */												\
		/* that is D0, D1 are on the same side, D2 on the other or on the plane */		\
		A=VV2; B=(VV0 - VV2)*D2; C=(VV1 - VV2)*D2; X0=D2 - D0; X1=D2 - D1;				\
	}																					\
	else if(D0D2>0.0f)																	\
	{																					\
		/* here we know that d0d1<=0.0 */												\
		A=VV1; B=(VV0 - VV1)*D1; C=(VV2 - VV1)*D1; X0=D1 - D0; X1=D1 - D2;				\
	}																					\
	else if(D1*D2>0.0f || D0!=0.0f)														\
	{																					\
		/* here we know that d0d1<=0.0 or that D0!=0.0 */								\
		A=VV0; B=(VV1 - VV0)*D0; C=(VV2 - VV0)*D0; X0=D0 - D1; X1=D0 - D2;				\
	}																					\
	else if(D1!=0.0f)																	\
	{																					\
		A=VV1; B=(VV0 - VV1)*D1; C=(VV2 - VV1)*D1; X0=D1 - D0; X1=D1 - D2;				\
	}																					\
	else if(D2!=0.0f)																	\
	{																					\
		A=VV2; B=(VV0 - VV2)*D2; C=(VV1 - VV2)*D2; X0=D2 - D0; X1=D2 - D1;				\
	}																					\
	else																				\
	{																					\
		/* triangles are coplanar */													\
		return CoplanarTriTri(N1, V0, V1, V2, U0, U1, U2);								\
	}																					\
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Triangle/triangle intersection test routine,
 *	by Tomas Moller, 1997.
 *	See article "A Fast Triangle-Triangle Intersection Test",
 *	Journal of Graphics Tools, 2(2), 1997
 *
 *	Updated June 1999: removed the divisions -- a little faster now!
 *	Updated October 1999: added {} to CROSS and SUB macros 
 *
 *	int NoDivTriTriIsect(float V0[3],float V1[3],float V2[3],
 *                      float U0[3],float U1[3],float U2[3])
 *
 *	\param		V0		[in] triangle 0, vertex 0
 *	\param		V1		[in] triangle 0, vertex 1
 *	\param		V2		[in] triangle 0, vertex 2
 *	\param		U0		[in] triangle 1, vertex 0
 *	\param		U1		[in] triangle 1, vertex 1
 *	\param		U2		[in] triangle 1, vertex 2
 *	\return		true if triangles overlap
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline_ BOOL AABBTreeCollider::TriTriOverlap(const Point& V0, const Point& V1, const Point& V2, const Point& U0, const Point& U1, const Point& U2)
{
	// Stats
	mNbPrimPrimTests++;

	// Compute plane equation of triangle(V0,V1,V2)
	Point E1 = V1 - V0;
	Point E2 = V2 - V0;
	const Point N1 = E1 ^ E2;
	const float d1 =-N1 | V0;
	// Plane equation 1: N1.X+d1=0

	// Put U0,U1,U2 into plane equation 1 to compute signed distances to the plane
	float du0 = (N1|U0) + d1;
	float du1 = (N1|U1) + d1;
	float du2 = (N1|U2) + d1;

	// Coplanarity robustness check
#ifdef OPC_TRITRI_EPSILON_TEST
	if(fabsf(du0)<LOCAL_EPSILON) du0 = 0.0f;
	if(fabsf(du1)<LOCAL_EPSILON) du1 = 0.0f;
	if(fabsf(du2)<LOCAL_EPSILON) du2 = 0.0f;
#endif
	const float du0du1 = du0 * du1;
	const float du0du2 = du0 * du2;

	if(du0du1>0.0f && du0du2>0.0f)	// same sign on all of them + not equal 0 ?
		return FALSE;				// no intersection occurs

	// Compute plane of triangle (U0,U1,U2)
	E1 = U1 - U0;
	E2 = U2 - U0;
	const Point N2 = E1 ^ E2;
	const float d2=-N2 | U0;
	// plane equation 2: N2.X+d2=0

	// put V0,V1,V2 into plane equation 2
	float dv0 = (N2|V0) + d2;
	float dv1 = (N2|V1) + d2;
	float dv2 = (N2|V2) + d2;

#ifdef OPC_TRITRI_EPSILON_TEST
	if(fabsf(dv0)<LOCAL_EPSILON) dv0 = 0.0f;
	if(fabsf(dv1)<LOCAL_EPSILON) dv1 = 0.0f;
	if(fabsf(dv2)<LOCAL_EPSILON) dv2 = 0.0f;
#endif

	const float dv0dv1 = dv0 * dv1;
	const float dv0dv2 = dv0 * dv2;

	if(dv0dv1>0.0f && dv0dv2>0.0f)	// same sign on all of them + not equal 0 ?
		return FALSE;				// no intersection occurs

	// Compute direction of intersection line
	const Point D = N1^N2;

	// Compute and index to the largest component of D
	float max=fabsf(D[0]);
	short index=0;
	float bb=fabsf(D[1]);
	float cc=fabsf(D[2]);
	if(bb>max) max=bb,index=1;
	if(cc>max) max=cc,index=2;

	// This is the simplified projection onto L
	const float vp0 = V0[index];
	const float vp1 = V1[index];
	const float vp2 = V2[index];

	const float up0 = U0[index];
	const float up1 = U1[index];
	const float up2 = U2[index];

	// Compute interval for triangle 1
	float a,b,c,x0,x1;
	NEWCOMPUTE_INTERVALS(vp0,vp1,vp2,dv0,dv1,dv2,dv0dv1,dv0dv2,a,b,c,x0,x1);

	// Compute interval for triangle 2
	float d,e,f,y0,y1;
	NEWCOMPUTE_INTERVALS(up0,up1,up2,du0,du1,du2,du0du1,du0du2,d,e,f,y0,y1);

	const float xx=x0*x1;
	const float yy=y0*y1;
	const float xxyy=xx*yy;

	float isect1[2], isect2[2];

	float tmp=a*xxyy;
	isect1[0]=tmp+b*x1*yy;
	isect1[1]=tmp+c*x0*yy;

	tmp=d*xxyy;
	isect2[0]=tmp+e*xx*y1;
	isect2[1]=tmp+f*xx*y0;

	SORT(isect1[0],isect1[1]);
	SORT(isect2[0],isect2[1]);

	if(isect1[1]<isect2[0] || isect2[1]<isect1[0]) return FALSE;
	return TRUE;
}
