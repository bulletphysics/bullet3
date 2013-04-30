/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2011 Advanced Micro Devices, Inc.  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


///This file was written by Erwin Coumans
///Separating axis rest based on work from Pierre Terdiman, see
///And contact clipping based on work from Simon Hobbs

//#define B3_DEBUG_SAT_FACE

#include "b3ConvexHullContact.h"
#include <string.h>//memcpy
#include "b3ConvexPolyhedronCL.h"


typedef b3AlignedObjectArray<b3Vector3> b3VertexArray;
#include "Bullet3Common/b3Quickprof.h"

#include <float.h> //for FLT_MAX
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"
//#include "AdlQuaternion.h"

#include "kernels/satKernels.h"
#include "kernels/satClipHullContacts.h"
#include "kernels/bvhTraversal.h"
#include "kernels/primitiveContacts.h"


#include "Bullet3Geometry/b3AabbUtil.h"

#define BT_NARROWPHASE_SAT_PATH "src/Bullet3OpenCL/NarrowphaseCollision/kernels/sat.cl"
#define BT_NARROWPHASE_CLIPHULL_PATH "src/Bullet3OpenCL/NarrowphaseCollision/kernels/satClipHullContacts.cl"
#define BT_NARROWPHASE_BVH_TRAVERSAL_PATH "src/Bullet3OpenCL/NarrowphaseCollision/kernels/bvhTraversal.cl"
#define BT_NARROWPHASE_PRIMITIVE_CONTACT_PATH "src/Bullet3OpenCL/NarrowphaseCollision/kernels/primitiveContacts.cl"


#define dot3F4 b3Dot

GpuSatCollision::GpuSatCollision(cl_context ctx,cl_device_id device, cl_command_queue  q )
:m_context(ctx),
m_device(device),
m_queue(q),
m_findSeparatingAxisKernel(0),
m_totalContactsOut(m_context, m_queue)
{
	m_totalContactsOut.push_back(0);
	
	cl_int errNum=0;

	if (1)
	{
		const char* src = satKernelsCL;

		char flags[1024]={0};
//#ifdef CL_PLATFORM_INTEL
//		sprintf(flags,"-g -s \"%s\"","C:/develop/bullet3_experiments2/opencl/gpu_narrowphase/kernels/sat.cl");
//#endif

		cl_program satProg = b3OpenCLUtils::compileCLProgramFromString(m_context,m_device,src,&errNum,flags,BT_NARROWPHASE_SAT_PATH);
		b3Assert(errNum==CL_SUCCESS);

		m_findSeparatingAxisKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,src, "findSeparatingAxisKernel",&errNum,satProg );
		b3Assert(m_findSeparatingAxisKernel);
		b3Assert(errNum==CL_SUCCESS);

		m_findConcaveSeparatingAxisKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,src, "findConcaveSeparatingAxisKernel",&errNum,satProg );
		b3Assert(m_findConcaveSeparatingAxisKernel);
		b3Assert(errNum==CL_SUCCESS);
		
		m_findCompoundPairsKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,src, "findCompoundPairsKernel",&errNum,satProg );
		b3Assert(m_findCompoundPairsKernel);
		b3Assert(errNum==CL_SUCCESS);
		m_processCompoundPairsKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,src, "processCompoundPairsKernel",&errNum,satProg );
		b3Assert(m_processCompoundPairsKernel);
		b3Assert(errNum==CL_SUCCESS);
	}

	if (1)
	{
		const char* srcClip = satClipKernelsCL;

		char flags[1024]={0};
//#ifdef CL_PLATFORM_INTEL
//		sprintf(flags,"-g -s \"%s\"","C:/develop/bullet3_experiments2/opencl/gpu_narrowphase/kernels/satClipHullContacts.cl");
//#endif

		cl_program satClipContactsProg = b3OpenCLUtils::compileCLProgramFromString(m_context,m_device,srcClip,&errNum,flags,BT_NARROWPHASE_CLIPHULL_PATH);
		b3Assert(errNum==CL_SUCCESS);

		m_clipHullHullKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,srcClip, "clipHullHullKernel",&errNum,satClipContactsProg);
		b3Assert(errNum==CL_SUCCESS);

		m_clipCompoundsHullHullKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,srcClip, "clipCompoundsHullHullKernel",&errNum,satClipContactsProg);
		b3Assert(errNum==CL_SUCCESS);
		

        m_findClippingFacesKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,srcClip, "findClippingFacesKernel",&errNum,satClipContactsProg);
		b3Assert(errNum==CL_SUCCESS);

        m_clipFacesAndContactReductionKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,srcClip, "clipFacesAndContactReductionKernel",&errNum,satClipContactsProg);
		b3Assert(errNum==CL_SUCCESS);        

		m_clipHullHullConcaveConvexKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,srcClip, "clipHullHullConcaveConvexKernel",&errNum,satClipContactsProg);
		b3Assert(errNum==CL_SUCCESS);

		m_extractManifoldAndAddContactKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,srcClip, "extractManifoldAndAddContactKernel",&errNum,satClipContactsProg);
		b3Assert(errNum==CL_SUCCESS);

        m_newContactReductionKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,srcClip,
                            "newContactReductionKernel",&errNum,satClipContactsProg);
		b3Assert(errNum==CL_SUCCESS);
	}
   else
	{
		m_clipHullHullKernel=0;
		m_clipCompoundsHullHullKernel = 0;
        m_findClippingFacesKernel = 0;
        m_newContactReductionKernel=0;
        m_clipFacesAndContactReductionKernel = 0;
		m_clipHullHullConcaveConvexKernel = 0;
		m_extractManifoldAndAddContactKernel = 0;
	}

	 if (1)
	{
		const char* srcBvh = bvhTraversalKernelCL;
		cl_program bvhTraversalProg = b3OpenCLUtils::compileCLProgramFromString(m_context,m_device,srcBvh,&errNum,"",BT_NARROWPHASE_BVH_TRAVERSAL_PATH);
		b3Assert(errNum==CL_SUCCESS);

		m_bvhTraversalKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,srcBvh, "bvhTraversalKernel",&errNum,bvhTraversalProg,"");
		b3Assert(errNum==CL_SUCCESS);

	}
        
	 {
		 const char* primitiveContactsSrc = primitiveContactsKernelsCL;
		cl_program primitiveContactsProg = b3OpenCLUtils::compileCLProgramFromString(m_context,m_device,primitiveContactsSrc,&errNum,"",BT_NARROWPHASE_PRIMITIVE_CONTACT_PATH);
		b3Assert(errNum==CL_SUCCESS);

		m_primitiveContactsKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,primitiveContactsSrc, "primitiveContactsKernel",&errNum,primitiveContactsProg,"");
		b3Assert(errNum==CL_SUCCESS);

		m_findConcaveSphereContactsKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,primitiveContactsSrc, "findConcaveSphereContactsKernel",&errNum,primitiveContactsProg );
		b3Assert(errNum==CL_SUCCESS);
		b3Assert(m_findConcaveSphereContactsKernel);

		m_processCompoundPairsPrimitivesKernel = b3OpenCLUtils::compileCLKernelFromString(m_context, m_device,primitiveContactsSrc, "processCompoundPairsPrimitivesKernel",&errNum,primitiveContactsProg,"");
		b3Assert(errNum==CL_SUCCESS);
		b3Assert(m_processCompoundPairsPrimitivesKernel);
		 
	 }
	

}

GpuSatCollision::~GpuSatCollision()
{
	
	if (m_findSeparatingAxisKernel)
		clReleaseKernel(m_findSeparatingAxisKernel);

	if (m_findConcaveSeparatingAxisKernel)
		clReleaseKernel(m_findConcaveSeparatingAxisKernel);

	if (m_findCompoundPairsKernel)
		clReleaseKernel(m_findCompoundPairsKernel);

	if (m_processCompoundPairsKernel)
		clReleaseKernel(m_processCompoundPairsKernel);
    
    if (m_findClippingFacesKernel)
        clReleaseKernel(m_findClippingFacesKernel);
   
    if (m_clipFacesAndContactReductionKernel)
        clReleaseKernel(m_clipFacesAndContactReductionKernel);
    if (m_newContactReductionKernel)
        clReleaseKernel(m_newContactReductionKernel);
	if (m_primitiveContactsKernel)
		clReleaseKernel(m_primitiveContactsKernel);
    
	if (m_findConcaveSphereContactsKernel)
		clReleaseKernel(m_findConcaveSphereContactsKernel);

	if (m_processCompoundPairsPrimitivesKernel)
		clReleaseKernel(m_processCompoundPairsPrimitivesKernel);

	if (m_clipHullHullKernel)
		clReleaseKernel(m_clipHullHullKernel);
	if (m_clipCompoundsHullHullKernel)
		clReleaseKernel(m_clipCompoundsHullHullKernel);

	if (m_clipHullHullConcaveConvexKernel)
		clReleaseKernel(m_clipHullHullConcaveConvexKernel);
	if (m_extractManifoldAndAddContactKernel)
		clReleaseKernel(m_extractManifoldAndAddContactKernel);

	if (m_bvhTraversalKernel)
		clReleaseKernel(m_bvhTraversalKernel);

}

struct MyTriangleCallback : public b3NodeOverlapCallback
{
	int m_bodyIndexA;
	int m_bodyIndexB;

	virtual void processNode(int subPart, int triangleIndex)
	{
		printf("bodyIndexA %d, bodyIndexB %d\n",m_bodyIndexA,m_bodyIndexB);
		printf("triangleIndex %d\n", triangleIndex);
	}
};


#define float4 b3Vector3
#define make_float4(x,y,z,w) b3Vector4(x,y,z,w)

float signedDistanceFromPointToPlane(const float4& point, const float4& planeEqn, float4* closestPointOnFace)
{
	float4 n = planeEqn;
	n[3] = 0.f;
	float dist = dot3F4(n, point) + planeEqn[3];
	*closestPointOnFace = point - dist * n;
	return dist;
}



#define cross3(a,b) (a.cross(b))
b3Vector3 transform(b3Vector3* v, const b3Vector3* pos, const b3Vector3* orn)
{
	b3Transform tr;
	tr.setIdentity();
	tr.setOrigin(*pos);
	b3Quaternion* o = (b3Quaternion*) orn;
	tr.setRotation(*o);
	b3Vector3 res = tr(*v);
	return res;
}


inline bool IsPointInPolygon(const float4& p, 
							const b3GpuFace* face,
							 const float4* baseVertex,
							const  int* convexIndices,
							float4* out)
{
    float4 a;
    float4 b;
    float4 ab;
    float4 ap;
    float4 v;

	float4 plane = make_float4(face->m_plane.x,face->m_plane.y,face->m_plane.z,0.f);
	
	if (face->m_numIndices<2)
		return false;

	
	float4 v0 = baseVertex[convexIndices[face->m_indexOffset + face->m_numIndices-1]];
	b = v0;

    for(unsigned i=0; i != face->m_numIndices; ++i)
    {
		a = b;
		float4 vi = baseVertex[convexIndices[face->m_indexOffset + i]];
		b = vi;
        ab = b-a;
        ap = p-a;
        v = cross3(ab,plane);

        if (b3Dot(ap, v) > 0.f)
        {
            float ab_m2 = b3Dot(ab, ab);
            float rt = ab_m2 != 0.f ? b3Dot(ab, ap) / ab_m2 : 0.f;
            if (rt <= 0.f)
            {
                *out = a;
            }
            else if (rt >= 1.f) 
            {
                *out = b;
            }
            else
            {
            	float s = 1.f - rt;
				out[0].x = s * a.x + rt * b.x;
				out[0].y = s * a.y + rt * b.y;
				out[0].z = s * a.z + rt * b.z;
            }
            return false;
        }
    }
    return true;
}

#define normalize3(a) (a.normalize())


int extractManifoldSequentialGlobal( const float4* p, int nPoints, const float4& nearNormal, b3Int4* contactIdx)
{
	if( nPoints == 0 )
        return 0;
    
    if (nPoints <=4)
        return nPoints;
    
    
    if (nPoints >64)
        nPoints = 64;
    
	float4 center = make_float4(0,0,0,0);
	{
		
		for (int i=0;i<nPoints;i++)
			center += p[i];
		center /= (float)nPoints;
	}
    
	
    
	//	sample 4 directions
    
    float4 aVector = p[0] - center;
    float4 u = cross3( nearNormal, aVector );
    float4 v = cross3( nearNormal, u );
    u = normalize3( u );
    v = normalize3( v );
    
    
    //keep point with deepest penetration
    float minW= FLT_MAX;
    
    int minIndex=-1;
    
    float4 maxDots;
    maxDots.x = FLT_MIN;
    maxDots.y = FLT_MIN;
    maxDots.z = FLT_MIN;
    maxDots.w = FLT_MIN;
    
    //	idx, distance
    for(int ie = 0; ie<nPoints; ie++ )
    {
        if (p[ie].w<minW)
        {
            minW = p[ie].w;
            minIndex=ie;
        }
        float f;
        float4 r = p[ie]-center;
        f = dot3F4( u, r );
        if (f<maxDots.x)
        {
            maxDots.x = f;
            contactIdx[0].x = ie;
        }
        
        f = dot3F4( -u, r );
        if (f<maxDots.y)
        {
            maxDots.y = f;
            contactIdx[0].y = ie;
        }
        
        
        f = dot3F4( v, r );
        if (f<maxDots.z)
        {
            maxDots.z = f;
            contactIdx[0].z = ie;
        }
        
        f = dot3F4( -v, r );
        if (f<maxDots.w)
        {
            maxDots.w = f;
            contactIdx[0].w = ie;
        }
        
    }
    
    if (contactIdx[0].x != minIndex && contactIdx[0].y != minIndex && contactIdx[0].z != minIndex && contactIdx[0].w != minIndex)
    {
        //replace the first contact with minimum (todo: replace contact with least penetration)
        contactIdx[0].x = minIndex;
    }
    
    return 4;
    
}





	



void computeContactPlaneConvex(int pairIndex,
																int bodyIndexA, int bodyIndexB, 
																int collidableIndexA, int collidableIndexB, 
																const b3RigidBodyCL* rigidBodies, 
																const b3Collidable* collidables,
																const b3ConvexPolyhedronCL* convexShapes,
																const b3Vector3* convexVertices,
																const int* convexIndices,
																const b3GpuFace* faces,
																b3Contact4* globalContactsOut,
																int& nGlobalContactsOut,
																int maxContactCapacity)
{

		int shapeIndex = collidables[collidableIndexB].m_shapeIndex;
	const b3ConvexPolyhedronCL* hullB = &convexShapes[shapeIndex];
	
	b3Vector3 posB = rigidBodies[bodyIndexB].m_pos;
	b3Quaternion ornB = rigidBodies[bodyIndexB].m_quat;
	b3Vector3 posA = rigidBodies[bodyIndexA].m_pos;
	b3Quaternion ornA = rigidBodies[bodyIndexA].m_quat;

	int numContactsOut = 0;
	int numWorldVertsB1= 0;

	b3Vector3 planeEq = faces[collidables[collidableIndexA].m_shapeIndex].m_plane;
	b3Vector3 planeNormal(planeEq.x,planeEq.y,planeEq.z);
	b3Vector3 planeNormalWorld = b3QuatRotate(ornA,planeNormal);
	float planeConstant = planeEq.w;
	b3Transform convexWorldTransform;
	convexWorldTransform.setIdentity();
	convexWorldTransform.setOrigin(posB);
	convexWorldTransform.setRotation(ornB);
	b3Transform planeTransform;
	planeTransform.setIdentity();
	planeTransform.setOrigin(posA);
	planeTransform.setRotation(ornA);

	b3Transform planeInConvex;
	planeInConvex= convexWorldTransform.inverse() * planeTransform;
	b3Transform convexInPlane;
	convexInPlane = planeTransform.inverse() * convexWorldTransform;
	
	b3Vector3 planeNormalInConvex = planeInConvex.getBasis()*-planeNormal;
	float maxDot = -1e30;
	int hitVertex=-1;
	b3Vector3 hitVtx;

#define MAX_PLANE_CONVEX_POINTS 64

	b3Vector3 contactPoints[MAX_PLANE_CONVEX_POINTS];
	int numPoints = 0;

	b3Int4 contactIdx;
	contactIdx.s[0] = 0;
	contactIdx.s[1] = 1;
	contactIdx.s[2] = 2;
	contactIdx.s[3] = 3;
	
	for (int i=0;i<hullB->m_numVertices;i++)
	{
		b3Vector3 vtx = convexVertices[hullB->m_vertexOffset+i];
		float curDot = vtx.dot(planeNormalInConvex);


		if (curDot>maxDot)
		{
			hitVertex=i;
			maxDot=curDot;
			hitVtx = vtx;
			//make sure the deepest points is always included
			if (numPoints==MAX_PLANE_CONVEX_POINTS)
				numPoints--;
		}

		if (numPoints<MAX_PLANE_CONVEX_POINTS)
		{
			b3Vector3 vtxWorld = convexWorldTransform*vtx;
			b3Vector3 vtxInPlane = planeTransform.inverse()*vtxWorld;
			float dist = planeNormal.dot(vtxInPlane)-planeConstant;
			if (dist<0.f)
			{
				vtxWorld.w = dist;
				contactPoints[numPoints] = vtxWorld;
				numPoints++;
			}
		}

	}

	int numReducedPoints  = 0;

	numReducedPoints = numPoints;
	
	if (numPoints>4)
	{
		numReducedPoints = extractManifoldSequentialGlobal( contactPoints, numPoints, planeNormalInConvex, &contactIdx);
	}
	int dstIdx;
//    dstIdx = nGlobalContactsOut++;//AppendInc( nGlobalContactsOut, dstIdx );
		
	if (numReducedPoints>0)
	{
		if (nGlobalContactsOut < maxContactCapacity)
		{
			dstIdx=nGlobalContactsOut;
			nGlobalContactsOut++;

			b3Contact4* c = &globalContactsOut[dstIdx];
			c->m_worldNormal = planeNormalWorld;
			c->setFrictionCoeff(0.7);
			c->setRestituitionCoeff(0.f);

			c->m_batchIdx = pairIndex;
			c->m_bodyAPtrAndSignBit = rigidBodies[bodyIndexA].m_invMass==0?-bodyIndexA:bodyIndexA;
			c->m_bodyBPtrAndSignBit = rigidBodies[bodyIndexB].m_invMass==0?-bodyIndexB:bodyIndexB;
			for (int i=0;i<numReducedPoints;i++)
			{
				b3Vector3 pOnB1 = contactPoints[contactIdx.s[i]];
				c->m_worldPos[i] = pOnB1;
			}
			c->m_worldNormal[3] = (b3Scalar)numReducedPoints;
		}//if (dstIdx < numPairs)
	}	
		


//	printf("computeContactPlaneConvex\n");
}




void computeContactPlaneCompound(int pairIndex,
																int bodyIndexA, int bodyIndexB, 
																int collidableIndexA, int collidableIndexB, 
																const b3RigidBodyCL* rigidBodies, 
																const b3Collidable* collidables,
																const b3ConvexPolyhedronCL* convexShapes,
																const b3Vector3* convexVertices,
																const int* convexIndices,
																const b3GpuFace* faces,
																b3Contact4* globalContactsOut,
																int& nGlobalContactsOut,
																int maxContactCapacity)
{

	int shapeTypeB = collidables[collidableIndexB].m_shapeType;
	b3Assert(shapeTypeB == SHAPE_COMPOUND_OF_CONVEX_HULLS);



	int shapeIndex = collidables[collidableIndexB].m_shapeIndex;
	const b3ConvexPolyhedronCL* hullB = &convexShapes[shapeIndex];
	
	b3Vector3 posB = rigidBodies[bodyIndexB].m_pos;
	b3Quaternion ornB = rigidBodies[bodyIndexB].m_quat;
	b3Vector3 posA = rigidBodies[bodyIndexA].m_pos;
	b3Quaternion ornA = rigidBodies[bodyIndexA].m_quat;

	int numContactsOut = 0;
	int numWorldVertsB1= 0;

	b3Vector3 planeEq = faces[collidables[collidableIndexA].m_shapeIndex].m_plane;
	b3Vector3 planeNormal(planeEq.x,planeEq.y,planeEq.z);
	b3Vector3 planeNormalWorld = b3QuatRotate(ornA,planeNormal);
	float planeConstant = planeEq.w;
	b3Transform convexWorldTransform;
	convexWorldTransform.setIdentity();
	convexWorldTransform.setOrigin(posB);
	convexWorldTransform.setRotation(ornB);
	b3Transform planeTransform;
	planeTransform.setIdentity();
	planeTransform.setOrigin(posA);
	planeTransform.setRotation(ornA);

	b3Transform planeInConvex;
	planeInConvex= convexWorldTransform.inverse() * planeTransform;
	b3Transform convexInPlane;
	convexInPlane = planeTransform.inverse() * convexWorldTransform;
	
	b3Vector3 planeNormalInConvex = planeInConvex.getBasis()*-planeNormal;
	float maxDot = -1e30;
	int hitVertex=-1;
	b3Vector3 hitVtx;

#define MAX_PLANE_CONVEX_POINTS 64

	b3Vector3 contactPoints[MAX_PLANE_CONVEX_POINTS];
	int numPoints = 0;

	b3Int4 contactIdx;
	contactIdx.s[0] = 0;
	contactIdx.s[1] = 1;
	contactIdx.s[2] = 2;
	contactIdx.s[3] = 3;
	
	for (int i=0;i<hullB->m_numVertices;i++)
	{
		b3Vector3 vtx = convexVertices[hullB->m_vertexOffset+i];
		float curDot = vtx.dot(planeNormalInConvex);


		if (curDot>maxDot)
		{
			hitVertex=i;
			maxDot=curDot;
			hitVtx = vtx;
			//make sure the deepest points is always included
			if (numPoints==MAX_PLANE_CONVEX_POINTS)
				numPoints--;
		}

		if (numPoints<MAX_PLANE_CONVEX_POINTS)
		{
			b3Vector3 vtxWorld = convexWorldTransform*vtx;
			b3Vector3 vtxInPlane = planeTransform.inverse()*vtxWorld;
			float dist = planeNormal.dot(vtxInPlane)-planeConstant;
			if (dist<0.f)
			{
				vtxWorld.w = dist;
				contactPoints[numPoints] = vtxWorld;
				numPoints++;
			}
		}

	}

	int numReducedPoints  = 0;

	numReducedPoints = numPoints;
	
	if (numPoints>4)
	{
		numReducedPoints = extractManifoldSequentialGlobal( contactPoints, numPoints, planeNormalInConvex, &contactIdx);
	}
	int dstIdx;
//    dstIdx = nGlobalContactsOut++;//AppendInc( nGlobalContactsOut, dstIdx );
		
	if (numReducedPoints>0)
	{
		if (nGlobalContactsOut < maxContactCapacity)
		{
			dstIdx=nGlobalContactsOut;
			nGlobalContactsOut++;

			b3Contact4* c = &globalContactsOut[dstIdx];
			c->m_worldNormal = planeNormalWorld;
			c->setFrictionCoeff(0.7);
			c->setRestituitionCoeff(0.f);

			c->m_batchIdx = pairIndex;
			c->m_bodyAPtrAndSignBit = rigidBodies[bodyIndexA].m_invMass==0?-bodyIndexA:bodyIndexA;
			c->m_bodyBPtrAndSignBit = rigidBodies[bodyIndexB].m_invMass==0?-bodyIndexB:bodyIndexB;
			for (int i=0;i<numReducedPoints;i++)
			{
				b3Vector3 pOnB1 = contactPoints[contactIdx.s[i]];
				c->m_worldPos[i] = pOnB1;
			}
			c->m_worldNormal[3] = (b3Scalar)numReducedPoints;
		}//if (dstIdx < numPairs)
	}	
		


//	printf("computeContactPlaneConvex\n");
}





void	computeContactSphereConvex(int pairIndex,
																int bodyIndexA, int bodyIndexB, 
																int collidableIndexA, int collidableIndexB, 
																const b3RigidBodyCL* rigidBodies, 
																const b3Collidable* collidables,
																const b3ConvexPolyhedronCL* convexShapes,
																const b3Vector3* convexVertices,
																const int* convexIndices,
																const b3GpuFace* faces,
																b3Contact4* globalContactsOut,
																int& nGlobalContactsOut,
																int maxContactCapacity)
{

	float radius = collidables[collidableIndexA].m_radius;
	float4 spherePos1 = rigidBodies[bodyIndexA].m_pos;
	b3Quaternion sphereOrn = rigidBodies[bodyIndexA].m_quat;



	float4 pos = rigidBodies[bodyIndexB].m_pos;
	

	b3Quaternion quat = rigidBodies[bodyIndexB].m_quat;

	b3Transform tr;
	tr.setIdentity();
	tr.setOrigin(pos);
	tr.setRotation(quat);
	b3Transform trInv = tr.inverse();

	float4 spherePos = trInv(spherePos1);

	int collidableIndex = rigidBodies[bodyIndexB].m_collidableIdx;
	int shapeIndex = collidables[collidableIndex].m_shapeIndex;
	int numFaces = convexShapes[shapeIndex].m_numFaces;
	float4 closestPnt = make_float4(0, 0, 0, 0);
	float4 hitNormalWorld = make_float4(0, 0, 0, 0);
	float minDist = -1000000.f; // TODO: What is the largest/smallest float?
	bool bCollide = true;
	int region = -1;
	float4 localHitNormal;
	for ( int f = 0; f < numFaces; f++ )
	{
		b3GpuFace face = faces[convexShapes[shapeIndex].m_faceOffset+f];
		float4 planeEqn;
		float4 localPlaneNormal = make_float4(face.m_plane.getX(),face.m_plane.getY(),face.m_plane.getZ(),0.f);
		float4 n1 = localPlaneNormal;//quatRotate(quat,localPlaneNormal);
		planeEqn = n1;
		planeEqn[3] = face.m_plane[3];

		float4 pntReturn;
		float dist = signedDistanceFromPointToPlane(spherePos, planeEqn, &pntReturn);

		if ( dist > radius)
		{
			bCollide = false;
			break;
		}

		if ( dist > 0 )
		{
			//might hit an edge or vertex
			b3Vector3 out;

			bool isInPoly = IsPointInPolygon(spherePos,
					&face,
					&convexVertices[convexShapes[shapeIndex].m_vertexOffset],
					convexIndices,
                    &out);
			if (isInPoly)
			{
				if (dist>minDist)
				{
					minDist = dist;
					closestPnt = pntReturn;
					localHitNormal = planeEqn;
					region=1;
				}
			} else
			{
				b3Vector3 tmp = spherePos-out;
				b3Scalar l2 = tmp.length2();
				if (l2<radius*radius)
				{
					dist  = b3Sqrt(l2);
					if (dist>minDist)
					{
						minDist = dist;
						closestPnt = out;
						localHitNormal = tmp/dist;
						region=2;
					}
					
				} else
				{
					bCollide = false;
					break;
				}
			}
		}
		else
		{
			if ( dist > minDist )
			{
				minDist = dist;
				closestPnt = pntReturn;
				localHitNormal = planeEqn;
				region=3;
			}
		}
	}
	static int numChecks = 0;
	numChecks++;

	if (bCollide && minDist > -10000)
	{
		
		float4 normalOnSurfaceB1 = tr.getBasis()*-localHitNormal;//-hitNormalWorld;
		float4 pOnB1 = tr(closestPnt);
		//printf("dist ,%f,",minDist);
		float actualDepth = minDist-radius;
		if (actualDepth<0)
		{
		//printf("actualDepth = ,%f,", actualDepth);
		//printf("normalOnSurfaceB1 = ,%f,%f,%f,", normalOnSurfaceB1.getX(),normalOnSurfaceB1.getY(),normalOnSurfaceB1.getZ());
		//printf("region=,%d,\n", region);
		pOnB1[3] = actualDepth;

		int dstIdx;
//    dstIdx = nGlobalContactsOut++;//AppendInc( nGlobalContactsOut, dstIdx );
		
		if (nGlobalContactsOut < maxContactCapacity)
		{
			dstIdx=nGlobalContactsOut;
			nGlobalContactsOut++;

			b3Contact4* c = &globalContactsOut[dstIdx];
			c->m_worldNormal = normalOnSurfaceB1;
			c->setFrictionCoeff(0.7);
			c->setRestituitionCoeff(0.f);

			c->m_batchIdx = pairIndex;
			c->m_bodyAPtrAndSignBit = rigidBodies[bodyIndexA].m_invMass==0?-bodyIndexA:bodyIndexA;
			c->m_bodyBPtrAndSignBit = rigidBodies[bodyIndexB].m_invMass==0?-bodyIndexB:bodyIndexB;
			c->m_worldPos[0] = pOnB1;
			int numPoints = 1;
			c->m_worldNormal[3] = (b3Scalar)numPoints;
		}//if (dstIdx < numPairs)
		}
	}//if (hasCollision)
	
}


void GpuSatCollision::computeConvexConvexContactsGPUSAT( const b3OpenCLArray<b3Int2>* pairs, int nPairs,
			const b3OpenCLArray<b3RigidBodyCL>* bodyBuf,
			b3OpenCLArray<b3Contact4>* contactOut, int& nContacts,
			int maxContactCapacity,
			const b3OpenCLArray<b3ConvexPolyhedronCL>& convexData,
			const b3OpenCLArray<b3Vector3>& gpuVertices,
			const b3OpenCLArray<b3Vector3>& gpuUniqueEdges,
			const b3OpenCLArray<b3GpuFace>& gpuFaces,
			const b3OpenCLArray<int>& gpuIndices,
			const b3OpenCLArray<b3Collidable>& gpuCollidables,
			const b3OpenCLArray<b3GpuChildShape>& gpuChildShapes,

			const b3OpenCLArray<b3YetAnotherAabb>& clAabbsWS,
            b3OpenCLArray<b3Vector3>& worldVertsB1GPU,
            b3OpenCLArray<b3Int4>& clippingFacesOutGPU,
            b3OpenCLArray<b3Vector3>& worldNormalsAGPU,
            b3OpenCLArray<b3Vector3>& worldVertsA1GPU,
            b3OpenCLArray<b3Vector3>& worldVertsB2GPU,    
			b3AlignedObjectArray<class b3OptimizedBvh*>& bvhData,
			b3OpenCLArray<b3QuantizedBvhNode>*	treeNodesGPU,
			b3OpenCLArray<b3BvhSubtreeInfo>*	subTreesGPU,
			b3OpenCLArray<b3BvhInfo>*	bvhInfo,

			int numObjects,
			int maxTriConvexPairCapacity,
			b3OpenCLArray<b3Int4>& triangleConvexPairsOut,
			int& numTriConvexPairsOut
			)
{
	if (!nPairs)
		return;


//#define CHECK_ON_HOST
#ifdef CHECK_ON_HOST
	b3AlignedObjectArray<b3YetAnotherAabb> hostAabbs;
	clAabbsWS.copyToHost(hostAabbs);
	b3AlignedObjectArray<b3Int2> hostPairs;
	pairs->copyToHost(hostPairs);

	b3AlignedObjectArray<b3RigidBodyCL> hostBodyBuf;
	bodyBuf->copyToHost(hostBodyBuf);

	

	b3AlignedObjectArray<b3ConvexPolyhedronCL> hostConvexData;
	convexData.copyToHost(hostConvexData);

	b3AlignedObjectArray<b3Vector3> hostVertices;
	gpuVertices.copyToHost(hostVertices);

	b3AlignedObjectArray<b3Vector3> hostUniqueEdges;
	gpuUniqueEdges.copyToHost(hostUniqueEdges);
	b3AlignedObjectArray<b3GpuFace> hostFaces;
	gpuFaces.copyToHost(hostFaces);
	b3AlignedObjectArray<int> hostIndices;
	gpuIndices.copyToHost(hostIndices);
	b3AlignedObjectArray<b3Collidable> hostCollidables;
	gpuCollidables.copyToHost(hostCollidables);
	
	b3AlignedObjectArray<b3GpuChildShape> cpuChildShapes;
	gpuChildShapes.copyToHost(cpuChildShapes);
	

	b3AlignedObjectArray<b3Int4> hostTriangleConvexPairs;

	b3AlignedObjectArray<b3Contact4> hostContacts;
	if (nContacts)
	{
		contactOut->copyToHost(hostContacts);
	}

	hostContacts.resize(nPairs);

	for (int i=0;i<nPairs;i++)
	{
		int bodyIndexA = hostPairs[i].x;
		int bodyIndexB = hostPairs[i].y;
		int collidableIndexA = hostBodyBuf[bodyIndexA].m_collidableIdx;
		int collidableIndexB = hostBodyBuf[bodyIndexB].m_collidableIdx;

		if (hostCollidables[collidableIndexA].m_shapeType == SHAPE_SPHERE &&
			hostCollidables[collidableIndexB].m_shapeType == SHAPE_CONVEX_HULL)
		{
			computeContactSphereConvex(i,bodyIndexA,bodyIndexB,collidableIndexA,collidableIndexB,&hostBodyBuf[0],
				&hostCollidables[0],&hostConvexData[0],&hostVertices[0],&hostIndices[0],&hostFaces[0],&hostContacts[0],nContacts,maxContactCapacity);
		}

		if (hostCollidables[collidableIndexA].m_shapeType == SHAPE_CONVEX_HULL &&
			hostCollidables[collidableIndexB].m_shapeType == SHAPE_SPHERE)
		{
			computeContactSphereConvex(i,bodyIndexB,bodyIndexA,collidableIndexB,collidableIndexA,&hostBodyBuf[0],
				&hostCollidables[0],&hostConvexData[0],&hostVertices[0],&hostIndices[0],&hostFaces[0],&hostContacts[0],nContacts,maxContactCapacity);
			//printf("convex-sphere\n");
			
		}

		if (hostCollidables[collidableIndexA].m_shapeType == SHAPE_CONVEX_HULL &&
			hostCollidables[collidableIndexB].m_shapeType == SHAPE_PLANE)
		{
			computeContactPlaneConvex(i,bodyIndexB,bodyIndexA,collidableIndexB,collidableIndexA,&hostBodyBuf[0],
			&hostCollidables[0],&hostConvexData[0],&hostVertices[0],&hostIndices[0],&hostFaces[0],&hostContacts[0],nContacts,maxContactCapacity);
//			printf("convex-plane\n");
			
		}

		if (hostCollidables[collidableIndexA].m_shapeType == SHAPE_PLANE &&
			hostCollidables[collidableIndexB].m_shapeType == SHAPE_CONVEX_HULL)
		{
			computeContactPlaneConvex(i,bodyIndexA,bodyIndexB,collidableIndexA,collidableIndexB,&hostBodyBuf[0],
			&hostCollidables[0],&hostConvexData[0],&hostVertices[0],&hostIndices[0],&hostFaces[0],&hostContacts[0],nContacts,maxContactCapacity);
//			printf("plane-convex\n");
			
		}


				if (hostCollidables[collidableIndexA].m_shapeType == SHAPE_COMPOUND_OF_CONVEX_HULLS &&
			hostCollidables[collidableIndexB].m_shapeType == SHAPE_PLANE)
		{
			computeContactPlaneCompound(i,bodyIndexB,bodyIndexA,collidableIndexB,collidableIndexA,&hostBodyBuf[0],
			&hostCollidables[0],&hostConvexData[0],&hostVertices[0],&hostIndices[0],&hostFaces[0],&hostContacts[0],nContacts,maxContactCapacity);
//			printf("convex-plane\n");
			
		}

		if (hostCollidables[collidableIndexA].m_shapeType == SHAPE_PLANE &&
			hostCollidables[collidableIndexB].m_shapeType == SHAPE_COMPOUND_OF_CONVEX_HULLS)
		{
			computeContactPlaneCompound(i,bodyIndexA,bodyIndexB,collidableIndexA,collidableIndexB,&hostBodyBuf[0],
			&hostCollidables[0],&hostConvexData[0],&hostVertices[0],&hostIndices[0],&hostFaces[0],&hostContacts[0],nContacts,maxContactCapacity);
//			printf("plane-convex\n");
			
		}


	}

	if (nContacts)
		{
			hostContacts.resize(nContacts);
			contactOut->copyFromHost(hostContacts);
		}

	
#else

	{
		if (nPairs)
		{
			m_totalContactsOut.copyFromHostPointer(&nContacts,1,0,true);

			B3_PROFILE("primitiveContactsKernel");
			b3BufferInfoCL bInfo[] = {
				b3BufferInfoCL( pairs->getBufferCL(), true ), 
				b3BufferInfoCL( bodyBuf->getBufferCL(),true), 
				b3BufferInfoCL( gpuCollidables.getBufferCL(),true), 
				b3BufferInfoCL( convexData.getBufferCL(),true),
				b3BufferInfoCL( gpuVertices.getBufferCL(),true),
				b3BufferInfoCL( gpuUniqueEdges.getBufferCL(),true),
				b3BufferInfoCL( gpuFaces.getBufferCL(),true),
				b3BufferInfoCL( gpuIndices.getBufferCL(),true),
				b3BufferInfoCL( contactOut->getBufferCL()),
				b3BufferInfoCL( m_totalContactsOut.getBufferCL())	
			};
			
			b3LauncherCL launcher(m_queue, m_primitiveContactsKernel);
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
			launcher.setConst( nPairs  );
			launcher.setConst(maxContactCapacity);
			int num = nPairs;
			launcher.launch1D( num);
			clFinish(m_queue);
		
			nContacts = m_totalContactsOut.at(0);
			contactOut->resize(nContacts);
		}
	}

#endif//CHECK_ON_HOST
	
	B3_PROFILE("computeConvexConvexContactsGPUSAT");
   // printf("nContacts = %d\n",nContacts);
    
	b3OpenCLArray<b3Vector3> sepNormals(m_context,m_queue);
	sepNormals.resize(nPairs);
	b3OpenCLArray<int> hasSeparatingNormals(m_context,m_queue);
	hasSeparatingNormals.resize(nPairs);
	
	int concaveCapacity=maxTriConvexPairCapacity;
	b3OpenCLArray<b3Vector3> concaveSepNormals(m_context,m_queue);
	concaveSepNormals.resize(concaveCapacity);

	b3OpenCLArray<int> numConcavePairsOut(m_context,m_queue);
	numConcavePairsOut.push_back(0);

	int compoundPairCapacity=65536*10;
	b3OpenCLArray<b3CompoundOverlappingPair> gpuCompoundPairs(m_context,m_queue);
	gpuCompoundPairs.resize(compoundPairCapacity);

	b3OpenCLArray<b3Vector3> gpuCompoundSepNormals(m_context,m_queue);
	gpuCompoundSepNormals.resize(compoundPairCapacity);
	
	
	b3OpenCLArray<int> gpuHasCompoundSepNormals(m_context,m_queue);
	gpuHasCompoundSepNormals.resize(compoundPairCapacity);
	
	b3OpenCLArray<int> numCompoundPairsOut(m_context,m_queue);
	numCompoundPairsOut.push_back(0);

	int numCompoundPairs = 0;

	bool findSeparatingAxisOnGpu = true;//false;
	int numConcavePairs =0;

	{
		clFinish(m_queue);
		if (findSeparatingAxisOnGpu)
		{
		
			{
				B3_PROFILE("findSeparatingAxisKernel");
				b3BufferInfoCL bInfo[] = { 
					b3BufferInfoCL( pairs->getBufferCL(), true ), 
					b3BufferInfoCL( bodyBuf->getBufferCL(),true), 
					b3BufferInfoCL( gpuCollidables.getBufferCL(),true), 
					b3BufferInfoCL( convexData.getBufferCL(),true),
					b3BufferInfoCL( gpuVertices.getBufferCL(),true),
					b3BufferInfoCL( gpuUniqueEdges.getBufferCL(),true),
					b3BufferInfoCL( gpuFaces.getBufferCL(),true),
					b3BufferInfoCL( gpuIndices.getBufferCL(),true),
					b3BufferInfoCL( clAabbsWS.getBufferCL(),true),
					b3BufferInfoCL( sepNormals.getBufferCL()),
					b3BufferInfoCL( hasSeparatingNormals.getBufferCL())
				};

				b3LauncherCL launcher(m_queue, m_findSeparatingAxisKernel);
				launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
				launcher.setConst( nPairs  );

				int num = nPairs;
				launcher.launch1D( num);
				clFinish(m_queue);
			}

			//now perform the tree query on GPU
			{
				
				
				
				{
				
					if (treeNodesGPU->size() && treeNodesGPU->size())
					{
						B3_PROFILE("m_bvhTraversalKernel");
						numConcavePairs = numConcavePairsOut.at(0);
						b3LauncherCL launcher(m_queue, m_bvhTraversalKernel);
						launcher.setBuffer( pairs->getBufferCL());
						launcher.setBuffer(  bodyBuf->getBufferCL());
						launcher.setBuffer( gpuCollidables.getBufferCL());
						launcher.setBuffer( clAabbsWS.getBufferCL());
						launcher.setBuffer( triangleConvexPairsOut.getBufferCL());
						launcher.setBuffer( numConcavePairsOut.getBufferCL());
						launcher.setBuffer( subTreesGPU->getBufferCL());
						launcher.setBuffer( treeNodesGPU->getBufferCL());
						launcher.setBuffer( bvhInfo->getBufferCL());
						
						launcher.setConst( nPairs  );
						launcher.setConst( maxTriConvexPairCapacity);
						int num = nPairs;
						launcher.launch1D( num);
						clFinish(m_queue);
						numConcavePairs = numConcavePairsOut.at(0);
						
						if (numConcavePairs > maxTriConvexPairCapacity)
						{
							static int exceeded_maxTriConvexPairCapacity_count = 0;
							printf("Rxceeded %d times the maxTriConvexPairCapacity (found %d but max is %d)\n", exceeded_maxTriConvexPairCapacity_count++,
								numConcavePairs,maxTriConvexPairCapacity);
							numConcavePairs = maxTriConvexPairCapacity;
						}
						triangleConvexPairsOut.resize(numConcavePairs);
						if (numConcavePairs)
						{
							//now perform a SAT test for each triangle-convex element (stored in triangleConvexPairsOut)
							B3_PROFILE("findConcaveSeparatingAxisKernel");
							b3BufferInfoCL bInfo[] = { 
								b3BufferInfoCL( triangleConvexPairsOut.getBufferCL() ), 
								b3BufferInfoCL( bodyBuf->getBufferCL(),true), 
								b3BufferInfoCL( gpuCollidables.getBufferCL(),true), 
								b3BufferInfoCL( convexData.getBufferCL(),true),
								b3BufferInfoCL( gpuVertices.getBufferCL(),true),
								b3BufferInfoCL( gpuUniqueEdges.getBufferCL(),true),
								b3BufferInfoCL( gpuFaces.getBufferCL(),true),
								b3BufferInfoCL( gpuIndices.getBufferCL(),true),
								b3BufferInfoCL( gpuChildShapes.getBufferCL(),true),
								b3BufferInfoCL( clAabbsWS.getBufferCL(),true),
								b3BufferInfoCL( concaveSepNormals.getBufferCL())
							};

							b3LauncherCL launcher(m_queue, m_findConcaveSeparatingAxisKernel);
							launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );

							launcher.setConst( numConcavePairs  );

							int num = numConcavePairs;
							launcher.launch1D( num);
							clFinish(m_queue);

//							b3AlignedObjectArray<b3Vector3> cpuCompoundSepNormals;
	//						concaveSepNormals.copyToHost(cpuCompoundSepNormals);
		//					b3AlignedObjectArray<b3Int4> cpuConcavePairs;
			//				triangleConvexPairsOut.copyToHost(cpuConcavePairs);


						}
					}
				}
			}
			
			numCompoundPairs = numCompoundPairsOut.at(0);

			if (1)
			{
				B3_PROFILE("findCompoundPairsKernel");
				b3BufferInfoCL bInfo[] = 
				{ 
					b3BufferInfoCL( pairs->getBufferCL(), true ), 
					b3BufferInfoCL( bodyBuf->getBufferCL(),true), 
					b3BufferInfoCL( gpuCollidables.getBufferCL(),true), 
					b3BufferInfoCL( convexData.getBufferCL(),true),
					b3BufferInfoCL( gpuVertices.getBufferCL(),true),
					b3BufferInfoCL( gpuUniqueEdges.getBufferCL(),true),
					b3BufferInfoCL( gpuFaces.getBufferCL(),true),
					b3BufferInfoCL( gpuIndices.getBufferCL(),true),
					b3BufferInfoCL( clAabbsWS.getBufferCL(),true),
					b3BufferInfoCL( gpuChildShapes.getBufferCL(),true),
					b3BufferInfoCL( gpuCompoundPairs.getBufferCL()),
					b3BufferInfoCL( numCompoundPairsOut.getBufferCL())
				};

				b3LauncherCL launcher(m_queue, m_findCompoundPairsKernel);
				launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
				launcher.setConst( nPairs  );
				launcher.setConst( compoundPairCapacity);

				int num = nPairs;
				launcher.launch1D( num);
				clFinish(m_queue);
			}


			numCompoundPairs = numCompoundPairsOut.at(0);
			//printf("numCompoundPairs =%d\n",numCompoundPairs );
			if (numCompoundPairs > compoundPairCapacity)
				numCompoundPairs = compoundPairCapacity;

			gpuCompoundPairs.resize(numCompoundPairs);
			gpuHasCompoundSepNormals.resize(numCompoundPairs);
			gpuCompoundSepNormals.resize(numCompoundPairs);
			

			if (numCompoundPairs)
			{
#ifndef CHECK_ON_HOST
				B3_PROFILE("processCompoundPairsPrimitivesKernel");
				b3BufferInfoCL bInfo[] = 
				{ 
					b3BufferInfoCL( gpuCompoundPairs.getBufferCL(), true ), 
					b3BufferInfoCL( bodyBuf->getBufferCL(),true), 
					b3BufferInfoCL( gpuCollidables.getBufferCL(),true), 
					b3BufferInfoCL( convexData.getBufferCL(),true),
					b3BufferInfoCL( gpuVertices.getBufferCL(),true),
					b3BufferInfoCL( gpuUniqueEdges.getBufferCL(),true),
					b3BufferInfoCL( gpuFaces.getBufferCL(),true),
					b3BufferInfoCL( gpuIndices.getBufferCL(),true),
					b3BufferInfoCL( clAabbsWS.getBufferCL(),true),
					b3BufferInfoCL( gpuChildShapes.getBufferCL(),true),
					b3BufferInfoCL( contactOut->getBufferCL()),
					b3BufferInfoCL( m_totalContactsOut.getBufferCL())	
				};

				b3LauncherCL launcher(m_queue, m_processCompoundPairsPrimitivesKernel);
				launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
				launcher.setConst( numCompoundPairs  );
				launcher.setConst(maxContactCapacity);

				int num = numCompoundPairs;
				launcher.launch1D( num);
				clFinish(m_queue);
				nContacts = m_totalContactsOut.at(0);
#endif
			}
			

			if (numCompoundPairs)
			{

				B3_PROFILE("processCompoundPairsKernel");
				b3BufferInfoCL bInfo[] = 
				{ 
					b3BufferInfoCL( gpuCompoundPairs.getBufferCL(), true ), 
					b3BufferInfoCL( bodyBuf->getBufferCL(),true), 
					b3BufferInfoCL( gpuCollidables.getBufferCL(),true), 
					b3BufferInfoCL( convexData.getBufferCL(),true),
					b3BufferInfoCL( gpuVertices.getBufferCL(),true),
					b3BufferInfoCL( gpuUniqueEdges.getBufferCL(),true),
					b3BufferInfoCL( gpuFaces.getBufferCL(),true),
					b3BufferInfoCL( gpuIndices.getBufferCL(),true),
					b3BufferInfoCL( clAabbsWS.getBufferCL(),true),
					b3BufferInfoCL( gpuChildShapes.getBufferCL(),true),
					b3BufferInfoCL( gpuCompoundSepNormals.getBufferCL()),
					b3BufferInfoCL( gpuHasCompoundSepNormals.getBufferCL())
				};

				b3LauncherCL launcher(m_queue, m_processCompoundPairsKernel);
				launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
				launcher.setConst( numCompoundPairs  );

				int num = numCompoundPairs;
				launcher.launch1D( num);
				clFinish(m_queue);

			
			}


			//printf("numConcave  = %d\n",numConcave);

		}//if (findSeparatingAxisOnGpu)


//		printf("hostNormals.size()=%d\n",hostNormals.size());
		//int numPairs = pairCount.at(0);
		
		
		
	}



	if (numConcavePairs)
	{
			if (numConcavePairs)
		{
			B3_PROFILE("findConcaveSphereContactsKernel");
				nContacts = m_totalContactsOut.at(0);
			b3BufferInfoCL bInfo[] = { 
				b3BufferInfoCL( triangleConvexPairsOut.getBufferCL() ), 
				b3BufferInfoCL( bodyBuf->getBufferCL(),true), 
				b3BufferInfoCL( gpuCollidables.getBufferCL(),true), 
				b3BufferInfoCL( convexData.getBufferCL(),true),
				b3BufferInfoCL( gpuVertices.getBufferCL(),true),
				b3BufferInfoCL( gpuUniqueEdges.getBufferCL(),true),
				b3BufferInfoCL( gpuFaces.getBufferCL(),true),
				b3BufferInfoCL( gpuIndices.getBufferCL(),true),
				b3BufferInfoCL( clAabbsWS.getBufferCL(),true),
				b3BufferInfoCL( contactOut->getBufferCL()),
				b3BufferInfoCL( m_totalContactsOut.getBufferCL())
			};

			b3LauncherCL launcher(m_queue, m_findConcaveSphereContactsKernel);
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );

			launcher.setConst( numConcavePairs  );
			launcher.setConst(maxContactCapacity);

			int num = numConcavePairs;
			launcher.launch1D( num);
			clFinish(m_queue);
			nContacts = m_totalContactsOut.at(0);
		}
		
	}



#ifdef __APPLE__
	bool contactClippingOnGpu = true;
#else
 bool contactClippingOnGpu = true;
#endif
	
	if (contactClippingOnGpu)
	{
		//B3_PROFILE("clipHullHullKernel");

		
		m_totalContactsOut.copyFromHostPointer(&nContacts,1,0,true);

		//concave-convex contact clipping

		if (numConcavePairs)
		{
//			printf("numConcavePairs = %d\n", numConcavePairs);
	//		nContacts = m_totalContactsOut.at(0);
		//	printf("nContacts before = %d\n", nContacts);

			B3_PROFILE("clipHullHullConcaveConvexKernel");
			nContacts = m_totalContactsOut.at(0);
			b3BufferInfoCL bInfo[] = { 
				b3BufferInfoCL( triangleConvexPairsOut.getBufferCL(), true ), 
				b3BufferInfoCL( bodyBuf->getBufferCL(),true), 
				b3BufferInfoCL( gpuCollidables.getBufferCL(),true), 
				b3BufferInfoCL( convexData.getBufferCL(),true),
				b3BufferInfoCL( gpuVertices.getBufferCL(),true),
				b3BufferInfoCL( gpuUniqueEdges.getBufferCL(),true),
				b3BufferInfoCL( gpuFaces.getBufferCL(),true),
				b3BufferInfoCL( gpuIndices.getBufferCL(),true),
				b3BufferInfoCL( gpuChildShapes.getBufferCL(),true),
				b3BufferInfoCL( concaveSepNormals.getBufferCL()),
				b3BufferInfoCL( contactOut->getBufferCL()),
				b3BufferInfoCL( m_totalContactsOut.getBufferCL())	
			};
			b3LauncherCL launcher(m_queue, m_clipHullHullConcaveConvexKernel);
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
			launcher.setConst( numConcavePairs  );
			int num = numConcavePairs;
			launcher.launch1D( num);
			clFinish(m_queue);
			nContacts = m_totalContactsOut.at(0);
			contactOut->resize(nContacts);
			b3AlignedObjectArray<b3Contact4> cpuContacts;
			contactOut->copyToHost(cpuContacts);
//			printf("nContacts after = %d\n", nContacts);
		}

		

		//convex-convex contact clipping
        if (1)
		{
			B3_PROFILE("clipHullHullKernel");
			bool breakupKernel = false;

#ifdef __APPLE__
			breakupKernel = true;
#endif

			if (breakupKernel)
			{


			
            int vertexFaceCapacity = 64;
            
            
            worldVertsB1GPU.resize(vertexFaceCapacity*nPairs);
            
            
            clippingFacesOutGPU.resize(nPairs);
            
            
            worldNormalsAGPU.resize(nPairs);
            
            
            worldVertsA1GPU.resize(vertexFaceCapacity*nPairs);
            
             
            worldVertsB2GPU.resize(vertexFaceCapacity*nPairs);
        
            
            
            {
				B3_PROFILE("findClippingFacesKernel");
            b3BufferInfoCL bInfo[] = {
                b3BufferInfoCL( pairs->getBufferCL(), true ),
                b3BufferInfoCL( bodyBuf->getBufferCL(),true),
                b3BufferInfoCL( gpuCollidables.getBufferCL(),true),
                b3BufferInfoCL( convexData.getBufferCL(),true),
                b3BufferInfoCL( gpuVertices.getBufferCL(),true),
                b3BufferInfoCL( gpuUniqueEdges.getBufferCL(),true),
                b3BufferInfoCL( gpuFaces.getBufferCL(),true), 
                b3BufferInfoCL( gpuIndices.getBufferCL(),true),
                b3BufferInfoCL( sepNormals.getBufferCL()),
                b3BufferInfoCL( hasSeparatingNormals.getBufferCL()),
                b3BufferInfoCL( clippingFacesOutGPU.getBufferCL()),
                b3BufferInfoCL( worldVertsA1GPU.getBufferCL()),
                b3BufferInfoCL( worldNormalsAGPU.getBufferCL()),
                b3BufferInfoCL( worldVertsB1GPU.getBufferCL())
            };
            
            b3LauncherCL launcher(m_queue, m_findClippingFacesKernel);
            launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
            launcher.setConst( vertexFaceCapacity);
            launcher.setConst( nPairs  );
            int num = nPairs;
            launcher.launch1D( num);
            clFinish(m_queue);

            }
            
  
          
            

            ///clip face B against face A, reduce contacts and append them to a global contact array
            if (1)
            {
				B3_PROFILE("clipFacesAndContactReductionKernel");
				//nContacts = m_totalContactsOut.at(0);
				//int h = hasSeparatingNormals.at(0);
				//int4 p = clippingFacesOutGPU.at(0);
                b3BufferInfoCL bInfo[] = {
                    b3BufferInfoCL( pairs->getBufferCL(), true ),
                    b3BufferInfoCL( bodyBuf->getBufferCL(),true),
                    b3BufferInfoCL( sepNormals.getBufferCL()),
                    b3BufferInfoCL( hasSeparatingNormals.getBufferCL()),
					b3BufferInfoCL( contactOut->getBufferCL()),
                    b3BufferInfoCL( clippingFacesOutGPU.getBufferCL()),
                    b3BufferInfoCL( worldVertsA1GPU.getBufferCL()),
                    b3BufferInfoCL( worldNormalsAGPU.getBufferCL()),
                    b3BufferInfoCL( worldVertsB1GPU.getBufferCL()),
                    b3BufferInfoCL( worldVertsB2GPU.getBufferCL()),
					b3BufferInfoCL( m_totalContactsOut.getBufferCL())
                };
                
                b3LauncherCL launcher(m_queue, m_clipFacesAndContactReductionKernel);
                launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
                launcher.setConst(vertexFaceCapacity);

				launcher.setConst( nPairs  );
                int debugMode = 0;
				launcher.setConst( debugMode);

				/*
				int serializationBytes = launcher.getSerializationBufferSize();
				unsigned char* buf = (unsigned char*)malloc(serializationBytes+1);
				int actualWritten = launcher.serializeArguments(buf,serializationBytes+1);
				FILE* f = fopen("clipFacesAndContactReductionKernel.bin","wb");
				fwrite(buf,actualWritten,1,f);
				fclose(f);
				free(buf);
				printf("serializationBytes=%d, actualWritten=%d\n",serializationBytes,actualWritten);
				*/

                int num = nPairs;

                launcher.launch1D( num);
                clFinish(m_queue);
                {
//                    nContacts = m_totalContactsOut.at(0);
  //                  printf("nContacts = %d\n",nContacts);
                    
                    contactOut->reserve(nContacts+nPairs);
                    
                    {
                        B3_PROFILE("newContactReductionKernel");
                            b3BufferInfoCL bInfo[] =
                        {
                            b3BufferInfoCL( pairs->getBufferCL(), true ),
                            b3BufferInfoCL( bodyBuf->getBufferCL(),true),
                            b3BufferInfoCL( sepNormals.getBufferCL()),
                            b3BufferInfoCL( hasSeparatingNormals.getBufferCL()),
                            b3BufferInfoCL( contactOut->getBufferCL()),
                            b3BufferInfoCL( clippingFacesOutGPU.getBufferCL()),
                            b3BufferInfoCL( worldVertsB2GPU.getBufferCL()),
                            b3BufferInfoCL( m_totalContactsOut.getBufferCL())
                        };
                        
                        b3LauncherCL launcher(m_queue, m_newContactReductionKernel);
                        launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
                        launcher.setConst(vertexFaceCapacity);
                        launcher.setConst( nPairs  );
                        int num = nPairs;
                        
                        launcher.launch1D( num);
                    }
                    nContacts = m_totalContactsOut.at(0);
                    contactOut->resize(nContacts);
                    
//                    Contact4 pt = contactOut->at(0);
                    
  //                  printf("nContacts = %d\n",nContacts);
                }
            }
	}            
	else
	{
	 
		if (nPairs)
		{
			b3BufferInfoCL bInfo[] = {
				b3BufferInfoCL( pairs->getBufferCL(), true ), 
				b3BufferInfoCL( bodyBuf->getBufferCL(),true), 
				b3BufferInfoCL( gpuCollidables.getBufferCL(),true), 
				b3BufferInfoCL( convexData.getBufferCL(),true),
				b3BufferInfoCL( gpuVertices.getBufferCL(),true),
				b3BufferInfoCL( gpuUniqueEdges.getBufferCL(),true),
				b3BufferInfoCL( gpuFaces.getBufferCL(),true),
				b3BufferInfoCL( gpuIndices.getBufferCL(),true),
				b3BufferInfoCL( sepNormals.getBufferCL()),
				b3BufferInfoCL( hasSeparatingNormals.getBufferCL()),
				b3BufferInfoCL( contactOut->getBufferCL()),
				b3BufferInfoCL( m_totalContactsOut.getBufferCL())	
			};
			b3LauncherCL launcher(m_queue, m_clipHullHullKernel);
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
			launcher.setConst( nPairs  );
			int num = nPairs;
			launcher.launch1D( num);
			clFinish(m_queue);
		
			nContacts = m_totalContactsOut.at(0);
			contactOut->resize(nContacts);
		}

		int nCompoundsPairs = gpuCompoundPairs.size();

		if (nCompoundsPairs)
		{
				b3BufferInfoCL bInfo[] = {
				b3BufferInfoCL( gpuCompoundPairs.getBufferCL(), true ), 
				b3BufferInfoCL( bodyBuf->getBufferCL(),true), 
				b3BufferInfoCL( gpuCollidables.getBufferCL(),true), 
				b3BufferInfoCL( convexData.getBufferCL(),true),
				b3BufferInfoCL( gpuVertices.getBufferCL(),true),
				b3BufferInfoCL( gpuUniqueEdges.getBufferCL(),true),
				b3BufferInfoCL( gpuFaces.getBufferCL(),true),
				b3BufferInfoCL( gpuIndices.getBufferCL(),true),
				b3BufferInfoCL( gpuChildShapes.getBufferCL(),true),
				b3BufferInfoCL( gpuCompoundSepNormals.getBufferCL(),true),
				b3BufferInfoCL( gpuHasCompoundSepNormals.getBufferCL(),true),
				b3BufferInfoCL( contactOut->getBufferCL()),
				b3BufferInfoCL( m_totalContactsOut.getBufferCL())	
			};
			b3LauncherCL launcher(m_queue, m_clipCompoundsHullHullKernel);
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(b3BufferInfoCL) );
			launcher.setConst( nCompoundsPairs  );
			launcher.setConst(maxContactCapacity);

			int num = nCompoundsPairs;
			launcher.launch1D( num);
			clFinish(m_queue);
		
			nContacts = m_totalContactsOut.at(0);
			contactOut->resize(nContacts);
		}
		}
		}

	}
}
