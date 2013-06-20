
#define SHAPE_CONVEX_HULL 3
#define SHAPE_PLANE 4
#define SHAPE_CONCAVE_TRIMESH 5
#define SHAPE_COMPOUND_OF_CONVEX_HULLS 6
#define SHAPE_SPHERE 7


typedef struct
{
	float4 m_from;
	float4 m_to;
} b3RayInfo;

typedef struct
{
		float m_hitFraction;
		int	m_hitResult0;
		int	m_hitResult1;
		int	m_hitResult2;
		float4	m_hitPoint;
		float4	m_hitNormal;
} b3RayHit;

typedef struct
{
	float4 m_pos;
	float4 m_quat;
	float4 m_linVel;
	float4 m_angVel;

	unsigned int m_collidableIdx;
	float m_invMass;
	float m_restituitionCoeff;
	float m_frictionCoeff;
} Body;

typedef struct Collidable
{
	union {
		int m_numChildShapes;
		int m_bvhIndex;
	};
	float m_radius;
	int m_shapeType;
	int m_shapeIndex;
} Collidable;


typedef struct  
{
	float4		m_localCenter;
	float4		m_extents;
	float4		mC;
	float4		mE;
	
	float			m_radius;
	int	m_faceOffset;
	int m_numFaces;
	int	m_numVertices;
	
	int m_vertexOffset;
	int	m_uniqueEdgesOffset;
	int	m_numUniqueEdges;
	int m_unused;

} ConvexPolyhedronCL;

typedef struct
{
	float4 m_plane;
	int m_indexOffset;
	int m_numIndices;
} b3GpuFace;



///////////////////////////////////////
//	Quaternion
///////////////////////////////////////

typedef float4 Quaternion;

__inline
Quaternion qtMul(Quaternion a, Quaternion b);

__inline
Quaternion qtNormalize(Quaternion in);

__inline
float4 qtRotate(Quaternion q, float4 vec);

__inline
Quaternion qtInvert(Quaternion q);


__inline
float dot3F4(float4 a, float4 b)
{
	float4 a1 = (float4)(a.xyz,0.f);
	float4 b1 = (float4)(b.xyz,0.f);
	return dot(a1, b1);
}


__inline
Quaternion qtMul(Quaternion a, Quaternion b)
{
	Quaternion ans;
	ans = cross( a, b );
	ans += a.w*b+b.w*a;
//	ans.w = a.w*b.w - (a.x*b.x+a.y*b.y+a.z*b.z);
	ans.w = a.w*b.w - dot3F4(a, b);
	return ans;
}

__inline
Quaternion qtNormalize(Quaternion in)
{
	return fast_normalize(in);
//	in /= length( in );
//	return in;
}
__inline
float4 qtRotate(Quaternion q, float4 vec)
{
	Quaternion qInv = qtInvert( q );
	float4 vcpy = vec;
	vcpy.w = 0.f;
	float4 out = qtMul(qtMul(q,vcpy),qInv);
	return out;
}

__inline
Quaternion qtInvert(Quaternion q)
{
	return (Quaternion)(-q.xyz, q.w);
}

__inline
float4 qtInvRotate(const Quaternion q, float4 vec)
{
	return qtRotate( qtInvert( q ), vec );
}

__inline
float4 transform(const float4* p, const float4* translation, const Quaternion* orientation)
{
	return qtRotate( *orientation, *p ) + (*translation);
}

void	trInverse(float4 translationIn, Quaternion orientationIn,
		float4* translationOut, Quaternion* orientationOut)
{
	*orientationOut = qtInvert(orientationIn);
	*translationOut = qtRotate(*orientationOut, -translationIn);
}

void	trMul(float4 translationA, Quaternion orientationA,
						float4 translationB, Quaternion orientationB,
		float4* translationOut, Quaternion* orientationOut)
{
	*orientationOut = qtMul(orientationA,orientationB);
	*translationOut = transform(&translationB,&translationA,&orientationA);
}



bool rayConvex(float4 rayFromLocal, float4 rayToLocal, int numFaces, int faceOffset,
	__global const b3GpuFace* faces, float* hitFraction, float4* hitNormal)
{
	rayFromLocal.w = 0.f;
	rayToLocal.w = 0.f;
  bool result = true;
  
	float exitFraction = *hitFraction;
	float enterFraction = -0.1f;
	float4 curHitNormal = (float4)(0,0,0,0);
	for (int i=0;i<numFaces && result;i++)
	{
		b3GpuFace face = faces[faceOffset+i];
		float fromPlaneDist = dot(rayFromLocal,face.m_plane)+face.m_plane.w;
		float toPlaneDist = dot(rayToLocal,face.m_plane)+face.m_plane.w;
		if (fromPlaneDist<0.f)
		{
			if (toPlaneDist >= 0.f)
			{
				float fraction = fromPlaneDist / (fromPlaneDist-toPlaneDist);
				if (exitFraction>fraction)
				{
					exitFraction = fraction;
				}
			} 			
		} else
		{
			if (toPlaneDist<0.f)
			{
				float fraction = fromPlaneDist / (fromPlaneDist-toPlaneDist);
				if (enterFraction <= fraction)
				{
					enterFraction = fraction;
					curHitNormal = face.m_plane;
					curHitNormal.w = 0.f;
				}
			} else
			{
				result = false;
			}
		}
		if (exitFraction <= enterFraction)
			result = false;
	}
	
	result = result && (enterFraction < 0.f);
	
	if (result)
	{	
		*hitFraction = enterFraction;
		*hitNormal = curHitNormal;
	}
	return result;
}






bool sphere_intersect(float4 spherePos,  float radius, float4 rayFrom, float4 rayTo, float* hitFraction)
{
	float4 rs = rayFrom - spherePos;
	rs.w = 0.f;
	float4 rayDir = rayTo-rayFrom;
	rayDir.w = 0.f;
	float A = dot(rayDir,rayDir);
	float B = dot(rs, rayDir);
	float C = dot(rs, rs) - (radius * radius);
    
	float D = B * B - A*C;

  if (D > 0.0)
  {
     float t = (-B - sqrt(D))/A;

        if ( (t >= 0.0f) && (t < (*hitFraction)) )
        {
					*hitFraction = t;
          return true;
				}
	}
	return false;
}

float4 setInterpolate3(float4 from, float4 to, float t)
{
		float s = 1.0f - t;
		float4 result;
		result = s * from + t * to;
		result.w = 0.f;	
		return result;	
}

__kernel void rayCastKernel(  
	int numRays, 
	const __global b3RayInfo* rays, 
	__global b3RayHit* hitResults, 
	const int numBodies, 
	__global Body* bodies,
	__global Collidable* collidables,
	__global const b3GpuFace* faces,
	__global const ConvexPolyhedronCL* convexShapes	)
{

	int i = get_global_id(0);
	if (i<numRays)
	{
		hitResults[i].m_hitFraction = 1.f;

		float4 rayFrom = rays[i].m_from;
		float4 rayTo = rays[i].m_to;
		float hitFraction = 1.f;
		float4 hitPoint;
		float4 hitNormal;
		int hitBodyIndex= -1;
		
		int cachedCollidableIndex = -1;		
		Collidable cachedCollidable;
		
		for (int b=0;b<numBodies;b++)
		{
					
				float4 pos = bodies[b].m_pos;
				float4 orn = bodies[b].m_quat;
				if (cachedCollidableIndex !=bodies[b].m_collidableIdx)
				{
						cachedCollidableIndex = bodies[b].m_collidableIdx;
						cachedCollidable = collidables[cachedCollidableIndex];
				}
				
				if (cachedCollidable.m_shapeType == SHAPE_SPHERE)
				{
					float radius = cachedCollidable.m_radius;
		
					if (sphere_intersect(pos,  radius, rayFrom, rayTo, &hitFraction))
					{
						hitBodyIndex = b;
						hitPoint = setInterpolate3(rayFrom, rayTo,hitFraction);
						hitNormal = (float4) (hitPoint-bodies[b].m_pos);
					}
				}
				
				if (cachedCollidable.m_shapeType == SHAPE_CONVEX_HULL)
				{
				
					float4 invPos = (float4)(0,0,0,0);
					float4 invOrn = (float4)(0,0,0,0);
					float4 rayFromLocal = (float4)(0,0,0,0);
					float4 rayToLocal = (float4)(0,0,0,0);
					
					trInverse(pos,orn, &invPos, &invOrn);
					rayFromLocal = transform(&rayFrom, &invPos, &invOrn);
					rayToLocal = transform(&rayTo, &invPos, &invOrn);
					
					int numFaces = convexShapes[cachedCollidable.m_shapeIndex].m_numFaces;
					int faceOffset = convexShapes[cachedCollidable.m_shapeIndex].m_faceOffset;
					
					if (rayConvex(rayFromLocal, rayToLocal, numFaces, faceOffset,faces, &hitFraction, &hitNormal))
					{
						hitBodyIndex = b;
					}
				}
			
		}
		
		if (hitBodyIndex>=0)
		{
			hitResults[i].m_hitFraction = hitFraction;
			hitResults[i].m_hitPoint = hitPoint;
			hitResults[i].m_hitNormal = normalize(hitNormal);
			hitResults[i].m_hitResult0 = hitBodyIndex;
		}
	}
}
