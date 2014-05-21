
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
	float4 out = qtMul(q,vcpy);
	out = qtMul(out,qInv);
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



void	trInverse(float4 translationIn, Quaternion orientationIn,
	float4* translationOut, Quaternion* orientationOut)
{
	*orientationOut = qtInvert(orientationIn);
	*translationOut = qtRotate(*orientationOut, -translationIn);
}







bool rayConvex(float4 rayFromLocal, float4 rayToLocal, int numFaces, int faceOffset,
	__global const b3GpuFace* faces, float* hitFraction, float4* hitNormal)
{
	rayFromLocal.w = 0.f;
	rayToLocal.w = 0.f;
	bool result = true;

	float exitFraction = hitFraction[0];
	float enterFraction = -0.3f;
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

	if (enterFraction < 0.f)
	{
		result = false;
	}

	if (result)
	{	
		hitFraction[0] = enterFraction;
		hitNormal[0] = curHitNormal;
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

	if (D > 0.0f)
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

float4 normalize3(float4 v)
{
	v.w = 0.f;
	return fast_normalize(v);	//vector4 normalize
}

float4 setInterpolate3(float4 from, float4 to, float t)
{
	float s = 1.0f - t;
	float4 result;
	result = s * from + t * to;
	result.w = 0.f;	
	return result;	
}

int rayIntersectsRigidBody(__global Body* bodies,
						__global Collidable* collidables,
						__global const b3GpuFace* faces,
						__global const ConvexPolyhedronCL* convexShapes,
						float4 rayFrom, float4 rayTo, int rigidBodyIndex, 
						float4* out_normal, float* out_hitFraction)
{
	Body body = bodies[rigidBodyIndex];
	Collidable rigidCollidable = collidables[body.m_collidableIdx];
	
	float hitFraction = 1.f;
	float4 hitNormal;
	int hitBodyIndex = -1;

	if (rigidCollidable.m_shapeType == SHAPE_CONVEX_HULL)
	{
		float4 invOrn = qtInvert(body.m_quat);
		float4 invPos = qtRotate(invOrn, -body.m_pos);
		float4 rayFromLocal = qtRotate( invOrn, rayFrom ) + invPos;
		float4 rayToLocal = qtRotate( invOrn, rayTo) + invPos;
		rayFromLocal.w = 0.f;
		rayToLocal.w = 0.f;
		
		int numFaces = convexShapes[rigidCollidable.m_shapeIndex].m_numFaces;
		int faceOffset = convexShapes[rigidCollidable.m_shapeIndex].m_faceOffset;
		
		if (numFaces && rayConvex(rayFromLocal, rayToLocal, numFaces, faceOffset, faces, &hitFraction, &hitNormal))
		{
			hitBodyIndex = rigidBodyIndex;
		}
	}
	
	if (rigidCollidable.m_shapeType == SHAPE_SPHERE)
	{
		if ( sphere_intersect(body.m_pos, rigidCollidable.m_radius, rayFrom, rayTo, &hitFraction) )
		{
			hitBodyIndex = rigidBodyIndex;
			float4 hitPoint = setInterpolate3(rayFrom, rayTo, hitFraction);
			hitNormal = (float4) (hitPoint - bodies[rigidBodyIndex].m_pos);
		}
	}
	
	if (hitBodyIndex >= 0)
	{
		*out_normal = normalize3(hitNormal);
		*out_hitFraction = hitFraction;
		return 1;
	}
	
	return 0;
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
	if (i >= numRays) return;

	hitResults[i].m_hitFraction = 1.f;

	float4 rayFrom = rays[i].m_from;
	float4 rayTo = rays[i].m_to;
	
	float nearestHitFraction = 1.f;
	float4 nearestNormal;
	int hitBodyIndex = -1;
	
	for (int rigidIndex = 0; rigidIndex < numBodies; rigidIndex++)
	{
		if (hitResults[i].m_hitResult2 == rigidIndex) continue;
		
		float hitFraction = 1.f;
		float4 hitNormal;
		int hasHit = rayIntersectsRigidBody(bodies, collidables, faces, convexShapes, rayFrom, rayTo, rigidIndex, &hitNormal, &hitFraction);
		if(hasHit && hitFraction < nearestHitFraction)
		{
			nearestHitFraction = hitFraction;
			nearestNormal = hitNormal;
			hitBodyIndex = rigidIndex;
		}
	}
	
	if (hitBodyIndex>=0)
	{
		float4 hitPoint = setInterpolate3(rayFrom, rayTo, nearestHitFraction);
		hitResults[i].m_hitFraction = nearestHitFraction;
		hitResults[i].m_hitPoint = hitPoint;
		hitResults[i].m_hitNormal = normalize(nearestNormal);
		hitResults[i].m_hitResult0 = hitBodyIndex;
	}
}


__kernel void findRayRigidPairIndexRanges(__global int2* rayRigidPairs,
											__global int* out_firstRayRigidPairIndexPerRay,
											__global int* out_numRayRigidPairsPerRay,
											int numRayRigidPairs)
{
	int rayRigidPairIndex = get_global_id(0);
	if (rayRigidPairIndex >= numRayRigidPairs) return;
	
	int rayIndex = rayRigidPairs[rayRigidPairIndex].x;
	
	atomic_min(&out_firstRayRigidPairIndexPerRay[rayIndex], rayRigidPairIndex);
	atomic_inc(&out_numRayRigidPairsPerRay[rayIndex]);
}

__kernel void rayCastPairsKernel(const __global b3RayInfo* rays,
								__global b3RayHit* hitResultsPerRay,
								
								__global Body* bodies,
								__global Collidable* collidables,
								__global const b3GpuFace* faces,
								__global const ConvexPolyhedronCL* convexShapes,
								
								__global int2* rayRigidPairs,
								__global float4* out_normalAndHitFractionPerPair,
								int numRayRigidPairs)
{
	int rayRigidPairIndex = get_global_id(0);
	if (rayRigidPairIndex >= numRayRigidPairs) return;

	int rayIndex = rayRigidPairs[rayRigidPairIndex].x;
	int rigidIndex = rayRigidPairs[rayRigidPairIndex].y;
	
	float4 normalAndHitFraction;	//normal in x,y,z and hitFraction in w
	normalAndHitFraction.w = 1.f;
	
	//m_hitResult2 == index of rigid body that is ignored by the ray
	if (hitResultsPerRay[rayIndex].m_hitResult2 != rigidIndex) 
	{
		float4 rayFrom = rays[rayIndex].m_from;
		float4 rayTo = rays[rayIndex].m_to;
		
		float hitFraction = 1.f;
		float4 hitNormal;
		int hasHit = rayIntersectsRigidBody(bodies, collidables, faces, convexShapes, rayFrom, rayTo, rigidIndex, &hitNormal, &hitFraction);
		if(hasHit)
		{
			normalAndHitFraction = normalize3(hitNormal);
			normalAndHitFraction.w = hitFraction;
		}
	}

	out_normalAndHitFractionPerPair[rayRigidPairIndex] = normalAndHitFraction;
}

__kernel void findFirstHitPerRay(const __global b3RayInfo* rays, 
								__global int2* rayRigidPairs,
								__global int* firstRayRigidPairIndexPerRay,
								__global int* numRayRigidPairsPerRay,
								__global float4* normalAndHitFractionPerPair,
								__global b3RayHit* out_hitResultsPerRay,
								int numRays)
{
	int rayIndex = get_global_id(0);
	if(rayIndex >= numRays) return;
	
	float nearestHitFraction = 1.f;
	int nearestRayRigidPairIndex = -1;
	
	for(int pair = 0; pair < numRayRigidPairsPerRay[rayIndex]; ++pair)
	{
		int rayRigidPairIndex = pair + firstRayRigidPairIndexPerRay[rayIndex];
		
		float4 normalAndHitFraction = normalAndHitFractionPerPair[rayRigidPairIndex];
		float hitFraction = normalAndHitFraction.w;
		
		if(hitFraction < nearestHitFraction)
		{
			nearestHitFraction = hitFraction;
			nearestRayRigidPairIndex = rayRigidPairIndex;
		}
	}
	
	b3RayHit result;
	result.m_hitFraction = 1.f;
	result.m_hitResult0 = -1;
	result.m_hitResult1 = out_hitResultsPerRay[rayIndex].m_hitResult1;
	result.m_hitResult2 = out_hitResultsPerRay[rayIndex].m_hitResult2;
	
	if(nearestRayRigidPairIndex != -1)
	{
		float4 normalAndHitFraction = normalAndHitFractionPerPair[nearestRayRigidPairIndex];
		
		float hitFraction = normalAndHitFraction.w;
		float4 hitNormal = normalAndHitFraction;
		hitNormal.w = 0.f;
		
		int rigidIndex = rayRigidPairs[nearestRayRigidPairIndex].y;
		
		result.m_hitFraction = hitFraction;
		result.m_hitResult0 = rigidIndex;
		result.m_hitPoint = setInterpolate3(rays[rayIndex].m_from, rays[rayIndex].m_to, hitFraction);
		result.m_hitNormal = hitNormal;
	}
	
	out_hitResultsPerRay[rayIndex] = result;
}


typedef float b3Scalar;
typedef float4 b3Vector3;
#define b3Max max
#define b3Min min
#define b3Sqrt sqrt

b3Vector3 b3Vector3_normalize(b3Vector3 v)
{
	b3Vector3 normal = (b3Vector3){v.x, v.y, v.z, 0.f};
	return normalize(normal);	//OpenCL normalize == vector4 normalize
}
b3Scalar b3Vector3_length2(b3Vector3 v) { return v.x*v.x + v.y*v.y + v.z*v.z; }
b3Scalar b3Vector3_dot(b3Vector3 a, b3Vector3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }

#define B3_PLVBH_TRAVERSE_MAX_STACK_SIZE 128
int isLeafNode(int index) { return (index >> 31 == 0); }
int getIndexWithInternalNodeMarkerRemoved(int index) { return index & (~0x80000000); }
int getIndexWithInternalNodeMarkerSet(int isLeaf, int index) { return (isLeaf) ? index : (index | 0x80000000); }

typedef struct
{
	unsigned int m_key;
	unsigned int m_value;
} SortDataCL;

typedef struct 
{
	union
	{
		float4	m_min;
		float   m_minElems[4];
		int			m_minIndices[4];
	};
	union
	{
		float4	m_max;
		float   m_maxElems[4];
		int			m_maxIndices[4];
	};
} b3AabbCL;

b3Scalar rayIntersectsAabb(b3Vector3 rayOrigin, b3Scalar rayLength, b3Vector3 rayNormalizedDirection, b3AabbCL aabb)
{
	//AABB is considered as 3 pairs of 2 planes( {x_min, x_max}, {y_min, y_max}, {z_min, z_max} ).
	//t_min is the point of intersection with the closer plane, t_max is the point of intersection with the farther plane.
	//
	//if (rayNormalizedDirection.x < 0.0f), then max.x will be the near plane 
	//and min.x will be the far plane; otherwise, it is reversed.
	//
	//In order for there to be a collision, the t_min and t_max of each pair must overlap.
	//This can be tested for by selecting the highest t_min and lowest t_max and comparing them.
	
	int4 isNegative = isless( rayNormalizedDirection, (b3Vector3){0.0f, 0.0f, 0.0f, 0.0f} );	//isless(x,y) returns (x < y)
	
	//When using vector types, the select() function checks the most signficant bit, 
	//but isless() sets the least significant bit.
	isNegative <<= 31;

	//select(b, a, condition) == condition ? a : b
	//When using select() with vector types, (condition[i]) is true if its most significant bit is 1
	b3Vector3 t_min = ( select(aabb.m_min, aabb.m_max, isNegative) - rayOrigin ) / rayNormalizedDirection;
	b3Vector3 t_max = ( select(aabb.m_max, aabb.m_min, isNegative) - rayOrigin ) / rayNormalizedDirection;
	
	b3Scalar t_min_final = 0.0f;
	b3Scalar t_max_final = rayLength;
	
	//Must use fmin()/fmax(); if one of the parameters is NaN, then the parameter that is not NaN is returned. 
	//Behavior of min()/max() with NaNs is undefined. (See OpenCL Specification 1.2 [6.12.2] and [6.12.4])
	//Since the innermost fmin()/fmax() is always not NaN, this should never return NaN.
	t_min_final = fmax( t_min.z, fmax(t_min.y, fmax(t_min.x, t_min_final)) );
	t_max_final = fmin( t_max.z, fmin(t_max.y, fmin(t_max.x, t_max_final)) );
	
	return (t_min_final <= t_max_final) ? (t_min_final / rayLength) : 1.f;
}

__kernel void plbvhRayTraverseFirstHit(__global b3AabbCL* rigidAabbs,	//Contains only small AABBs

										__global int* rootNodeIndex, 
										__global int2* internalNodeChildIndices, 
										__global b3AabbCL* internalNodeAabbs,
										__global SortDataCL* mortonCodesAndAabbIndices,
										
										__global Body* bodies,
										__global Collidable* collidables,
										__global const b3GpuFace* faces,
										__global const ConvexPolyhedronCL* convexShapes,
										
										__global b3RayInfo* rays,
										__global b3RayHit* out_hitResults,
										int numRigids, int numRays)
{
	int rayIndex = get_global_id(0);
	if(rayIndex >= numRays) return;
	
	//
	b3Vector3 rayFrom = rays[rayIndex].m_from;
	b3Vector3 rayTo = rays[rayIndex].m_to;
	b3Vector3 rayNormalizedDirection = b3Vector3_normalize(rayTo - rayFrom);
	b3Scalar rayLength = b3Sqrt( b3Vector3_length2(rayTo - rayFrom) );
	
	//
	int stack[B3_PLVBH_TRAVERSE_MAX_STACK_SIZE];
	
	int stackSize = 1;
	stack[0] = *rootNodeIndex;
	
	b3Scalar nearestHitFraction = 1.f;		//Actual ray-rigid hit fraction, not ray-AABB fraction
	b3Vector3 nearestNormal;
	int nearestRigidBodyIndex = -1;
	
	while(stackSize)
	{
		int internalOrLeafNodeIndex = stack[ stackSize - 1 ];
		--stackSize;
		
		int isLeaf = isLeafNode(internalOrLeafNodeIndex);	//Internal node if false
		int bvhNodeIndex = getIndexWithInternalNodeMarkerRemoved(internalOrLeafNodeIndex);
		
		//bvhRigidIndex is not used if internal node
		int bvhRigidIndex = (isLeaf) ? mortonCodesAndAabbIndices[bvhNodeIndex].m_value : -1;
	
		b3AabbCL bvhNodeAabb = (isLeaf) ? rigidAabbs[bvhRigidIndex] : internalNodeAabbs[bvhNodeIndex];
		
		b3Scalar aabbHitFraction = rayIntersectsAabb(rayFrom, rayLength, rayNormalizedDirection, bvhNodeAabb);
		if(aabbHitFraction != 1.f && aabbHitFraction <= nearestHitFraction)
		{
			if(isLeaf)
			{
				//bvhRigidIndex is the index of the small AABB in the BVH; not the actual rigid index
				int rigidIndex = bvhNodeAabb.m_minIndices[3];
				
				float hitFraction = 1.f;
				float4 hitNormal;
				
				int hasHit = rayIntersectsRigidBody(bodies, collidables, faces, convexShapes, rayFrom, rayTo, rigidIndex, &hitNormal, &hitFraction);
				if(hasHit && hitFraction < nearestHitFraction)
				{
					nearestHitFraction = hitFraction;
					nearestNormal = hitNormal;
					nearestRigidBodyIndex = rigidIndex;
				}
			}
			
			if(!isLeaf)	//Internal node
			{
				if(stackSize + 2 > B3_PLVBH_TRAVERSE_MAX_STACK_SIZE)
				{
					//Error
				}
				else
				{
					stack[ stackSize++ ] = internalNodeChildIndices[bvhNodeIndex].x;
					stack[ stackSize++ ] = internalNodeChildIndices[bvhNodeIndex].y;
				}
			}
		}
	}
	
	b3RayHit result;
	
	result.m_hitResult1 = out_hitResults[rayIndex].m_hitResult1;
	result.m_hitResult2 = out_hitResults[rayIndex].m_hitResult2;
	
	result.m_hitFraction = nearestHitFraction;
	result.m_hitResult0 = nearestRigidBodyIndex;
	result.m_hitPoint = setInterpolate3(rayFrom, rayTo, nearestHitFraction);
	result.m_hitNormal = nearestNormal;
	
	out_hitResults[rayIndex] = result;
}


__kernel void plbvhLargeAabbRayTestFirstHit(__global b3AabbCL* largeRigidAabbs, 

										__global Body* bodies,
										__global Collidable* collidables,
										__global const b3GpuFace* faces,
										__global const ConvexPolyhedronCL* convexShapes,

										__global b3RayInfo* rays,
										__global b3RayHit* hitResults,
										
										int numLargeAabbRigids, int numRays)
{
	int rayIndex = get_global_id(0);
	if(rayIndex >= numRays) return;
	
	b3Vector3 rayFrom = rays[rayIndex].m_from;
	b3Vector3 rayTo = rays[rayIndex].m_to;
	b3Vector3 rayNormalizedDirection = b3Vector3_normalize(rayTo - rayFrom);
	b3Scalar rayLength = b3Sqrt( b3Vector3_length2(rayTo - rayFrom) );
	
	
	b3RayHit result = hitResults[rayIndex];
	
	for(int i = 0; i < numLargeAabbRigids; ++i)
	{
		b3AabbCL rigidAabb = largeRigidAabbs[i];
		int rigidIndex = rigidAabb.m_minIndices[3];
		
		b3Scalar aabbHitFraction = rayIntersectsAabb(rayFrom, rayLength, rayNormalizedDirection, rigidAabb);
		if(aabbHitFraction != 1.f && aabbHitFraction <= result.m_hitFraction)
		{
			float hitFraction = 1.f;
			float4 hitNormal;
			int hasHit = rayIntersectsRigidBody(bodies, collidables, faces, convexShapes, rayFrom, rayTo, rigidIndex, &hitNormal, &hitFraction);
			if(hasHit && hitFraction < result.m_hitFraction)
			{
				result.m_hitFraction = hitFraction;
				result.m_hitResult0 = rigidIndex;
				result.m_hitPoint = setInterpolate3(rayFrom, rayTo, hitFraction);
				result.m_hitNormal = hitNormal;
			}
		}
	}
	
	hitResults[rayIndex] = result;
}

