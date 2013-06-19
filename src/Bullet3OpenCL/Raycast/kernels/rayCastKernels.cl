
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
	__global Collidable* collidables)
{


	int i = get_global_id(0);
	if (i<numRays)
	{
		hitResults[i].m_hitFraction = 1.f;

		float4 rayFrom = rays[i].m_from;
		float4 rayTo = rays[i].m_to;
		float hitFraction = 1.f;
		int hitBodyIndex= -1;
		
		int cachedCollidableIndex = -1;		
		Collidable cachedCollidable;
		
		for (int b=0;b<numBodies;b++)
		{
					
				float4 pos = bodies[b].m_pos;
	//		float4 orn = bodies[b].m_quat;
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
					}
				}
		}
		
		if (hitBodyIndex>=0)
		{
			hitResults[i].m_hitFraction = hitFraction;
			hitResults[i].m_hitPoint = setInterpolate3(rayFrom, rayTo,hitFraction);
			float4 hitNormal = (float4) (hitResults[i].m_hitPoint-bodies[hitBodyIndex].m_pos);
			hitResults[i].m_hitNormal = normalize(hitNormal);
			hitResults[i].m_hitResult0 = hitBodyIndex;
		}
	}
}
