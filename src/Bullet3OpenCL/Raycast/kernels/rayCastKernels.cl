
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
	int m_unused1;
	int m_unused2;
	int m_shapeType;
	int m_shapeIndex;
} Collidable;

bool sphere_intersect(float4 spherePos,  float radius, float4 rayFrom, float4 rayTo)
{
    // rs = ray.org - sphere.center
  float4 rs = rayFrom - spherePos;
  rs.w = 0.f;
	float4 rayDir = (rayTo-rayFrom);
	rayDir.w = 0.f;
	rayDir = normalize(rayDir);

  float B = dot(rs, rayDir);
  float C = dot(rs, rs) - (radius * radius);
  float D = B * B - C;

    if (D > 0.0)
    {
        float t = -B - sqrt(D);
        if ( (t > 0.0))// && (t < isect.t) )
        {
            return true;//isect.t = t;
		}
	}
	return false;
}

__kernel void rayCastKernel(  
	int numRays, 
	const __global b3RayInfo* rays, 
	__global b3RayHit* hits, 
	const int numBodies, 
	__global Body* bodies,
	__global Collidable* collidables)
{

	bool hit=false;

	int i = get_global_id(0);
	if (i<numRays)
	{
		hits[i].m_hitFraction = 1.f;

		float4 rayFrom = rays[i].m_from;
		float4 rayTo = rays[i].m_to;
		
		for (int b=0;b<numBodies;b++)
		{
					
				float4 pos = bodies[b].m_pos;
	//		float4 orn = bodies[b].m_quat;
				
				float radius = 1.f;
	
				if (sphere_intersect(pos,  radius, rayFrom, rayTo))
					hit = true;
		}
		if (hit)
					hits[i].m_hitFraction = 0.f;
	}
}
