
#include "b3GpuRaycast.h"
#include "Bullet3OpenCL/NarrowphaseCollision/b3Collidable.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3RigidBodyCL.h"
#include "Bullet3Common/b3Quickprof.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3OpenCLArray.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"
#include "Bullet3OpenCL/Raycast/kernels/rayCastKernels.h"


#define B3_RAYCAST_PATH "src/Bullet3OpenCL/Raycast/kernels/rayCastKernels.cl"



struct b3GpuRaycastInternalData
{
	cl_context m_context;
	cl_device_id m_device;
	cl_command_queue  m_q;
	cl_kernel	m_raytraceKernel;
	int m_test;
};

b3GpuRaycast::b3GpuRaycast(cl_context ctx,cl_device_id device, cl_command_queue  q)
{
	m_data = new b3GpuRaycastInternalData;
	m_data->m_context = ctx;
	m_data->m_device = device;
	m_data->m_q = q;
	m_data->m_raytraceKernel = 0;


	{
		cl_int errNum=0;
		cl_program prog = b3OpenCLUtils::compileCLProgramFromString(m_data->m_context,m_data->m_device,rayCastKernelCL,&errNum,"",B3_RAYCAST_PATH);
		b3Assert(errNum==CL_SUCCESS);
		m_data->m_raytraceKernel = b3OpenCLUtils::compileCLKernelFromString(m_data->m_context, m_data->m_device,rayCastKernelCL, "rayCastKernel",&errNum,prog);
		b3Assert(errNum==CL_SUCCESS);
		clReleaseProgram(prog);
	}


}

b3GpuRaycast::~b3GpuRaycast()
{
	clReleaseKernel(m_data->m_raytraceKernel);
	delete m_data;
}

bool sphere_intersect(const b3Vector3& spherePos,  b3Scalar radius, const b3Vector3& rayFrom, const b3Vector3& rayTo, float& hitFraction)
{
    b3Vector3 rs = rayFrom - spherePos;
	b3Vector3 rayDir = rayTo-rayFrom;
	
	float A = b3Dot(rayDir,rayDir);
    float B = b3Dot(rs, rayDir);
    float C = b3Dot(rs, rs) - (radius * radius);
    
	float D = B * B - A*C;

    if (D > 0.0)
    {
        float t = (-B - sqrt(D))/A;

        if ( (t >= 0.0f) && (t < hitFraction) )
        {
			hitFraction = t;
            return true;
		}
	}
	return false;
}


void b3GpuRaycast::castRaysHost(const b3AlignedObjectArray<b3RayInfo>& rays,	b3AlignedObjectArray<b3RayHit>& hitResults,
		int numBodies,const struct b3RigidBodyCL* bodies, int numCollidables,const struct b3Collidable* collidables)
{

//	return castRays(rays,hitResults,numBodies,bodies,numCollidables,collidables);

	B3_PROFILE("castRaysHost");
	for (int r=0;r<rays.size();r++)
	{
		b3Vector3 rayFrom = rays[r].m_from;
		b3Vector3 rayTo = rays[r].m_to;
		float hitFraction = hitResults[r].m_hitFraction;

		int hitBodyIndex= -1;

		for (int b=0;b<numBodies;b++)
		{
				
			const b3Vector3& pos = bodies[b].m_pos;
			const b3Quaternion& orn = bodies[b].m_quat;
			
			switch (collidables[bodies[b].m_collidableIdx].m_shapeType)
			{
			case SHAPE_SPHERE:
				{
					b3Scalar radius = collidables[bodies[b].m_collidableIdx].m_radius;
					if (sphere_intersect(pos,  radius, rayFrom, rayTo,hitFraction))
					{
						hitBodyIndex = b;
					}
				}

			default:
				{
					static bool once=true;
					if (once)
					{
						once=false;
						b3Warning("Raytest: unsupported shape type\n");
					}
				}
			}
		}
		if (hitBodyIndex>=0)
		{

			hitResults[r].m_hitFraction = hitFraction;
			hitResults[r].m_hitPoint.setInterpolate3(rays[r].m_from, rays[r].m_to,hitFraction);
			hitResults[r].m_hitNormal = (hitResults[r].m_hitPoint-bodies[hitBodyIndex].m_pos).normalize();
			hitResults[r].m_hitResult0 = hitBodyIndex;
		}

	}
}

void b3GpuRaycast::castRays(const b3AlignedObjectArray<b3RayInfo>& rays,	b3AlignedObjectArray<b3RayHit>& hitResults,
		int numBodies,const struct b3RigidBodyCL* bodies, int numCollidables, const struct b3Collidable* collidables)
{
	B3_PROFILE("castRaysGPU");

	b3OpenCLArray<b3RayInfo> gpuRays(m_data->m_context,m_data->m_q);
	gpuRays.copyFromHost(rays);

	b3OpenCLArray<b3RayHit> gpuHitResults(m_data->m_context,m_data->m_q);
	gpuHitResults.resize(hitResults.size());
	gpuHitResults.copyFromHost(hitResults);

	b3OpenCLArray<b3RigidBodyCL> gpuBodies(m_data->m_context,m_data->m_q);
	gpuBodies.resize(numBodies);
	gpuBodies.copyFromHostPointer(bodies,numBodies);

	b3OpenCLArray<b3Collidable> gpuCollidables(m_data->m_context,m_data->m_q);
	gpuCollidables.resize(numCollidables);
	gpuCollidables.copyFromHostPointer(collidables,numCollidables);


	//run kernel
	{
		B3_PROFILE("raycast launch1D");

		b3LauncherCL launcher(m_data->m_q,m_data->m_raytraceKernel);
		int numRays = rays.size();
		launcher.setConst(numRays);

		launcher.setBuffer(gpuRays.getBufferCL());
		launcher.setBuffer(gpuHitResults.getBufferCL());

		launcher.setConst(numBodies);
		launcher.setBuffer(gpuBodies.getBufferCL());
		launcher.setBuffer(gpuCollidables.getBufferCL());

		launcher.launch1D(numRays);
		clFinish(m_data->m_q);
	}

	//copy results
	gpuHitResults.copyToHost(hitResults);

}