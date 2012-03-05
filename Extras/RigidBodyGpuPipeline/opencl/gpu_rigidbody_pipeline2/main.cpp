/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Erwin Coumans

//
//#include "vld.h"
#include <GL/glew.h>

#include "GLInstancingRenderer.h"


#include "GLInstancingRenderer.h"
#include "../opengl_interop/btOpenCLGLInteropBuffer.h"
#include "Win32OpenGLRenderManager.h"
#include "CLPhysicsDemo.h"
#include "../broadphase_benchmark/btGridBroadphaseCl.h"
#include "../../opencl/gpu_rigidbody_pipeline/btGpuNarrowPhaseAndSolver.h"
#include "ShapeData.h"
#include "LinearMath/btQuickprof.h"

int NUM_OBJECTS_X = 32;
int NUM_OBJECTS_Y = 24;
int NUM_OBJECTS_Z = 32;


float X_GAP = 2.f;
float Y_GAP = 2.f;
float Z_GAP = 2.f;

extern int numPairsOut;


void createScene(GLInstancingRenderer& renderer,CLPhysicsDemo& physicsSim)
{
	int strideInBytes = sizeof(float)*9;

	int barrelShapeIndex = -1;
	int cubeShapeIndex = -1;

	float position[4]={0,0,0,0};
	float orn[4] = {0,0,0,1};
	float color[4] = {1,1,1,1};
	int index=0;
#if 1
	{
		int numVertices = sizeof(barrel_vertices)/strideInBytes;
		int numIndices = sizeof(barrel_indices)/sizeof(int);
		barrelShapeIndex = renderer.registerShape(&barrel_vertices[0],numVertices,barrel_indices,numIndices);
	}


	float barrelScaling[4] = {2,2,2,1};


	int barrelCollisionShapeIndex = physicsSim.registerCollisionShape(&barrel_vertices[0],strideInBytes, sizeof(barrel_vertices)/strideInBytes,&barrelScaling[0]);
	


	for (int i=0;i<NUM_OBJECTS_X;i++)
	{
		for (int j=0;j<(NUM_OBJECTS_Y/2);j++)
		{
			for (int k=0;k<NUM_OBJECTS_Z;k++)
			{
				float mass = j? 1.f : 0.f;

				position[0]=(i*X_GAP-NUM_OBJECTS_X/2)+5;
				position[1]=(j*Y_GAP*2-NUM_OBJECTS_Y/2);
				position[2]=(k*Z_GAP-NUM_OBJECTS_Z/2)-NUM_OBJECTS_Z*3;
				position[3] = 1.f;
				
				renderer.registerGraphicsInstance(barrelShapeIndex,position,orn,color,barrelScaling);
				void* ptr = (void*) index;
				physicsSim.registerPhysicsInstance(mass,  position, orn, barrelCollisionShapeIndex,ptr);
				
				index++;
			}
		}
	}
#endif

	float cubeScaling[4] = {2,2,2,1};
	int cubeCollisionShapeIndex = physicsSim.registerCollisionShape(&cube_vertices[0],strideInBytes, sizeof(cube_vertices)/strideInBytes,&cubeScaling[0]);


	{
		int numVertices = sizeof(cube_vertices)/strideInBytes;
		int numIndices = sizeof(cube_indices)/sizeof(int);
		cubeShapeIndex = renderer.registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	}

	for (int i=0;i<NUM_OBJECTS_X;i++)
	{
		for (int j=0;j<NUM_OBJECTS_Y/2;j++)
		{
			for (int k=0;k<NUM_OBJECTS_Z;k++)
			{
				float mass = 1.f;//j? 1.f : 0.f;

				position[0]=(i*X_GAP-NUM_OBJECTS_X/2)+(j&1);
				position[1]=(j*Y_GAP-NUM_OBJECTS_Y/2);
				position[2]=(k*Z_GAP-NUM_OBJECTS_Z/2)+(j&1);
				position[3] = 1.f;
				
				renderer.registerGraphicsInstance(cubeShapeIndex,position,orn,color,cubeScaling);
				void* ptr = (void*) index;
				physicsSim.registerPhysicsInstance(mass,  position, orn, cubeCollisionShapeIndex,ptr);
				
				index++;
			}
		}
	}

	if (1)
	{
		//add some 'special' plane shape
		void* ptr = (void*) index;
		position[0] = 0.f;
		position[1] = -NUM_OBJECTS_Y/2-1;
		position[2] = 0.f;
		position[3] = 1.f;

		physicsSim.registerPhysicsInstance(0.f,position, orn, -1,ptr);
		color[0] = 1.f;
		color[1] = 0.f;
		color[2] = 0.f;
		cubeScaling[0] = 5000.f;
		cubeScaling[1] = 0.01f;
		cubeScaling[2] = 5000.f;

		renderer.registerGraphicsInstance(cubeShapeIndex,position,orn,color,cubeScaling);
	}
	physicsSim.writeBodiesToGpu();


}

int main(int argc, char* argv[])
{
		
	Win32OpenGLWindow* window = new Win32OpenGLWindow();
		
	window->init(1024,768);
	GLenum err = glewInit();
	window->startRendering();
	window->endRendering();

	GLInstancingRenderer render;

	
		


	CLPhysicsDemo demo(window);
	
	bool useInterop = true;
	demo.init(-1,-1,useInterop);

		render.InitShaders();

		if (useInterop)
		demo.setupInterop();

	createScene(render, demo);
		

	printf("num objects = %d\n", NUM_OBJECTS_X*NUM_OBJECTS_Y*NUM_OBJECTS_Z);


	render.writeTransforms();


	while (!window->requestedExit())
	{
		CProfileManager::Reset();
		
		demo.stepSimulation();


		window->startRendering();
		render.RenderScene();
		window->endRendering();

		CProfileManager::Increment_Frame_Counter();

		static bool printStats  = true;

		 if (printStats)
		 {
			static int count = 10;
			count--;
			if (count<0)
			{
				CProfileManager::dumpAll();
				//printf("total broadphase pairs= %d\n", gFpIO.m_numOverlap);
				printf("numPairsOut (culled)  = %d\n", numPairsOut);
				printStats  = false;
			}
		 }

	}

	
	demo.cleanup();

	render.CleanupShaders();
	window->exit();
	delete window;
	
	
	
	return 0;
}