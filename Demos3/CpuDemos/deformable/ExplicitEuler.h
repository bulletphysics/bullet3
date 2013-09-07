
#ifndef EXPLICIT_EULER_H
#define EXPLICIT_EULER_H

struct ExplicitEuler
{
	static void computeForces(struct CpuSoftClothDemoInternalData* data, char* vtx, int vertexStride, float dt);
	
	static void integrateExplicitEuler(struct CpuSoftClothDemoInternalData* data, char* vtx, int vertexStride,float dt);

	static void solveConstraints(struct CpuSoftClothDemoInternalData* data, char* vtx, int vertexStride,float dt);
	
};

#endif //EXPLICIT_EULER_H