
#ifndef EXPLICIT_EULER_H
#define EXPLICIT_EULER_H

struct ExplicitEuler
{
	static void computeGravityForces(struct CpuSoftClothDemoInternalData* clothData, char* vtx, int vertexStride, float dt);
	static void	computeSpringForces(struct CpuSoftClothDemoInternalData* clothData, char* vertexPositions, int vertexStride, float dt);

	static void integrateExplicitEuler(struct CpuSoftClothDemoInternalData* clothData, char* vtx, int vertexStride,float dt);

	static void solveConstraints(struct CpuSoftClothDemoInternalData* clothData, char* vtx, int vertexStride,float dt);
	
};

#endif //EXPLICIT_EULER_H