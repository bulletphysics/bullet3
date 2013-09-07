#ifndef POSITION_BASED_DYNAMICS_H
#define POSITION_BASED_DYNAMICS_H

struct PositionBasedDynamics
{
	static void solveLinks(struct CpuSoftClothDemoInternalData* clothData, char* vtx, int vertexStride,float dt);
	static void solveConstraints(struct CpuSoftClothDemoInternalData* data, char* vtx, int vertexStride,float dt);
};

#endif //POSITION_BASED_DYNAMICS_H