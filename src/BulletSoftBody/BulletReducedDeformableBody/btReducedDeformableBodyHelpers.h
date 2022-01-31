#ifndef BT_REDUCED_SOFT_BODY_HELPERS_H
#define BT_REDUCED_SOFT_BODY_HELPERS_H

#include "btReducedDeformableBody.h"

struct btReducedDeformableBodyHelpers
{
	// create a beam
	static btReducedDeformableBody* createReducedBeam(btSoftBodyWorldInfo& worldInfo, const int num_modes);
	// create a cube
	static btReducedDeformableBody* createReducedCube(btSoftBodyWorldInfo& worldInfo, const int num_modes);
	// create a torus
	static btReducedDeformableBody* createReducedTorus(btSoftBodyWorldInfo& worldInfo, const int num_modes);

	// read in geometry info from Vtk file
  static btReducedDeformableBody* createFromVtkFile(btSoftBodyWorldInfo& worldInfo, const char* vtk_file);
	// read in all reduced files
	static void readReducedDeformableInfoFromFiles(btReducedDeformableBody* rsb, const char* file_path);
	// read in a binary vector
	static void readBinaryVec(btReducedDeformableBody::tDenseArray& vec, const unsigned int n_size, const char* file);
	// read in a binary matrix
	static void readBinaryMat(btReducedDeformableBody::tDenseMatrix& mat, const unsigned int n_modes, const unsigned int n_full, const char* file);
	
	// calculate the local inertia tensor for a box shape reduced deformable object
	static void calculateLocalInertia(btVector3& inertia, const btScalar mass, const btVector3& half_extents, const btVector3& margin);
};


#endif // BT_REDUCED_SOFT_BODY_HELPERS_H