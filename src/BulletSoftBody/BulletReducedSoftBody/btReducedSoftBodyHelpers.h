#ifndef BT_REDUCED_SOFT_BODY_HELPERS_H
#define BT_REDUCED_SOFT_BODY_HELPERS_H

#include "btReducedSoftBody.h"

struct btReducedSoftBodyHelpers
{
	// create a beam
	static btReducedSoftBody* createReducedBeam(btSoftBodyWorldInfo& worldInfo, const int num_modes);
	// create a cube
	static btReducedSoftBody* createReducedCube(btSoftBodyWorldInfo& worldInfo, const int num_modes);

	// read in geometry info from Vtk file
  static btReducedSoftBody* createFromVtkFile(btSoftBodyWorldInfo& worldInfo, const char* vtk_file);
	// read in all reduced files
	static void readReducedDeformableInfoFromFiles(btReducedSoftBody* rsb, const char* file_path, const btVector3& half_extents = btVector3(0, 0, 0));
	// read in a binary vector
	static void readBinaryVec(btReducedSoftBody::tDenseArray& vec, const unsigned int n_size, const char* file);
	// read in a binary matrix
	static void readBinaryMat(btReducedSoftBody::tDenseMatrix& mat, const unsigned int n_modes, const unsigned int n_full, const char* file);
	
	// calculate the local inertia tensor for a box shape reduced deformable object
	static void calculateLocalInertia(btVector3& inertia, const btScalar mass, const btVector3& half_extents, const btVector3& margin);
};


#endif // BT_REDUCED_SOFT_BODY_HELPERS_H