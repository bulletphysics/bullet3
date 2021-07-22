#ifndef BT_REDUCED_SOFT_BODY_HELPERS_H
#define BT_REDUCED_SOFT_BODY_HELPERS_H

#include "btReducedSoftBody.h"

struct btReducedSoftBodyHelpers
{
  static btReducedSoftBody* CreateFromVtkFile(btSoftBodyWorldInfo& worldInfo, const char* vtk_file);

	// TODO: no world info passed in here. may be required in the future
  // static btReducedSoftBody* CreateFromVtkFile(btSoftBodyWorldInfo& worldInfo, const char* vtk_file);

	// read in a binary vector
	static void readBinary(btReducedSoftBody::tDenseArray& vec, const unsigned int n_start, const unsigned int n_modes, const unsigned int n_full, const char* file);
	// read in a binary matrix
	static void readBinaryMat(btReducedSoftBody::tDenseMatrix& mat, const unsigned int n_start, const unsigned int n_modes, const unsigned int n_full, const char* file);
	// read in modes file (different version of read in matrix)
	static void readBinaryModes(btReducedSoftBody::tDenseMatrix& mat, const unsigned int n_start, const unsigned int n_modes, const unsigned int n_full, const char* file);
};


#endif // BT_REDUCED_SOFT_BODY_HELPERS_H