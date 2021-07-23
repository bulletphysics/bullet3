#ifndef BT_REDUCED_SOFT_BODY_H
#define BT_REDUCED_SOFT_BODY_H

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btVector3.h"

// #include "BulletDynamics/Dynamics/btRigidBody.h"
#include "../btSoftBody.h"

// Reduced deformable body is a simplified deformable object embedded in a rigid frame.
class btReducedSoftBody : public btSoftBody
{
 public:
  //
  //  Typedefs
  //
  typedef btAlignedObjectArray<btVector3> TVStack;
  typedef btAlignedObjectArray<btScalar> tDenseArray;
  typedef btAlignedObjectArray<btAlignedObjectArray<btScalar> > tDenseMatrix;

  using btSoftBody::tNodeArray;

  //
  //  Fields
  //
  
  bool m_reducedModel;																	 // Reduced deformable model flag

  // reduced space
	tDenseMatrix m_modes;														// modes of the reduced deformable model. Each inner array is a mode, outer array size = n_modes
	tDenseArray m_reducedDofs;				   // Reduced degree of freedom
	tDenseArray m_reducedVelocity;		   // Reduced velocity array
	tDenseArray m_eigenvalues;		// eigenvalues of the reduce deformable model
	tDenseArray m_Kr;	// reduced stiffness matrix
	tDenseArray m_Mr;	// reduced mass matrix //TODO: do we need this?
  
  // full space
  TVStack m_x0;									 // Rest position

  // rigid frame


  //
  // Api
  //
  btReducedSoftBody(btSoftBodyWorldInfo* worldInfo, int node_count, const btVector3* x, const btScalar* m);
  ~btReducedSoftBody() {}


};

#endif // BT_REDUCED_SOFT_BODY_H