#include "TestPolarDecomposition.h"
#include "BulletSoftBody/btSoftBodyInternals.h"
#include "LinearMath/btPolarDecomposition.h"
#include "btCholeskyDecomposition.h"

namespace
{
  const int MAX_ITERATIONS = btPolarDecomposition::DEFAULT_MAX_ITERATIONS;
  const btScalar TOLERANCE = btPolarDecomposition::DEFAULT_TOLERANCE;
}

void TestPolarDecomposition::testZeroMatrix()
{
  const btMatrix3x3 A(0,0,0,0,0,0,0,0,0);
  const int iterations = PolarDecompose(A, U, H);

  // The decomposition should fail because the zero matrix is singular
  CPPUNIT_ASSERT(iterations == MAX_ITERATIONS);
}

void TestPolarDecomposition::testSingularMatrix()
{
  const btMatrix3x3 A(1,0,0,1,0,0,0,0,1);

  const int iterations = PolarDecompose(A, U, H);

  // The cdecomposition should fail because the matrix is singular.
  CPPUNIT_ASSERT(iterations == MAX_ITERATIONS);
}

void TestPolarDecomposition::testPoorlyConditionedMatrix()
{
  const btScalar e = btScalar(TOLERANCE);
  const btMatrix3x3 A(1,0,0,1,e,0,0,0,1);

  const int iterations = PolarDecompose(A, U, H);

  // The decomposition should succeed, however, the matrix is poorly
  // conditioned, i.e. as 'e' approaches zero, the matrix approaches a singular
  // matrix (no inverse).
  CPPUNIT_ASSERT(iterations < MAX_ITERATIONS);
  CPPUNIT_ASSERT(equal(A, U * H));
  CPPUNIT_ASSERT(isOrthogonal(U));
  CPPUNIT_ASSERT(isSymmetric(H));
  CPPUNIT_ASSERT(isPositiveDefinite(H));
}

void TestPolarDecomposition::testIdentityMatrix()
{
  const btMatrix3x3 A = I;
  const int iterations = PolarDecompose(A, U, H);

  // The identity is a special case. The decomposition should succeed and both
  // the U and H matrices should be equal to the identity.
  CPPUNIT_ASSERT(iterations < MAX_ITERATIONS);
  CPPUNIT_ASSERT(equal(A, U * H));
  CPPUNIT_ASSERT(equal(U, I));
  CPPUNIT_ASSERT(equal(H, I));
}

void TestPolarDecomposition::testNonSingularMatrix()
{
  const btMatrix3x3 A(4, 6, 6, 9, 2, 0, 1, 6, 0);
  const int iterations = PolarDecompose(A, U, H);

  // The decomposition should succeed so that A = U*H, where U is orthogonal and
  // H is symmetric and positive definite.
  CPPUNIT_ASSERT(iterations < MAX_ITERATIONS);
  CPPUNIT_ASSERT(equal(A, U * H));
  CPPUNIT_ASSERT(isOrthogonal(U));
  CPPUNIT_ASSERT(isSymmetric(H));
  CPPUNIT_ASSERT(isPositiveDefinite(H));
}

bool TestPolarDecomposition::equal(const btMatrix3x3& A, const btMatrix3x3& B) const
{
  for (unsigned int i = 0; i < 3; ++i)
    for (unsigned int j = 0; j < 3; ++j)
      if (btFabs(A[i][j] - B[i][j]) > TOLERANCE)
        return false;

  return true;
}

bool TestPolarDecomposition::isOrthogonal(const btMatrix3x3& A) const
{
  return equal(A.transpose() * A, I);
}

bool TestPolarDecomposition::isPositiveDefinite(const btMatrix3x3& A) const
{
  static btMatrix3x3 storage;
  return (0 == choleskyDecompose(A, storage));
}

bool TestPolarDecomposition::isSymmetric(const btMatrix3x3& A) const
{
  return 
    (btFabs(A[0][1] - A[1][0]) < TOLERANCE) &&
    (btFabs(A[0][2] - A[2][0]) < TOLERANCE) &&
    (btFabs(A[1][2] - A[2][1]) < TOLERANCE);
}

