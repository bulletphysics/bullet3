#include "TestCholeskyDecomposition.h"
#include "btCholeskyDecomposition.h"

void TestCholeskyDecomposition::testZeroMatrix()
{
  const btMatrix3x3 A(0,0,0,0,0,0,0,0,0);
  const int result = choleskyDecompose(A, L);

  // The zero matrix is not positive definite so the decomposition does not
  // exist.
  CPPUNIT_ASSERT(result != 0);
}

void TestCholeskyDecomposition::testIdentityMatrix()
{
  const btMatrix3x3 A = I;
  const int result = choleskyDecompose(A, L);

  // The identity is a special case where the result should also be the
  // identity.
  CPPUNIT_ASSERT(result == 0);
  CPPUNIT_ASSERT(equal(L, I));
}

void TestCholeskyDecomposition::testPositiveDefiniteMatrix()
{
  const btMatrix3x3 M(3,0,0,1,2,0,3,2,1);
  const btMatrix3x3 A = M * M.transpose();
  const int result = choleskyDecompose(A, L);

  // By construction, the resulting decomposition of A should be equal to M
  CPPUNIT_ASSERT(result == 0);
  CPPUNIT_ASSERT(equal(L, M));
  CPPUNIT_ASSERT(equal(A, L * L.transpose()));
}

void TestCholeskyDecomposition::testPositiveSemiDefiniteMatrix()
{
  const btMatrix3x3 M(3,0,0,1,0,0,3,2,1);
  const btMatrix3x3 A = M * M.transpose();
  const int result = choleskyDecompose(A, L);

  // The matrix is semi definite, i.e. one of the eigenvalues is zero, so the
  // Cholesky decomposition does not exist.
  CPPUNIT_ASSERT(result != 0);
}

void TestCholeskyDecomposition::testNegativeDefiniteMatrix()
{
  const btMatrix3x3 M(3,0,0,1,2,0,3,2,1);
  const btMatrix3x3 A = M * M.transpose() * (-1.0);
  const int result = choleskyDecompose(A, L);

  // The matrix is negative definite, i.e. all of the eigenvalues are negative,
  // so the Cholesky decomposition does not exist.
  CPPUNIT_ASSERT(result != 0);
}

bool TestCholeskyDecomposition::equal(const btMatrix3x3& A, const btMatrix3x3& B) const
{
  for (unsigned int i = 0; i < 3; ++i)
    for (unsigned int j = 0; j < 3; ++j)
      if (btFabs(A[i][j] - B[i][j]) > SIMD_EPSILON)
        return false;

  return true;
}

