#ifndef TESTPOLARDECOMPOSITION_H
#define TESTPOLARDECOMPOSITION_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <LinearMath/btMatrix3x3.h>

class TestPolarDecomposition : public CppUnit::TestFixture
{
  public:

    void setUp() 
    {
      I.setIdentity();
    }

    void tearDown() 
    {
    }

    void testPoorlyConditionedMatrix();
    void testSingularMatrix();
    void testNonSingularMatrix();
    void testIdentityMatrix();
    void testZeroMatrix();

    CPPUNIT_TEST_SUITE(TestPolarDecomposition);
    CPPUNIT_TEST(testZeroMatrix);
    CPPUNIT_TEST(testSingularMatrix);
    CPPUNIT_TEST(testPoorlyConditionedMatrix);
    CPPUNIT_TEST(testIdentityMatrix);
    CPPUNIT_TEST(testNonSingularMatrix);
    CPPUNIT_TEST_SUITE_END();

  private:
    /**
     * Returns TRUE if the specified matrix is orthogonal and FALSE otherwise.
     * Orthogonality is checked by pre-multiplying the matrix by its transpose
     * and comparing the result to the identity matrix.
     *
     * @param A - the matrix that is being tested
     *
     * @return a boolean indicating whether the specified matrix is orthogonal.
     */
    bool isOrthogonal(const btMatrix3x3& A) const;

    /**
     * Returns TRUE if the specified matrix is symmetric and FALSE otherwise.
     * The symmetry of the matrix is determined by simplying testing the
     * equality of the three off-diagonal elements.
     *
     * @param A - the matrix that is being tested.
     *
     * @return a boolean indicating whether the specified matrix is symmetric.
     */
    bool isSymmetric(const btMatrix3x3& A) const;

    /**
     * Returns TRUE if the specified matrix is positive definite and FALSE
     * otherwise. The positive definiteness of the matrix is determined by
     * calculating the Cholesky decomposition of the matrix; If the Cholesky
     * decomposition exists, the original matrix is positive definite.
     *
     * @param A - the matrix that is being tested.
     *
     * @return a boolean indicating whether the specified matrix is positive
     * definite.
     */
    bool isPositiveDefinite(const btMatrix3x3& A) const;

    /**
     * Returns TRUE if the specified matrices are equal and FALSE otherwise.
     *
     * @param A - the first matrix to be tested.
     * @param B - the second matrix to be tested.
     *
     * @return a boolean indicating whether the specified matrix is symmetric.
     */
    bool equal(const btMatrix3x3& A, const btMatrix3x3& B) const;

  private:
    btMatrix3x3 U;
    btMatrix3x3 H;
    btMatrix3x3 I;
};

#endif // TESTPOLARDECOMPOSITION_H

