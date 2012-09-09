#ifndef TESTCHOLESKYDECOMPOSITION_H
#define TESTCHOLESKYDECOMPOSITION_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <LinearMath/btMatrix3x3.h>

class TestCholeskyDecomposition : public CppUnit::TestFixture
{
  public:

    void setUp() 
    {
      I.setIdentity();
    }

    void tearDown() 
    {
    }

    void testZeroMatrix();
    void testIdentityMatrix();
    void testPositiveDefiniteMatrix();
    void testPositiveSemiDefiniteMatrix();
    void testNegativeDefiniteMatrix();

    CPPUNIT_TEST_SUITE(TestCholeskyDecomposition);
    CPPUNIT_TEST(testZeroMatrix);
    CPPUNIT_TEST(testIdentityMatrix);
    CPPUNIT_TEST(testPositiveDefiniteMatrix);
    CPPUNIT_TEST(testPositiveSemiDefiniteMatrix);
    CPPUNIT_TEST(testNegativeDefiniteMatrix);
    CPPUNIT_TEST_SUITE_END();

  private:
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
    btMatrix3x3 L;
    btMatrix3x3 I;
};

#endif // TESTCHOLESKYDECOMPOSITION_H

