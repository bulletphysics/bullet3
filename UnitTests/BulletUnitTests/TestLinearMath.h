#ifndef TEST_LINEAR_MATH_HAS_BEEN_INCLUDED
#define TEST_LINEAR_MATH_HAS_BEEN_INCLUDED

#include "cppunit/TestFixture.h"
#include "cppunit/extensions/HelperMacros.h"

#include "LinearMath/btScalar.h"

// ---------------------------------------------------------------------------

class TestLinearMath : public CppUnit::TestFixture
{
	

public:

	void setUp() 
	{
	}

	void tearDown() 
	{
	}


	void testNormalize()
	{
		
		const btVector3 xaxis(1,0,0);
		const btVector3 yaxis(0,1,0);
		const btVector3 zaxis(0,0,1);
		
		const btVector3 negxaxis(-1,0,0);
		const btVector3 negyaxis(0,-1,0);
		const btVector3 negzaxis(0,0,-1);

		btVector3 vec;
		
		vec.setValue(1e-20,0,0);
		vec.safeNormalize();
		CPPUNIT_ASSERT_DOUBLES_EQUAL( 1.0, vec.length2(), 1e-6 );

		vec.setValue(1e20,0,0);
		vec.safeNormalize();
		CPPUNIT_ASSERT_DOUBLES_EQUAL( 1.0, vec.length2(), 1e-6 );

		vec.setValue(1e-20,0,0);
		vec.normalize();
		CPPUNIT_ASSERT_DOUBLES_EQUAL( 1.0, vec.length2(), 1e-5 );

		vec.setValue(1e19,0,0);
		vec.normalize();
		CPPUNIT_ASSERT_DOUBLES_EQUAL( 1.0, vec.length2(), 1e-5 );
		
	}

	CPPUNIT_TEST_SUITE(TestLinearMath);
	CPPUNIT_TEST(testNormalize);
	CPPUNIT_TEST_SUITE_END();

private:

};

#endif //TEST_LINEAR_MATH_HAS_BEEN_INCLUDED
