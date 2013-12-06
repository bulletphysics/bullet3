#ifndef TEST_LINEAR_MATH_HAS_BEEN_INCLUDED
#define TEST_LINEAR_MATH_HAS_BEEN_INCLUDED

#include "cppunit/TestFixture.h"
#include "cppunit/extensions/HelperMacros.h"

#include "LinearMath/btScalar.h"

#define TEST_NUM_UNITSPHERE_POINTS 42

namespace
{
  struct compLess
  {
    bool operator()(const int& p1, const int& p2)  const
    {
      return p1 < p2;
    }
  };
}

static btVector3	sPenetrationDirections[TEST_NUM_UNITSPHERE_POINTS] = 
	{
	btVector3(btScalar(0.000000) , btScalar(-0.000000),btScalar(-1.000000)),
	btVector3(btScalar(0.723608) , btScalar(-0.525725),btScalar(-0.447219)),
	btVector3(btScalar(-0.276388) , btScalar(-0.850649),btScalar(-0.447219)),
	btVector3(btScalar(-0.894426) , btScalar(-0.000000),btScalar(-0.447216)),
	btVector3(btScalar(-0.276388) , btScalar(0.850649),btScalar(-0.447220)),
	btVector3(btScalar(0.723608) , btScalar(0.525725),btScalar(-0.447219)),
	btVector3(btScalar(0.276388) , btScalar(-0.850649),btScalar(0.447220)),
	btVector3(btScalar(-0.723608) , btScalar(-0.525725),btScalar(0.447219)),
	btVector3(btScalar(-0.723608) , btScalar(0.525725),btScalar(0.447219)),
	btVector3(btScalar(0.276388) , btScalar(0.850649),btScalar(0.447219)),
	btVector3(btScalar(0.894426) , btScalar(0.000000),btScalar(0.447216)),
	btVector3(btScalar(-0.000000) , btScalar(0.000000),btScalar(1.000000)),
	btVector3(btScalar(0.425323) , btScalar(-0.309011),btScalar(-0.850654)),
	btVector3(btScalar(-0.162456) , btScalar(-0.499995),btScalar(-0.850654)),
	btVector3(btScalar(0.262869) , btScalar(-0.809012),btScalar(-0.525738)),
	btVector3(btScalar(0.425323) , btScalar(0.309011),btScalar(-0.850654)),
	btVector3(btScalar(0.850648) , btScalar(-0.000000),btScalar(-0.525736)),
	btVector3(btScalar(-0.525730) , btScalar(-0.000000),btScalar(-0.850652)),
	btVector3(btScalar(-0.688190) , btScalar(-0.499997),btScalar(-0.525736)),
	btVector3(btScalar(-0.162456) , btScalar(0.499995),btScalar(-0.850654)),
	btVector3(btScalar(-0.688190) , btScalar(0.499997),btScalar(-0.525736)),
	btVector3(btScalar(0.262869) , btScalar(0.809012),btScalar(-0.525738)),
	btVector3(btScalar(0.951058) , btScalar(0.309013),btScalar(0.000000)),
	btVector3(btScalar(0.951058) , btScalar(-0.309013),btScalar(0.000000)),
	btVector3(btScalar(0.587786) , btScalar(-0.809017),btScalar(0.000000)),
	btVector3(btScalar(0.000000) , btScalar(-1.000000),btScalar(0.000000)),
	btVector3(btScalar(-0.587786) , btScalar(-0.809017),btScalar(0.000000)),
	btVector3(btScalar(-0.951058) , btScalar(-0.309013),btScalar(-0.000000)),
	btVector3(btScalar(-0.951058) , btScalar(0.309013),btScalar(-0.000000)),
	btVector3(btScalar(-0.587786) , btScalar(0.809017),btScalar(-0.000000)),
	btVector3(btScalar(-0.000000) , btScalar(1.000000),btScalar(-0.000000)),
	btVector3(btScalar(0.587786) , btScalar(0.809017),btScalar(-0.000000)),
	btVector3(btScalar(0.688190) , btScalar(-0.499997),btScalar(0.525736)),
	btVector3(btScalar(-0.262869) , btScalar(-0.809012),btScalar(0.525738)),
	btVector3(btScalar(-0.850648) , btScalar(0.000000),btScalar(0.525736)),
	btVector3(btScalar(-0.262869) , btScalar(0.809012),btScalar(0.525738)),
	btVector3(btScalar(0.688190) , btScalar(0.499997),btScalar(0.525736)),
	btVector3(btScalar(0.525730) , btScalar(0.000000),btScalar(0.850652)),
	btVector3(btScalar(0.162456) , btScalar(-0.499995),btScalar(0.850654)),
	btVector3(btScalar(-0.425323) , btScalar(-0.309011),btScalar(0.850654)),
	btVector3(btScalar(-0.425323) , btScalar(0.309011),btScalar(0.850654)),
	btVector3(btScalar(0.162456) , btScalar(0.499995),btScalar(0.850654))
	};



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

		//vec.setValue(1e-20,0,0);
		//vec.normalize();
		//CPPUNIT_ASSERT_DOUBLES_EQUAL( 1.0, vec.length2(), 1e-5 );

		vec.setValue(1e19,0,0);
		vec.normalize();
		CPPUNIT_ASSERT_DOUBLES_EQUAL( 1.0, vec.length2(), 1e-5 );
		
	}
	
	void testQuicksort()
	{
		int tests = 0;
		int numElems = 100;
		btAlignedObjectArray<int> m_unsortedIntegers;
		m_unsortedIntegers.resize(numElems);
		for (int i=0;i<numElems;i++)
		{
			m_unsortedIntegers[i] = i;
		}

		m_unsortedIntegers.quickSort(::compLess());
		for (int i=1;i<numElems;i++)
		{
			CPPUNIT_ASSERT(m_unsortedIntegers[i-1]<=m_unsortedIntegers[i]);
		}
		for (int i=0;i<numElems;i++)
		{
			m_unsortedIntegers[i] = numElems-i;
		}
		m_unsortedIntegers.quickSort(::compLess());
		for (int i=1;i<numElems;i++)
		{
			CPPUNIT_ASSERT(m_unsortedIntegers[i-1]<=m_unsortedIntegers[i]);
		}
		
		
	}
	

	void testQuaternionGetAxisAngle()
	{
		int steps=100;

		for (int j=0;j<TEST_NUM_UNITSPHERE_POINTS;j++)
		{
			btVector3 axis = sPenetrationDirections[j];

			for (int i=-steps+1;i<steps;i++)
			{
				btScalar angle=i*SIMD_2_PI/btScalar(steps);

				btQuaternion quat(axis,angle);
				btScalar compAngle = quat.getAngle();
				if (i>=0)
					CPPUNIT_ASSERT_DOUBLES_EQUAL( angle, compAngle, 1e-5 );
				else
				{
					CPPUNIT_ASSERT_DOUBLES_EQUAL( btFabs(angle), btFabs(compAngle), 1e-5 );
				}
				btVector3 compAxis = quat.getAxis();

				if (compAngle>SIMD_EPSILON)
				{
					if (i>=0)
					{
						CPPUNIT_ASSERT_DOUBLES_EQUAL( axis.getX(), compAxis.getX(), 1e-4 );
						CPPUNIT_ASSERT_DOUBLES_EQUAL( axis.getY(), compAxis.getY(), 1e-4 );
						CPPUNIT_ASSERT_DOUBLES_EQUAL( axis.getZ(), compAxis.getZ(), 1e-4 );
					} else
					{
						btScalar sign = compAngle*angle<0? -1 : 1;
						CPPUNIT_ASSERT_DOUBLES_EQUAL( sign*axis.getX(), compAxis.getX(), 1e-4 );
						CPPUNIT_ASSERT_DOUBLES_EQUAL( sign*axis.getY(), compAxis.getY(), 1e-4 );
						CPPUNIT_ASSERT_DOUBLES_EQUAL( sign*axis.getZ(), compAxis.getZ(), 1e-4 );
					}
				} else
				{
					CPPUNIT_ASSERT_DOUBLES_EQUAL( 1.f, compAxis.getX(), 1e-4 );
					CPPUNIT_ASSERT_DOUBLES_EQUAL( 0.f, compAxis.getY(), 1e-4 );
					CPPUNIT_ASSERT_DOUBLES_EQUAL( 0.f, compAxis.getZ(), 1e-4 );
				
				}
			}
		}
	}


	
	
	CPPUNIT_TEST_SUITE(TestLinearMath);
	CPPUNIT_TEST(testQuicksort);
	CPPUNIT_TEST(testNormalize);
	CPPUNIT_TEST(testQuaternionGetAxisAngle);
	
	CPPUNIT_TEST_SUITE_END();

private:

};

#endif //TEST_LINEAR_MATH_HAS_BEEN_INCLUDED
