#ifndef CPPUNIT_EXTENSIONS_XMLINPUTHELPER_H
#define CPPUNIT_EXTENSIONS_XMLINPUTHELPER_H

#include <cppunit/ParameterizedTestCase.h>


/*! \brief Adds a parameterized test method to the suite.
 * \param testMethod Name of the method of the test case to add to the
 *                   suite. The signature of the method must be of
 *                   type: void testMethod(std::istream& param_in, std::istream& exp_in);
 * \see  CPPUNIT_TEST_SUITE.
 */
#define CPPUNIT_TEST_XML( testMethod)											                    \
	CPPUNIT_TEST_ADD( new CppUnit::ParameterizedTestCase<ThisTestFixtureType>(  \
					context.getTestNameFor( #testMethod ),							                \
					#testMethod,												                                \
					&TestFixtureType::testMethod,							                          \
					context.makeFixture(),										                          \
					context.getStringProperty( std::string("XmlFileName") ) ) )



#endif // CPPUNIT_EXTENSIONS_XMLINPUTHELPER_H