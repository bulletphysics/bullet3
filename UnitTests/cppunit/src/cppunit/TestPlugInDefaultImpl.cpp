#include <cppunit/config/SourcePrefix.h>

#if !defined(CPPUNIT_NO_TESTPLUGIN)

#include <cppunit/TestSuite.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/plugin/TestPlugInDefaultImpl.h>


CPPUNIT_NS_BEGIN


TestPlugInDefaultImpl::TestPlugInDefaultImpl() 
{
}


TestPlugInDefaultImpl::~TestPlugInDefaultImpl()
{
}


void 
TestPlugInDefaultImpl::initialize( TestFactoryRegistry *,
                                   const PlugInParameters & )
{
}


void 
TestPlugInDefaultImpl::addListener( TestResult * )
{
}


void 
TestPlugInDefaultImpl::removeListener( TestResult * )
{
}


void 
TestPlugInDefaultImpl::addXmlOutputterHooks( XmlOutputter * )
{
}


void 
TestPlugInDefaultImpl::removeXmlOutputterHooks()
{
}


void 
TestPlugInDefaultImpl::uninitialize( TestFactoryRegistry * )
{
}


CPPUNIT_NS_END


#endif // !defined(CPPUNIT_NO_TESTPLUGIN)
