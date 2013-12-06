#include <cppunit/XmlOutputterHook.h>


CPPUNIT_NS_BEGIN


void 
XmlOutputterHook::beginDocument( XmlDocument * )
{
}


void 
XmlOutputterHook::endDocument( XmlDocument * )
{
}


void 
XmlOutputterHook::failTestAdded( XmlDocument *,
                                 XmlElement *,
                                 Test *,
                                 TestFailure * )
{
}


void 
XmlOutputterHook::successfulTestAdded( XmlDocument *,
                                       XmlElement *,
                                       Test * )
{
}


void 
XmlOutputterHook::statisticsAdded( XmlDocument *,
                                   XmlElement * )
{
}


CPPUNIT_NS_END

