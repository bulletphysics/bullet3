INCLUDE_DIRECTORIES(
	include
)
	
ADD_LIBRARY(cppunit

#core
	src/cppunit/AdditionalMessage.cpp
	src/cppunit/Asserter.cpp
	src/cppunit/Exception.cpp
	src/cppunit/Message.cpp
	src/cppunit/SourceLine.cpp
	src/cppunit/SynchronizedObject.cpp
	src/cppunit/Test.cpp
	src/cppunit/TestAssert.cpp
	src/cppunit/TestCase.cpp
	src/cppunit/TestComposite.cpp
	src/cppunit/TestFailure.cpp
	src/cppunit/TestLeaf.cpp
	src/cppunit/TestPath.cpp
	src/cppunit/TestResult.cpp
	src/cppunit/TestRunner.cpp
	src/cppunit/TestSuite.cpp

#extension	
	src/cppunit/RepeatedTest.cpp
	src/cppunit/TestCaseDecorator.cpp
	src/cppunit/TestDecorator.cpp
	src/cppunit/TestSetUp.cpp
	
#helper
	src/cppunit/TestFactoryRegistry.cpp
	src/cppunit/TestNamer.cpp
	src/cppunit/TestSuiteBuilderContext.cpp
	src/cppunit/TypeInfoHelper.cpp
	
#listener
	src/cppunit/BriefTestProgressListener.cpp
	src/cppunit/TestResultCollector.cpp
	src/cppunit/TestSuccessListener.cpp
	src/cppunit/TextTestProgressListener.cpp
	src/cppunit/TextTestResult.cpp
	
#output
	src/cppunit/CompilerOutputter.cpp
	src/cppunit/TextOutputter.cpp
	src/cppunit/XmlOutputter.cpp
	src/cppunit/XmlOutputterHook.cpp
	
#plugin	
	src/cppunit/BeOsDynamicLibraryManager.cpp
	src/cppunit/DynamicLibraryManager.cpp
	src/cppunit/DynamicLibraryManagerException.cpp
	src/cppunit/PlugInManager.cpp
	src/cppunit/PlugInParameters.cpp
	src/cppunit/ShlDynamicLibraryManager.cpp
	src/cppunit/TestPlugInDefaultImpl.cpp
	src/cppunit/UnixDynamicLibraryManager.cpp
	src/cppunit/Win32DynamicLibraryManager.cpp
	
#protector
	src/cppunit/DefaultProtector.cpp
	src/cppunit/Protector.cpp
	src/cppunit/ProtectorChain.cpp
	
#textui
	src/cppunit/TextTestRunner.cpp
	
#tools
	src/cppunit/StringTools.cpp
	src/cppunit/XmlDocument.cpp
	src/cppunit/XmlElement.cpp
	
)