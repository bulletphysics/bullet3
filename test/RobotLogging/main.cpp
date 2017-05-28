#include "Utils/RobotLoggingUtil.h"

#ifndef ENABLE_GTEST

#include <assert.h>
#define ASSERT_EQ(a,b) assert((a)==(b));
#else
#include <gtest/gtest.h>
#define printf
#endif


void testMinitaurLogging()
{
	const char* fileName = "d:/logTest.txt";
	btAlignedObjectArray<std::string> structNames;
	std::string structTypes;
	btAlignedObjectArray<MinitaurLogRecord> logRecords;
	bool verbose = false;

	int status = readMinitaurLogFile(fileName, structNames, structTypes, logRecords, verbose);

	for (int i=0;i<logRecords.size();i++)
	{
		for (int j=0;j<structTypes.size();j++)
		{
			switch (structTypes[j])
			{
				case 'I':
				{
					int v = logRecords[i].m_values[j].m_intVal;
					printf("record %d, %s = %d\n",i,structNames[j].c_str(),v);
					break;
				}
				case 'f':
				{
					float v = logRecords[i].m_values[j].m_floatVal;
					printf("record %d, %s = %f\n",i,structNames[j].c_str(),v);
					break;

				}
				case 'B':
				{
					int v = logRecords[i].m_values[j].m_charVal;
					printf("record %d, %s = %d\n",i,structNames[j].c_str(),v);
					break;
				}
				default:
				{
				}
			}
		}
	}
}

#ifdef ENABLE_GTEST
TEST(RobotLoggingTest, LogMinitaur) {
        testMinitaurLogging();
}
#endif

int main(int argc, char* argv[])
{

//b3SetCustomPrintfFunc(myprintf);
//b3SetCustomWarningMessageFunc(myprintf);

#ifdef ENABLE_GTEST

#if _MSC_VER
        _CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
        //void *testWhetherMemoryLeakDetectionWorks = malloc(1);
#endif
        ::testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
#else
	testMinitaurLogging();
#endif


}