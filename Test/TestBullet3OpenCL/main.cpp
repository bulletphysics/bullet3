
#include <gtest/gtest.h>

#include "Bullet3Common/b3Logging.h"

void myerrorwarningprintf(const char* msg)
{
	//OutputDebugStringA(msg);
	printf(msg);
}

void myprintf(const char* msg)
{
	//OutputDebugStringA(msg);
	//printf(msg);
}

int gArgc=0;
char** gArgv=0;


int main(int argc, char **argv) {
#if _MSC_VER
        _CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
        //void *testWhetherMemoryLeakDetectionWorks = malloc(1);
#endif
        ::testing::InitGoogleTest(&argc, argv);

		gArgc = argc;
		gArgv = argv;

		b3SetCustomPrintfFunc(myprintf);
		b3SetCustomWarningMessageFunc(myerrorwarningprintf);
		b3SetCustomErrorMessageFunc(myerrorwarningprintf);

        return RUN_ALL_TESTS();
}
