//
//  main.c
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//
#include <stdio.h>
#ifdef __APPLE__
#include <libgen.h>
#endif //__APPLE__

#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include "main.h"
#include "Utils.h"
#include "TestList.h"
#include "LinearMath/btScalar.h"

#if defined (BT_USE_NEON) || defined (BT_USE_SSE_IN_API)

#ifdef _WIN32
#define strcasecmp _stricmp
#define basename(A) A
#endif

#define EXIT_NO_ERROR INT_MIN

//int gReportNanoseconds = 0;    // in Utils.c

int gReportAverageTimes = 0;
int gExitOnError = 0;
char *gFullPath = NULL;
const char *gAppName = NULL;
int gArgc;
const char **gArgv;

typedef struct TestNode
{
    struct TestNode *next;
    const char      *name;
}TestNode;

TestNode *gNodeList = NULL;

static int ParseArgs( int argc, const char *argv[] );
static void PrintUsage( void );
static int Init( void );
static void ListTests(void );

const char *gArch = 
#ifdef __i386__ 
    "i386";
#elif defined __x86_64__
    "x86_64";
#elif defined __arm__
    "arm";
#elif defined _WIN64
	"win64";
#elif defined _WIN32
	"win32";
#else
    #error unknown arch
#endif





#include <stdio.h>

int main (int argc, const char * argv[])
{

   // Enable just one test programatically (instead of command-line param)
   // TestNode *node = (TestNode*) malloc( sizeof( TestNode ) );
   // node->name = "btDbvt";
   // node->next = 0;
   // gNodeList = node;
    
	srand(0.f);

    int numPassedTests=0;
	int numFailedTests= 0;

    int err;
    
    // Parse arguments. Build gNodeList.
    if( (err = ParseArgs( argc, argv ) ) )
    {
        if( EXIT_NO_ERROR == err )
            return 0;
        
        PrintUsage();        
        return err;
    }
    
    printf("Arch: %s\n", gArch );
    
    if( gReportAverageTimes )
        printf( "Reporting average times.\n" );
    else
        printf( "Reporting best times.\n" );
    
    // Set a few things up
    if( (err = Init() ))
    {
        printf( "Init failed.\n" );
        return err;
    }
    
    if( NULL == gNodeList )
    { // test everything
        printf( "No function list found. Testing everything...\n" );
        size_t i;
        for( i = 0; NULL != gTestList[i].test_func; i++ )
        {
            printf( "\n----------------------------------------------\n" );
            printf( "Testing %s:\n", gTestList[i].name );
            printf( "----------------------------------------------\n" );
            uint64_t startTime = ReadTicks();
            int local_error = gTestList[i].test_func();
            uint64_t currentTime = ReadTicks() - startTime;
            if( local_error )
            {
				numFailedTests++;
                printf( "*** %s test failed with error: %d\n", gTestList[i].name, local_error );
                if( gExitOnError )
                    return local_error;
                if( 0 == err )
                    err = local_error;
            }
            else
			{
				numPassedTests++;
                printf("%s Passed.\t\t\t(%2.2gs)\n", gTestList[i].name, TicksToSeconds(currentTime));
			}
        }
    }
    else
    { // test just the list
        while( NULL != gNodeList )
        {
            TestNode *currentNode = gNodeList;
            gNodeList = gNodeList->next;
            
            // Find the test with that name
            size_t i;
            for( i = 0; NULL != gTestList[i].test_func; i++ )
                if( 0 == strcasecmp( currentNode->name, gTestList[i].name ) )
                    break;
            
            if( NULL != gTestList[i].test_func )
            {
                printf( "\n----------------------------------------------\n" );
                printf( "Testing %s:\n", gTestList[i].name );
                printf( "----------------------------------------------\n" );
                uint64_t startTime = ReadTicks();
                int local_error = gTestList[i].test_func();
                uint64_t currentTime = ReadTicks() - startTime;
                if( local_error )
                {
					numFailedTests++;
                    printf( "*** %s test failed with error: %d\n", gTestList[i].name, local_error );
                    if( gExitOnError )
                        return local_error;
                    if( 0 == err )
                        err = local_error;
                }
                else
				{
					numPassedTests++;
                    printf("%s Passed.\t\t\t(%2.2gs)\n", gTestList[i].name, TicksToSeconds(currentTime));
				}
            }
            else
            {
                printf( "\n***Error: Test name \"%s\" not found! Skipping.\n", currentNode->name );
                err = -1;
                if( gExitOnError )
                    return -1;
            }
            
            free( currentNode );
        }
    }
	printf( "\n----------------------------------------------\n" );
	printf("numPassedTests = %d, numFailedTests = %d\n",numPassedTests,numFailedTests);
    
    free(gFullPath);
    return err;
}

static int Init( void )
{
    // init the timer
    TicksToCycles(0);
    
    return 0;
}

static int ParseArgs( int argc, const char *argv[] )
{
    int listTests = 0;
    TestNode *list = NULL;
    
    gArgc = argc;
    gArgv = argv;
    gFullPath = (char*)malloc( strlen(argv[0]) + 1);
    strcpy(gFullPath, argv[0]);
    gAppName = basename( gFullPath );
    if( NULL == gAppName )
        gAppName = "<unknown app name>";
    
    printf( "%s ", gAppName );
    int skipremaining=0;
    
    size_t i;
    for( i = 1; i < argc; i++ )
    {
        const char *arg = argv[i];
        printf( "\t%s", arg );
        if( arg[0] == '-' )
        {
            arg++;
            while( arg[0] != '\0' )
            {
                int stop = 0;
                switch( arg[0] )
                {
                    case 'a':
                        gReportAverageTimes ^= 1;
                        break;
                    case 'e':
                        gExitOnError ^= 1;
                        break;
                    case 'h':
                        PrintUsage();
                        return EXIT_NO_ERROR;
                    case 'l':
                        listTests ^= 1;
                        return EXIT_NO_ERROR;
                    case 's':
                        gReportNanoseconds ^= 1;
                        break;
                    case ' ':
                        stop = 1;
                        break;
                    case 'N'://ignore the -NSDocumentRevisionsDebugMode argument from XCode 4.3.2
                        skipremaining = 1;
                       stop = 1;
                        break;
                    default:
                        printf( "\nError: Unknown flag \'%c\'\n", arg[0] );
                        return -1;
                }
                if( stop )
                    break;
                arg++;
            }
        }
        else
        { // add function name to the list
            TestNode *node = (TestNode*) malloc( sizeof( TestNode ) );
            node->name = arg;
            node->next = list;
            list = node;
        }
        if (skipremaining)
            break;
    }
    
    // reverse the list of test names, and stick on gNodeList
    while( list )
    {
        TestNode *node = list;
        TestNode *next = node->next;
        node->next = gNodeList;
        gNodeList = node;
        list = next;
    }
    
    printf( "\n" );
    if( listTests )
        ListTests();
    
    return 0;
}


static void PrintUsage( void )
{
    printf("\nUsage:\n" );
    printf("%s: <-aehls> <test names>", gAppName);
    printf("Options:\n");
    printf("\t-a\tToggle report average times vs. best times. (Default: best times)\n");
    printf("\t-e\tToggle exit immediately on error behavior. (Default: off)\n");
    printf("\t-h\tPrint this message.\n");
    printf("\t-l\tToggle list available test names.  (Default: off)\n");
    printf("\t-s\tToggle report times in cycles or nanoseconds. (Default: cycles)\n\n");
    printf("\tOptions may be followed by one or more test names. If no test names \n" );
    printf("\tare provided, then all tests are run.\n\n");
}

static void ListTests(void )
{
    size_t i;
    
    printf("\nTests:\n");
    for( i = 0; NULL != gTestList[i].test_func; i++ )
    {
        printf( "%19s", gTestList[i].name );
        if( NULL != gTestList[i].test_func )
            printf( "," );
        if( 3 == (i&3) )
            printf( "\n" );
    }
}
#else
#include <stdio.h>
int main(int argc, char* argv[])
{
	printf("error: no SIMD enabled through BT_USE_NEON or BT_USE_SSE_IN_API \n(enable in LinearMath/btScalar.h or through build system)\n");
	return 0;
}
#endif
