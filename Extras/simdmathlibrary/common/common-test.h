/* Header file for common parts of the testsuite
   Copyright (C) 2006, 2007 Sony Computer Entertainment Inc.
   All rights reserved.

   Redistribution and use in source and binary forms,
   with or without modification, are permitted provided that the
   following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Sony Computer Entertainment Inc nor the names
      of its contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
 */


#include <stdio.h>

#ifdef __SPU__

#include <spu_intrinsics.h>

static inline unsigned long long clock_start(void)
{
	spu_writech(SPU_WrDec, 0);
	return -spu_readch(SPU_RdDec);
}

static inline unsigned long long clock_stop(void)
{
	return -spu_readch(SPU_RdDec);
}

#else

static inline unsigned long long clock(void)
{
        unsigned long long ret;
	/* This need to be fixed for the hardware errata.  */
        __asm __volatile__ ( "mftb %0\n"
                : "=r" (ret)
		:
    		: "memory");
        return (ret);
}

static inline unsigned long long clock_start(void)
{
	return clock();
}

static inline unsigned long long clock_stop(void)
{
	return clock();
}

#endif

// Test files begin with TEST_SET_START("your initials","test set description")
// Individual tests begin with TEST_START("name of test")
// and end with TEST_PASS(), TEST_FAIL("reason for failure") or TEST_CHECK(<test to evaluate>)
// Or you can run a test encapsulated in a function with:
// TEST_FUNCTION("name of test", function(), "reason for failure")
//
// The clock starts when you call TEST_START and stops with TEST_PASS, TEST_FAIL or TEST_CHECK
// After a start there can be several PASS, FAIL or CHECK calls, each one counts as a test, time is measured from the prior call
//
char 
  *__initials,       // Test owner's initials
  *__description,    // short descriptive name for this test set
  *__name,           // name of the currently running test
  *__set_id;         // id of the the test set
int
//  __zip=0,
  __success=1,       // set to 0 if any tests failed 
  __count,           // Total number of tests run 
  __passed;          // Total number of tests passed
unsigned long long
  __ttemp,
  __time,            // For timing tests (usually start time of last test)
  __ttime;           // Cumulative test runtime NOT counting runtime of the TEST macros

// TEST_SET_START
// Call at the start of a set of related tests to identify them
// Prints a "start of set banner message"
// set_id - unique test set identifyer a time in the format yyyymmddhhmmss followed by your initials ie: 20040716104615GAC
// initials - your initials
// description - brief descriptive name for this test set
#define TEST_SET_START(set_id,initials,description) \
  do { \
    __set_id=set_id; \
    __initials=initials; \
    __description=description; \
    __count=0;  \
    __passed=0; \
    __ttime=0; \
    printf("0\t%s\t%d\t%s\tSTART\tpassed\ttotal\ttime\t%s\tunique test id   \t%s\n",__FILE__,__LINE__,__initials,__set_id, __description); \
    __time = clock_start(); \
  } while(0)

// TEST_START
// Begins a test, and starts the clock
// name - brief name for this test
#define TEST_START(name) \
  do { \
    (void)clock_stop(); \
    __name=name; \
    __time = clock_start(); \
     } while(0)

// TEST_PASS
// Indicates the test passed
// test_id - unique test ID number, same format as the set_id number
// This should match the id provided to the matching TEST_FAIL call
#define TEST_PASS(test_id) \
  do { \
    __ttemp=clock_stop(); \
    __time=__ttemp-__time; \
    __ttime+=__time; \
    __count++; \
    __passed++; \
    printf("1\t%s\t%d\t%s\tPASS\t%d\t%d\t%lld\t%s\t%s\t%s\n",__FILE__,__LINE__,__initials,__passed,__count,__time,__set_id,test_id,__name); \
    __time=clock_start(); \
  } while(0)

// TEST_FAIL
// Indicates the test failed
// test_id - unique test ID number, same format as the set_id number
// This should match the id provided to the matching TEST_PASS call
// why - brief description of why it failed
#define TEST_FAIL(test_id,why,error_code)    \
  do { \
    __ttemp=clock_stop(); \
    __time=__ttemp-__time; \
    __ttime+=__time; \
    __count++; \
    __success=0; \
    printf("1\t%s\t%d\t%s\tFAIL\t%d\t%d\t%lld\t%s\t%s\t%s\tFAILED BECAUSE: %s\t%d\n",__FILE__,__LINE__,__initials,__passed,__count,__time,__set_id,test_id,__name,why,error_code); \
    __time=clock_start(); \
  } while(0)

// TEST_CHECK
// Passes or fails the test after evaluating the "test" argument (just like assert but without terminating the program)
// The clock is immediately stopped so the time required to evaluate "test" will NOT be included in the reported time
// If the test failed, the reason will be printed as FAILED BECAUSE: check (value of "test") failed
// test_id - unique test ID number, same format as the set_id number
// test - expression evaluating to true/false
#define TEST_CHECK(test_id,test,error_code)  \
  do { \
    __ttemp=clock_stop(); \
    __time=__ttemp-__time; \
    __ttime+=__time; \
    __count++; \
    if(test)  \
    { \
      __passed++; \
      printf("1\t%s\t%d\t%s\tPASS\t%d\t%d\t%lld\t%s\t%s\t%s\n",__FILE__,__LINE__,__initials,__passed,__count,__time,__set_id,test_id,__name); \
    } \
    else  \
    {  \
    __success=0; \
      printf("1\t%s\t%d\t%s\tFAIL\t%d\t%d\t%lld\t%s\t%s\t%s\tFAILED BECAUSE: check %s failed\t%d\n",__FILE__,__LINE__,__initials,__passed,__count,__time,__set_id,test_id,__name,#test,error_code); \
    } \
    __time=clock_start(); \
  } while(0) 

// TEST_FUNCTION
// Runs a test encapsulated in a function that returns 0 if the test passed and an error number if it failed
// The clock is started on calling the function and stopped as soon as it returns so the branching logic will not be included in the time
// test_id - unique test ID number, same format as the set_id number
// name - brief name for the test
// func - function invocation (should include parenthesis, may have arguments)
// why  - brief description to print if the test fails
#define TEST_FUNCTION(test_id,name,func,why)   \
  do {  \
    TEST_START(name);  \
    int result=func; \
    __ttemp=clock_stop(); \
    __time=__ttemp-__time; \
    __ttime+=__time; \
    __count++; \
    if(result==0)  \
    { \
      __passed++; \
      printf("1\t%s\t%d\t%s\tPASS\t%d\t%d\t%d\t%s\t%s\t%s\n",__FILE__,__LINE__,__initials,__passed,__count,__time,__set_id,test_id,__name); \
    } \
    else  \
    {  \
    __success=0; \
      printf("1\t%s\t%d\t%s\tFAIL\t%d\t%d\t%d\t%s\t%s\t%s\tFAILED BECAUSE: %s\t%d\n",__FILE__,__LINE__,__initials,__passed,__count,__time,__set_id,test_id,__name,why,result); \
    } \
    __time=clock_start(); \
  } while(0)

// TEST_SET_DONE
// Ends a set of tests, prints out the closing banner (OK if all tests pass, PROBLEM if any fail)
// Also prints count of tests passed, tests run and total time
#define TEST_SET_DONE()   \
  do { \
    printf("9\t%s\t%d\t%s\t%s\t%d\t%d\t%lld\t%s\tunique test id   \t%s\n",__FILE__,__LINE__,__initials,(__count==__passed)?"OK":"PROBLEM",__passed,__count,__ttime,__set_id,__description); \
  } while(0)

// TEST_EXIT
// Call this ONCE at the very end of the test program, it calls "exit" to return
// EXIT_SUCCESS if all tests passed or EXIT_FAILURE if any tests failed.
// This allows the makefile/shell script running the tests to know which ones failed
#define TEST_EXIT() \
  do { \
    printf("FINISHED!\n"); \
    if(__success) \
      exit(0); \
    else \
      exit(-1); \
  } while (0)
