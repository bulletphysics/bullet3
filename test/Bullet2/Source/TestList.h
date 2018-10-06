//
//  TestList.h
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//

#ifndef BulletTest_TestList_h
#define BulletTest_TestList_h

#ifdef __cplusplus
extern "C"
{
#endif

	typedef struct TestDesc
	{
		const char *name;
		int (*test_func)(void);  // return 0 for success, non-zero for failure
	} TestDesc;

	extern TestDesc gTestList[];

#ifdef __cplusplus
}
#endif

#endif
