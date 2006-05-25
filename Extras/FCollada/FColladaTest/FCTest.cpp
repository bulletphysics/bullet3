/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	Available only to licensees.
	Distribution of this file or its content is strictly prohibited.
*/

#include "StdAfx.h"
#include <direct.h>

int _tmain(int UNUSED(argc), TCHAR* UNUSED(argv)[])
{
	// Set the current folder to the folder with the samples DAE files
	chdir("Samples\\");

	// First, declare all the test functions.
#define FCTEST(test_fn) extern void test_fn()
#include "FCTestList.h"
#undef FCTEST

	// Second, call them one at a time.
#define FCTEST(test_fn) test_fn()
#include "FCTestList.h"
#undef FCTEST
}