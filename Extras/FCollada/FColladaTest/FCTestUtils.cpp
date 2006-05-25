/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	Available only to licensees.
	Distribution of this file or its content is strictly prohibited.
*/

#include "StdAfx.h"
#include "FUtils/FUUniqueStringMap.h"

void TestUniqueStringMap()
{
#define VERIFY_UNIQUE(c) { for (StringList::iterator itN = uniqueNames.begin(); itN != uniqueNames.end(); ++itN) { FailIf((*itN) == (c)); } }
	StringList uniqueNames;

	FUSUniqueStringMap names;
	FailIf(names.Exists("Test"));
	FailIf(names.Exists("Test0"));
	FailIf(names.Exists("Test1"));
	FailIf(names.Exists("Test2"));

	// Add a first name: should always be unique
	string name1 = "Test";
	names.AddUniqueString(name1);
	PassIf(names.Exists("Test"));
	PassIf(name1 == "Test");
	uniqueNames.push_back(name1);

	// Add a second name that should also be unique
	string name2 = "Glad";
	names.AddUniqueString(name2);
	PassIf(names.Exists("Glad"));
	PassIf(name2 == "Glad");
	uniqueNames.push_back(name2);

	// Add the first name a couple more times
	string name3 = name1;
	names.AddUniqueString(name3);
	PassIf(names.Exists(name3));
	VERIFY_UNIQUE(name3);
	uniqueNames.push_back(name3);

	name3 = name1;
	names.AddUniqueString(name3);
	PassIf(names.Exists(name3));
	VERIFY_UNIQUE(name3);
	uniqueNames.push_back(name3);

	// Add the second name a couple more times
	name3 = name2;
	names.AddUniqueString(name3);
	PassIf(names.Exists(name3));
	VERIFY_UNIQUE(name3);
	uniqueNames.push_back(name3);

	name3 = name2;
	names.AddUniqueString(name3);
	PassIf(names.Exists(name3));
	VERIFY_UNIQUE(name3);
	uniqueNames.push_back(name3);

	// There should now be 6 unique names, so pick some randomly and verify that we're always getting more unique names.
	for (int32 i = 0; i < 5; ++i)
	{
		int32 index = rand() % ((int32)uniqueNames.size());
		name3 = uniqueNames[index];
		names.AddUniqueString(name3);
		PassIf(names.Exists(name3));
		VERIFY_UNIQUE(name3);
		uniqueNames.push_back(name3);
	}
}