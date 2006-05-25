/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	Available only to licensees.
	Distribution of this file or its content is strictly prohibited.
*/

#ifndef _FC_TEST_H_
#define _FC_TEST_H_

inline void PassIf(bool condition)
{
	if (!condition)
	{
		FUFail(exit(-1));
	}
}
inline void FailIf(bool condition) { return PassIf(!condition); }
inline void CheckStatus(const FUStatus& s) { PassIf(s.IsSuccessful()); }

#endif // _FC_TEST_H_