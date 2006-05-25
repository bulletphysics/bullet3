/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FUAssert.h
	This file contains a simple debugging assertion mechanism.
*/

#ifndef _FU_ASSERT_H_
#define _FU_ASSERT_H_

/**
	Breaks into the debugger.
	In debug builds, this intentionally crashes the application.
	In release builds, this is an empty function.
*/
inline void DEBUGGER_BREAK()
{
#ifdef _DEBUG
#ifdef WIN32

	_asm int 3;
#else

	// Force a crash?
	uint32* __p = NULL;
	uint32 __v = *__p;
	*__p = 0xD1ED0D1E;
	__v = __v + 1;

#endif // WIN32
#endif // _DEBUG
}

/** Forces the debugger to break, or take the fall-back.
	@param command The fall_back command to execute. */
#define FUFail(command) { DEBUGGER_BREAK(); command; }

/** Asserts that a condition is met.
	Use this macro, instead of 'if' statements
	when you are asserting for a programmer's error.
	@param condition The condition to assert.
	@param fall_back The command to execute if the condition is not met. */
#define FUAssert(condition, fall_back) { if (!(condition)) { DEBUGGER_BREAK(); fall_back; } }

#endif // _FU_ASSERT_H_
