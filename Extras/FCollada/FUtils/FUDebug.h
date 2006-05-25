/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FUDebug.h
	This file contains macros useful to write debugging output.
*/

#ifndef _FU_DEBUG_H_
#define _FU_DEBUG_H_

/** Outputs a string to the debug monitor.
	@param token The string to output. */
#define DEBUG_OUT(token) ::DebugOut(__FILE__, __LINE__, token);

/** Outputs a string to the debug monitor.
	@param token The formatted string to output.
	@param arg1 A first argument. */
#define DEBUG_OUT1(token, arg1) ::DebugOut(__FILE__, __LINE__, token, arg1);

/** Outputs a string to the debug monitor.
	@param token The formatted string to output.
	@param arg1 A first argument.
	@param arg2 A second argument. */
#define DEBUG_OUT2(token, arg1, arg2) ::DebugOut(__FILE__, __LINE__, token, arg1, arg2);

#ifndef _DEBUG

	/**	Outputs a string to the debug monitor.
		The formatted message is the first parameter. */
	inline void DebugOut(const char*, ...) {}
#ifdef UNICODE
	inline void DebugOut(const fchar*, ...) {} /**< See above. */
#endif // UNICODE

	/**	Outputs a string to the debug monitor.
		The formatted message is the first parameter.
		The second parameter is the variable parmeter list. */
	inline void DebugOutV(const char*, va_list&) {}
#ifdef UNICODE
	inline void DebugOutV(const fchar*, va_list&) {} /**< See above. */
#endif // UNICODE

	/**	Outputs a string to the debug monitor.
		The filename and line number are the first two parameters.
		The formatted message is the third parameter. */
	inline void DebugOut(const char*, uint32, const char*, ...) {}
#ifdef UNICODE
	inline void DebugOut(const char*, uint32, const fchar*, ...) {} /**< See above. */
#endif // UNICODE

	/**	Outputs a string to the debug monitor.
		The filename and line number are the first two parameters.
		The formatted message is the third parameter.
		The fourth parameter is the variable parameter list. */
	inline void DebugOutV(const char*, uint32, const char*, va_list&) {}
#ifdef UNICODE
	inline void DebugOutV(const char*, uint32, const fchar*, va_list&) {} /**< See above. */
#endif // UNICODE

#else // _DEBUG

#ifdef UNICODE
	void FCOLLADA_EXPORT DebugOut(const char* filename, uint32 line, const fchar* message, ...);
	void FCOLLADA_EXPORT DebugOut(const fchar* message, ...);
	void FCOLLADA_EXPORT DebugOutV(const char* filename, uint32 line, const fchar* message, va_list& vars);
	void FCOLLADA_EXPORT DebugOutV(const fchar* message, va_list& vars);
#endif // UNICODE
	void FCOLLADA_EXPORT DebugOut(const char* filename, uint32 line, const char* message, ...);
	void FCOLLADA_EXPORT DebugOut(const char* message, ...);
	void FCOLLADA_EXPORT DebugOutV(const char* filename, uint32 line, const char* message, va_list& vars);
	void FCOLLADA_EXPORT DebugOutV(const char* message, va_list& vars);

#endif // _DEBUG

#endif // _FU_DEBUG_H_

