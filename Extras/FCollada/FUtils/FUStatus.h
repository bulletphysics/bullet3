/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/
/*
	Based on the FS Import classes:
	Copyright (C) 2005-2006 Feeling Software Inc
	Copyright (C) 2005-2006 Autodesk Media Entertainment
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FUStatus.h
	This file contains the FUStatus class.
*/

#ifndef _FU_STATUS_H_
#define _FU_STATUS_H_

/**
	A return status structure that includes a string.
	Contains a return code as well as a string which explains the errors and warnings
	attached to the return code. The return code is either 'successful' or 'failure'.
	If the return code is 'failure', proceeding normally may be dangerous.
	This structure is used by the FCDocument classes to explain parsing errors.

	@ingroup FUtils
*/
class FCOLLADA_EXPORT FUStatus
{
private:
	fstring errorString;
	bool callSuccessful;

public:
	/** Default Constructor.
		The default return code is 'successful'.
		@param _callSuccessful Whether the return code should be set to 'successful'. */
	FUStatus(bool _callSuccessful=true) { callSuccessful = _callSuccessful; }

	/** Sets the value of the return code to 'failure'.
		@return The error code structure. Returns itself to support
			multiple subsequent operations in one statement. */
	FUStatus& Fail() { callSuccessful = false; return *this; }

	/** Sets the value of the return code to 'failure'.
		@param str The error string for the error encountered.
		@param line A line number.
		@return The return status structure. Returns itself to support
			multiple subsequent operations in one statement. */
	FUStatus& Fail(const fstring& str, size_t line=0) { callSuccessful = false; AppendString(FS("ERROR[") + TO_FSTRING(line) + FS("]: ") + str); return *this; }
	FUStatus& Fail(const fchar* str, size_t line=0) { callSuccessful = false; AppendString(FS("ERROR[") + TO_FSTRING(line) + FS("]: ") + str); return *this; } /**< See above. */

	/** Adds a warning to the return status. A warning does not set the return code to 'failure'.
		@param str The warning string for the error encountered.
		@param line A line number.
		@return The return status structure. Returns itself to support
			multiple subsequent operations in one statement. */
	FUStatus& Warning(const fstring& str, size_t line=0) { AppendString(FS("WARNING[") + TO_FSTRING(line) + FS("]: ") + str); return *this; }
	FUStatus& Warning(const fchar* str, size_t line=0) { AppendString(FS("WARNING[") + TO_FSTRING(line) + FS("]: ") + str); return *this; } /**< See above. */

	/** Merges two return stati together.
		Appends to this return status the error string from the given status.
		The merged return code is 'failure', if either return codes are 'failure'.
		@param a The return status to merge with. */
	void AppendStatus(const FUStatus& a) { AppendString(a.errorString); callSuccessful &= a.callSuccessful; }

	/** Appends a return string to the return status.
		A 'newline' code is added between return strings.
		@param str The return string to append. */
	void AppendString(const fstring& str) { AppendString(str.c_str()); }
	void AppendString(const fchar* str) { if (*str != 0) { if (!errorString.empty()) errorString += FC("\r\n"); errorString += str; } } /**< See above. */

	/** Retrieves the error/return string for this return status.
		@return The return string. */
	const fchar* GetErrorString() const { return errorString.empty() ? (callSuccessful ? FC("Success") : FC("Failure")) : errorString.c_str(); }

	/** Retrieves whether the return status is 'successful'.
		@return Whether the return status is 'successful'. */
	bool IsSuccessful() const { return callSuccessful; }

	/** Retrieves whether the return status is 'failure'.
		@return Whether the return status is 'failure'. */
	bool IsFailure() const { return !callSuccessful; }

	/** Transforms the return status into a boolean primitive.
		@return Whether the return status is 'successful'. */
	operator bool() const { return callSuccessful; }
};

#endif // _FU_STATUS_H_
