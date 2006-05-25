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
	@file FUStringBuilder.h
	This file contains the FUStringBuilderT template class,
	its defined template classes and its helper classes.
*/

#ifndef _FCU_STRING_BUILDER_
#define _FCU_STRING_BUILDER_

/**
	A dynamically-sized string object.
	The template has two arguments: the character definition and
	the sprintf() functor class for float to string conversions.

	This class should be used for all the string operations, as it contains a
	dynamically-resized buffer that is not directly tied to its content's length.

	@ingroup FUtils
*/
template <class Char, class SPrintF>
class FUStringBuilderT
{
private:
	Char* buffer;
	size_t reserved;
	size_t size;

public:
	/** The standard string object which correspond to the builder. */
	typedef std::basic_string<Char> String;

	/** Creates a new builder with the content of the given string.
		@param sz A string. Its content will be copied within the builder. */
	FUStringBuilderT(const String& sz);

	/** Creates a new builder with the content of the given character array.
		@param sz A character array. Its content will be copied within the builder.
			It must terminate with an element containing the 'zero' value. */
	FUStringBuilderT(const Char* sz);

	/** Creates a new builder with an empty buffer. 
		@see reserve
		@param reserved The number of character slots to reserve within the empty buffer. */
	FUStringBuilderT(size_t reserved);

	/** Creates a new builder with an empty buffer. */
	FUStringBuilderT();

	/** Deletes the builder. Its buffer will be cleared. 
		Any pointers to its data will be dangling. */
	~FUStringBuilderT();

	/** Reserves a given number of character slots.
		If the builder has a buffer with a different number of character slots, a new
		buffer will be allocated. If the builder has contents, it will be copied within
		the new buffer. If there is more content than the new buffer can handle, it will
		be discarded.
		@param length The number of character slots to reserve. */
	void reserve(size_t length);

	/** Retrieves the length of the content within the builder.
		@return The length of the string. */
	inline size_t length() { return size; }

	/** Clears the content of the builder.
		This does not re-allocate a new buffer. */
	void clear();

	/** Retrieves whether the builder is empty.
		A builder is considered empty when it has no content, regardless of
		the size or allocation status of its buffer.
		@return Whether the builder is empty. */
	inline bool empty() { return size == 0; }

	/** Appends a character to the content of the builder.
		@param c A character. May not be the 'zero' value. */
	void append(Char c);

	/** Appends a string to the content of the builder.
		@param sz A string. */
	void append(const String& sz);

	/** Appends a character array to the content of the builder.
		@param sz A character array. It must terminate with an
			element containing the 'zero' value. */
	void append(const Char* sz);

	/** Appends the content of a builder to the content of this builder.
		@param b A string builder. */
	void append(const FUStringBuilderT& b);

	/** Appends the integer value, after converting it to a string,
		to the content of the builder.
		@param i An integer value. */
	void append(int32 i);
	void append(uint32 i); /**< See above. */
	void append(uint64 i); /**< See above. */

	inline void append(int i) { append((int32) i); } /**< See above. */
#ifdef _W64
	inline void append(_W64 unsigned int i) { append((uint32) i); } /**< See above. */
#else
	inline void append(unsigned int i) { append((uint32) i); } /**< See above. */
#endif 

	/** Appends the floating-point value, after converting it to a string,
		to the content of the builder. If the floating-point value is the special token
		that represents infinity, the string "INF" is appended. If it represents
		the negative infinity, the string "-INF" is appended. If it represents the
		impossibility, the string "NaN" is appended.
		@param f A floating-point value. */
	void append(float f);
	void append(double f); /**< See above. */

	/** Appends a value to the content of the builder.
		This is a shortcut for the append function.
		@see append
		@param val A value. This may be numerical, a character, a character array or a string. */
	template<typename TYPE> inline FUStringBuilderT& operator+=(const TYPE& val) { append(val); return *this; }

	/** Appends a character array to the content of the builder.
		A newline character will be appended after the character array.
		@param sz A character array. It must terminate with an
			element containing the 'zero' value. */
	void appendLine(const Char* sz);

	/** Removes a section of the content of the builder.
		Every character that occurs after the given index will be removed,
		resulting in a shrunk string.
		@param start An index within the content of the builder. */
	void remove(int32 start);

	/** Removes a section of the content of the builder.
		The substring defined by the 'start' and 'end' indices will be
		removed. The 'start' character is removed and is replaced by
		the 'end' character.
		@param start The index of the first character of the substring to remove.
		@param end The index of the first character after the removed substring. */
	void remove(int32 start, int32 end);

	/** Removes the last character of the content of the builder. */
	inline void pop_back() { if (size > 0) --size; }

	/** Sets the content of the builder to a given value.
		This clears the builder of all its content and appends the given value.
		@param val A value. This may be numerical, a character, a character array or a string. */
	template<typename TYPE> inline void set(const TYPE& val) { clear(); append(val); }
	template<typename TYPE> inline FUStringBuilderT& operator=(const TYPE& val) { clear(); append(val); return *this; } /**< See above. */

	/** Converts the content of the builder to a standard string.
		@return A string with the content of the builder. */
	String ToString();
	operator String() { return ToString(); } /**< See above. */

	/** Converts the content of the builder to a character array.
		@return A character array with the content of the builder.
			This pointer is valid for the lifetime of the buffer of the builder, so
			do not keep it around. This character array should not be modified. */
	const Char* ToCharPtr();
	operator const Char*() { return ToCharPtr(); } /**< See above. */

	/** Retrieves the index of the first character within the content of the builder
		that is equivalent to the given character.
		@param c The character to match.
		@return The index of the first equivalent character. -1 is returned if no
			character matches the given character. */
	int32 index(Char c);

	/** Retrieves the index of the last character within the content of the builder
		that is equivalent to the given character.
		@param c The character to match.
		@return The index of the last equivalent character. -1 is returned if no
			character matches the given character. */
	int32 rindex(Char c);

private:
	void enlarge(size_t minimum);
};

/**
	Encapsulates the 8-bit string numerical conversion functions.
	The 'snprintf' function is used so no locale information is handled.
*/
class SprintF
{
public:
	/** Converts a signed integer into the given constant-sized string.
		@param output A constant-sized string.
		@param length The size of the constant-sized string.
		@param i A signed integer. */
	void PrintInt32(char* output, uint32 length, int32 i) { snprintf(output, length, "%i", i); } 

	/** Converts an unsigned integer into the given constant-sized string.
		@param output A constant-sized string.
		@param length The size of the constant-sized string.
		@param i An unsigned integer. */
	void PrintUInt32(char* output, uint32 length, uint32 i) { snprintf(output, length, "%u", i); }
	void PrintUInt64(char* output, uint32 length, uint64 i) { snprintf(output, length, "%u", i); } /**< See above. */

	/** Converts a floating-point value into the given constant-sized string.
		@param output A constant-sized string.
		@param length The size of the constant-sized string.
		@param f A floating-point value. */
	void PrintFloat(char* output, uint32 length, double f) { snprintf(output, length, "%f", f); }

	/** Retrieves the length of a constant-sized string.
		@param in A character array which is terminated with a 'zero' element.
		@return The number of element preceeding the 'zero' element. */
	size_t StrLen(const char* in) { return strlen(in); }
};

/**
	Encapsulates the Unicode string numerical conversion functions.
	The 'fsnprintf' function is used so no locale information is handled.
*/
class SFprintF
{
public:
	/** Converts a signed integer into the given constant-sized string.
		@param output A constant-sized string.
		@param length The size of the constant-sized string.
		@param i A signed integer. */
	void PrintInt32(fchar* output, uint32 length, int32 i) { fsnprintf(output, length, FC("%i"), i); }

	/** Converts an unsigned integer into the given constant-sized string.
		@param output A constant-sized string.
		@param length The size of the constant-sized string.
		@param i An unsigned integer. */
	void PrintUInt32(fchar* output, uint32 length, uint32 i) { fsnprintf(output, length, FC("%u"), i); }

	/** Converts an unsigned integer into the given constant-sized string.
		@param output A constant-sized string.
		@param length The size of the constant-sized string.
		@param i An unsigned integer. */
	void PrintUInt64(fchar* output, uint32 length, uint64 i) { fsnprintf(output, length, FC("%u"), i); }

	/** Converts a floating-point value into the given constant-sized string.
		@param output A constant-sized string.
		@param length The size of the constant-sized string.
		@param f A floating-point value. */
	void PrintFloat(fchar* output, uint32 length, double f) { fsnprintf(output, length, FC("%f"), f); }

	/** Retrieves the length of a constant-sized string.
		@param in A character array which is terminated with a 'zero' element.
		@return The number of element preceeding the 'zero' element. */
	size_t StrLen(const fchar* in) { return fstrlen(in); }
};

typedef FUStringBuilderT<fchar, SFprintF> FUStringBuilder; /**< A Unicode string builder. */
typedef FUStringBuilderT<char, SprintF> FUSStringBuilder;  /**< A 8-bit string builder. */

/** Declares a global Unicode string builder.
	As many functions within FCollada use the global string builders, their content is often overwritten.
	Use this builder only for quick conversion or character accumulation. */
FCOLLADA_EXPORT extern FUStringBuilder globalBuilder; 

/** Declares a global 8-bit string builder.
	As many functions within FCollada use the global string builders, their content is often overwritten.
	Use this builder only for quick conversion or character accumulation. */
FCOLLADA_EXPORT extern FUSStringBuilder globalSBuilder;

#include "FUtils/FUStringBuilder.hpp"

#endif // _FCU_STRING_BUILDER_

