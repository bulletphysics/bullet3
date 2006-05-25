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
	@file FUStringConversion.h
	This file contains the FUStringConversion class.
*/

#ifndef _FCU_STRING_CONVERSION_
#define _FCU_STRING_CONVERSION_

class FUDateTime;

/**
	Common string conversion.

	This static class contains the parsing function for Unicode and 8-bit/UTF-8
	strings into common data types: integers, booleans, floating-point values,
	vectors, matrices, date-time, etc. and dynamically-sized array of these types.

	This class can also convert common data types into an 8-bit or a Unicode string and 
	it contains conversion functions to convert string between 8-bit and Unicode.

	@ingroup FUtils
*/
class FCOLLADA_EXPORT FUStringConversion
{
private: FUStringConversion() {}
public:
	/** Converts a 8-bit string to a Unicode string.
		@param value The 8-bit string.
		@return The converted Unicode string. */
	static fstring ToFString(const char* value);
	inline static fstring ToFString(const string& value) { return ToFString(value.c_str()); } /**< See above. */

	/** Converts an Unicode string to a 8-bit string.
		@param value The Unicode string.
		@return The converted 8-bit string. */
	static string ToString(const fchar* value);
	inline static string ToString(const fstring& value) { return ToString(value.c_str()); } /**< See above. */

	/** Parses a string into a boolean value.
		@param value The string.
		@return The parsed boolean value. */
	static bool ToBoolean(const char* value);
	inline static bool ToBoolean(const string& value) { return ToBoolean(value.c_str()); } /**< See above. */
#ifdef UNICODE
	static bool ToBoolean(const fchar* value); /**< See above. */
	inline static bool ToBoolean(const fstring& value) { return ToBoolean(value.c_str()); } /**< See above. */
#endif

	/** Parses a string into a floating-point value.
		@param value The string. For the string pointer versions of this function,
			the pointer will point to the last processed characters after the parsing.
		@return The parsed floating-point value. */
	static float ToFloat(const char** value);
	inline static float ToFloat(const char* value) { return ToFloat(&value); } /**< See above. */
	inline static float ToFloat(const string& value) { return ToFloat(value.c_str()); } /**< See above. */
#ifdef UNICODE
	static float ToFloat(const fchar** value); /**< See above. */
	inline static float ToFloat(const fstring& value) { return ToFloat(value.c_str()); } /**< See above. */
	inline static float ToFloat(const fchar* value) { return ToFloat(&value); } /**< See above. */
#endif

	/** Parses a string into a signed integer.
		@param value The string. For the string pointer versions of this function,
			the pointer will point to the last processed characters after the parsing.
		@return The parsed signed integer. */
	static int32 ToInt32(const char** value);
	inline static int32 ToInt32(const char* value) { return ToInt32(&value); } /**< See above. */
	inline static int32 ToInt32(const string& value) { return ToInt32(value.c_str()); } /**< See above. */
#ifdef UNICODE
	static int32 ToInt32(const fchar** value); /**< See above. */
	inline static int32 ToInt32(const fchar* value) { return ToInt32(&value); } /**< See above. */
	inline static int32 ToInt32(const fstring& value) { return ToInt32(value.c_str()); } /**< See above. */
#endif

	/** Parses a string into an unsigned integer.
		@param value The string. For the string pointer versions of this function,
			the pointer will point to the last processed characters after the parsing.
		@return The parsed unsigned integer. */
	static uint32 ToUInt32(const char** value);
	inline static uint32 ToUInt32(const char* value) { return ToUInt32(&value); } /**< See above. */
	inline static uint32 ToUInt32(const string& value) { return ToUInt32(value.c_str()); } /**< See above. */
#ifdef UNICODE
	static uint32 ToUInt32(const fchar** value); /**< See above. */
	inline static uint32 ToUInt32(const fchar* value) { return ToUInt32(&value); } /**< See above. */
	inline static uint32 ToUInt32(const fstring& value) { return ToUInt32(value.c_str()); } /**< See above. */
#endif

	/** Parses a string into an unsigned integer. The string is assumed to have
		an unsigned integer in hexadecimal format.
		@param value The string. For the string pointer versions of this function,
			the pointer will point to the last processed characters after the parsing.
		@param count The maxmimum number of characters to parse.
			For example, a count of 2 will read in an 8-bit character.
		@return The parsed unsigned integer. */
	static uint32 HexToUInt32(const char** value, uint32 count=UINT_MAX);
	inline static uint32 HexToUInt32(const char* value, uint32 count=UINT_MAX) { return HexToUInt32(&value, count); } /**< See above. */
	inline static uint32 HexToUInt32(const string& value, uint32 count=UINT_MAX) { return HexToUInt32(value.c_str(), count); } /**< See above. */
#ifdef UNICODE
	static uint32 HexToUInt32(const fchar** value, uint32 count=UINT_MAX); /**< See above. */
	inline static uint32 HexToUInt32(const fchar* value, uint32 count=UINT_MAX) { return HexToUInt32(&value, count); } /**< See above. */
	inline static uint32 HexToUInt32(const fstring& value, uint32 count=UINT_MAX) { return HexToUInt32(value.c_str(), count); } /**< See above. */
#endif

	/** Parses a string into a 3D vector.
		@param value The string. For the string pointer versions of this function,
			the pointer will point to the last processed characters after the parsing.
		@param lengthFactor An optional factor that will scale the 3D vector.
		@return The parsed 3D vector. */
	static FMVector3 ToPoint(const char** value, float lengthFactor=1.0f);
	inline static FMVector3 ToPoint(const char* value, float lengthFactor=1.0f) { return ToPoint(&value, lengthFactor); } /**< See above. */
	inline static FMVector3 ToPoint(const string& value, float lengthFactor=1.0f) { return ToPoint(value.c_str(), lengthFactor); } /**< See above. */
#ifdef UNICODE
	static FMVector3 ToPoint(const fchar** value, float lengthFactor=1.0f); /**< See above. */
	inline static FMVector3 ToPoint(const fchar* value, float lengthFactor=1.0f) { return ToPoint(&value, lengthFactor); } /**< See above. */
	inline static FMVector3 ToPoint(const fstring& value, float lengthFactor=1.0f) { return ToPoint(value.c_str(), lengthFactor); } /**< See above. */
#endif

	/** Parses a string into a 4x4 matrix.
		@param value The string. For the string pointer versions of this function,
			the pointer will point to the last processed characters after the parsing.
		@param mx The matrix to be filled in.
		@param lengthFactor An optional factor that will scale the translation column
			of the matrix. */
	static void ToMatrix(const char** value, FMMatrix44& mx, float lengthFactor=1.0f);
	inline static void ToMatrix(const char* value, FMMatrix44& mx, float lengthFactor=1.0f) { return ToMatrix(&value, mx, lengthFactor); } /**< See above. */
	inline static void ToMatrix(const string& value, FMMatrix44& mx, float lengthFactor=1.0f) { return ToMatrix(value.c_str(), mx, lengthFactor); } /**< See above. */
#ifdef UNICODE
	static void ToMatrix(const fchar** value, FMMatrix44& mx, float lengthFactor=1.0f); /**< See above. */
	inline static void ToMatrix(const fchar* value, FMMatrix44& mx, float lengthFactor=1.0f) { return ToMatrix(&value, mx, lengthFactor); } /**< See above. */
	inline static void ToMatrix(const fstring& value, FMMatrix44& mx, float lengthFactor=1.0f) { return ToMatrix(value.c_str(), mx, lengthFactor); } /**< See above. */
#endif

	/** Parses a string into a datetime structure.
		@param value The string.
		@param dateTime The datetime structure to fill in. */
	static void ToDateTime(const char* value, FUDateTime& dateTime);
	inline static void ToDateTime(const string& value, FUDateTime& dateTime) { return ToDateTime(value.c_str(), dateTime); } /**< See above. */
#ifdef UNICODE
	static void ToDateTime(const fchar* value, FUDateTime& dateTime); /**< See above. */
	inline static void ToDateTime(const fstring& value, FUDateTime& dateTime) { return ToDateTime(value.c_str(), dateTime); } /**< See above. */
#endif

#ifdef HAS_VECTORTYPES

	/** Splits a string into multiple substrings.
		The separator used here are the white-spaces.
		@param value The string.
		@param array A list of strings that will be filled in. */
	static void ToFStringList(const fstring& value, FStringList& array);
	static void ToStringList(const string& value, StringList& array); /**< See above. */
#ifdef UNICODE
	static void ToStringList(const fchar* value, StringList& array); /**< See above. */
	inline static void ToStringList(const fstring& value, StringList& array) { return ToStringList(value.c_str(), array); } /**< See above. */
#endif

	/** Parses a string into a list of floating point values.
		@param value The string.
		@param array The list of floating point values to fill in. */
	static void ToFloatList(const char* value, FloatList& array);
	inline static void ToFloatList(const string& value, FloatList& array) { return ToFloatList(value.c_str(), array); } /**< See above. */
#ifdef UNICODE
	static void ToFloatList(const fchar* value, FloatList& array); /**< See above. */
	inline static void ToFloatList(const fstring& value, FloatList& array) { return ToFloatList(value.c_str(), array); } /**< See above. */
#endif

	/** Parses a string into a list of signed integers.
		@param value The string.
		@param array The list of signed integers to fill in. */
	static void ToInt32List(const char* value, Int32List& array);
	inline static void ToInt32List(const string& value, Int32List& array) { return ToInt32List(value.c_str(), array); } /**< See above. */
#ifdef UNICODE
	static void ToInt32List(const fchar* value, Int32List& array); /**< See above. */
	inline static void ToInt32List(const fstring& value, Int32List& array) { return ToInt32List(value.c_str(), array); } /**< See above. */
#endif

	/** Parses a string into a list of unsigned integers.
		@param value The string.
		@param array The list of unsigned integers to fill in. */
	static void ToUInt32List(const char* value, UInt32List& array);
	inline static void ToUInt32List(const string& value, UInt32List& array) { return ToUInt32List(value.c_str(), array); } /**< See above. */
#ifdef UNICODE
	static void ToUInt32List(const fchar* value, UInt32List& array); /**< See above. */
	inline static void ToUInt32List(const fstring& value, UInt32List& array) { return ToUInt32List(value.c_str(), array); } /**< See above. */
#endif

	/** Parses a string containing interleaved floating-point values.
		The values will be stored in multiple, independent lists.
		@param value The string containing interleaved floating-point values.
		@param arrays The lists of floating-point values to fill in. */
	static void ToInterleavedFloatList(const char* value, const vector<FloatList*>& arrays);
	inline static void ToInterleavedFloatList(const string& value, const vector<FloatList*>& arrays) { return ToInterleavedFloatList(value.c_str(), arrays); } /**< See above. */
#ifdef UNICODE
	static void ToInterleavedFloatList(const fchar* value, const vector<FloatList*>& arrays); /**< See above. */
	inline static void ToInterleavedFloatList(const fstring& value, const vector<FloatList*>& arrays) { return ToInterleavedFloatList(value.c_str(), arrays); } /**< See above. */
#endif

	/** Parses a string into a list of matrices.
		@param value The string.
		@param array The list of matrices to fill in.
		@param lengthFactor An optional factor that will scale the translation column
			of the matrices. */
	static void ToMatrixList(const char* value, FMMatrix44List& array, float lengthFactor=1.0f);
	inline static void ToMatrixList(const string& value, FMMatrix44List& array, float lengthFactor=1.0f) { return ToMatrixList(value.c_str(), array, lengthFactor); } /**< See above. */
#ifdef UNICODE
	static void ToMatrixList(const fchar* value, FMMatrix44List& array, float lengthFactor=1.0f); /**< See above. */
	inline static void ToMatrixList(const fstring& value, FMMatrix44List& array, float lengthFactor=1.0f) { return ToMatrixList(value.c_str(), array, lengthFactor); } /**< See above. */
#endif

	/** Parses a string into a list of 3D points.
		@param value The string.
		@param array The list of 3D points to fill in.
		@param lengthFactor An optional factor that will scale the points. */
	static void ToPointList(const char* value, FMVector3List& array, float lengthFactor=1.0f);
	inline static void ToPointList(const string& value, FMVector3List& array, float lengthFactor=1.0f) { return ToPointList(value.c_str(), array, lengthFactor); } /**< See above. */
#ifdef UNICODE
	static void ToPointList(const fchar* value, FMVector3List& array, float lengthFactor=1.0f); /**< See above. */
	inline static void ToPointList(const fstring& value, FMVector3List& array, float lengthFactor=1.0f) { return ToPointList(value.c_str(), array, lengthFactor); } /**< See above. */
#endif

	/** Converts a list of floating-point values into a string.
		@param builder The string builder that will contain the list of values.
			This string builder is not cleared of its contents and a space
			character will be added if it is not empty.
		@param values The list of floating-point values to convert.
		@param lengthFactor An optional factor that will scale all the
			floating-point values. */
	static void ToString(FUSStringBuilder& builder, const FloatList& values, float lengthFactor=1.0f);

	/** Converts a list of signed integers into a string.
		@param builder The string builder that will contain the list of values.
			This string builder is not cleared of its contents and a space
			character will be added if it is not empty.
		@param values The list of signed integers to convert. */
	static void ToString(FUSStringBuilder& builder, const Int32List& values);

	/** Converts a list of unsigned integers into a string.
		@param builder The string builder that will contain the list of values.
			This string builder is not cleared of its contents and a space
			character will be added if it is not empty.
		@param values The list of unsigned integers to convert. */
	static void ToString(FUSStringBuilder& builder, const UInt32List& values);

#endif // HAS_VECTORTYPES

	/** Converts a 4D vector into a string.
		@param p The 4D vector to convert.
		@param lengthFactor An optional factor that will scale the vector.
		@return The string containing the converted vector. */
	static string ToString(const FMVector4& p, float lengthFactor=1.0f);

	/** Converts a matrix into a string.
		@param value The matrix to convert.
		@param lengthFactor An optional factor that will scale the translation
			column of the matrix.
		@return The string containing the converted matrix. */
	static string ToString(const FMMatrix44& value, float lengthFactor=1.0f);
	static fstring ToFString(const FMMatrix44& value, float lengthFactor=1.0f); /**< See above. */

	/** Converts a 3D vector into a string.
		@param value The 3D vector to convert.
		@param lengthFactor An optional factor that will scale the vector.
		@return The string containing the converted vector. */
	static string ToString(const FMVector3& value, float lengthFactor=1.0f);
	static fstring ToFString(const FMVector3& value, float lengthFactor=1.0f); /**< See above. */

	/** Converts a datetime structure into a string.
		@param dateTime The datetime structure to convert.
		@return The string containing the converted datetime structure. */
	static string ToString(const FUDateTime& dateTime);
	static fstring ToFString(const FUDateTime& dateTime); /**< See above. */

	/** Converts a primitive value into a string.
		This function is templatized to use the global string builders to
		convert most primitive value types, such as signed integers,
		unsigned integers and single floating-point values, into strings.
		@see FUStringBuilderT
		@param value A primitive value.
		@return The string containing the converted primitive value. */
	template <typename T> static string ToString(const T& value) { globalSBuilder.set(value); return globalSBuilder.ToString(); }
	template <typename T> static fstring ToFString(const T& value) { globalBuilder.set(value); return globalBuilder.ToString(); } /**< See above. */

	/** Converts a matrix into a string.
		@param builder The string builder that will contain the matrix.
			This string builder is not cleared of its contents.
		@param value The matrix to convert.
		@param lengthFactor An optional factor that will scale the translation
			column of the matrix. */
	static void ToString(FUSStringBuilder& builder, const FMMatrix44& value, float lengthFactor=1.0f);
	static void ToFString(FUStringBuilder& builder, const FMMatrix44& value, float lengthFactor=1.0f); /**< See above. */

	/** Converts a 3D vector into a string.
		@param builder The string builder that will contain the 3D vector.
			This string builder is not cleared of its contents.
		@param value The 3D vector to convert.
		@param lengthFactor An optional factor that will scale the vector. */
	static void ToString(FUSStringBuilder& builder, const FMVector3& value, float lengthFactor=1.0f);
	static void ToFString(FUStringBuilder& builder, const FMVector3& value, float lengthFactor=1.0f); /**< See above. */

	/** Converts a 4D vector into a string.
		@param builder The string builder that will contain the 4D vector.
			This string builder is not cleared of its contents.
		@param p The 4D vector to convert.
		@param lengthFactor An optional factor that will scale the vector. */
	static void ToString(FUSStringBuilder& builder, const FMVector4& p, float lengthFactor=1.0f);
};

#endif // _FCU_STRING_CONVERSION_
