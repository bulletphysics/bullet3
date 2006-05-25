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

#include "StdAfx.h"
#include "FUtils/FUStringConversion.h"
#include "FUtils/FUStringBuilder.h"
#include "FUtils/FUDateTime.h"

FUStringBuilder globalBuilder;
FUSStringBuilder globalSBuilder;

// Some string writer macros
#define SPACE builder.append(' ')
#define VAL(x) builder.append(x)

// Convert a UTF-8 string to a fstring
#ifdef UNICODE
	fstring FUStringConversion::ToFString(const char* value)
	{
		globalBuilder.clear();
		uint32 length = (uint32) strlen(value);
		globalBuilder.reserve(length + 1);
		for(uint32 i = 0; i < length; ++i)
		{
			globalBuilder.append((fchar)value[i]);
		}
		return globalBuilder.ToString();
	}
#else // UNICODE
	fstring FUStringConversion::ToFString(const char* value) { return fstring(value); }
#endif // UNICODE

// Convert a fstring string to a UTF-8 string
#ifdef UNICODE
	string FUStringConversion::ToString(const fchar* value)
	{
		globalSBuilder.clear();
		uint32 length = (uint32) fstrlen(value);
		globalSBuilder.reserve(length + 1);
		for(uint32 i = 0; i < length; ++i)
		{
			if (value[i] < 0xFF || (value[i] & (~0xFF)) >= 32) globalSBuilder.append((char)value[i]);
			else globalSBuilder.append('_'); // some generic enough character
		}
		return globalSBuilder.ToString();
	}
#else // UNICODE
	string FUStringConversion::ToString(const fchar* value) { return string(value); }
#endif // UNICODE

// Convert a fstring to a boolean value
#ifdef UNICODE
bool FUStringConversion::ToBoolean(const fchar* value)
{
	return value != NULL && *value != 0 && *value != '0' && *value != 'f' && *value != 'F';
}
#endif // UNICODE
bool FUStringConversion::ToBoolean(const char* value)
{
	return value != NULL && *value != 0 && *value != '0' && *value != 'f' && *value != 'F';
}

// Convert a fstring to a int32 and advance the character pointer
#ifdef UNICODE
int32 FUStringConversion::ToInt32(const fchar** value)
{
	if (!*value) return 0;

	// Skip beginning white spaces
	const fchar* s = *value;
	fchar c;
	while ((c = *s) != 0 && (c == ' ' || c == '\t' || c == '\r' || c == '\n')) { ++s; }

	int32 val = 0;
	int32 sign = 1;
	if (*s == '-') { ++s; sign = -1; }

	while ((c = *s) != 0)
	{
		if (c >= '0' && c <= '9') val = val * 10 + c - '0';
		else break;
		++s;
	}
	val *= sign;
	while ((c = *s) != '\0' && (c != ' ' && c != '\t' && c != '\n')) s++;
	while ((c = *s) != '\0' && (c == ' ' || c == '\t' || c == '\n')) s++;
	*value = s;
	return val;
}
#endif // UNICODE
int32 FUStringConversion::ToInt32(const char** value)
{
	if (!*value) return 0;

	// Skip beginning white spaces
	const char* s = *value;
	char c;
	while ((c = *s) != 0 && (c == ' ' || c == '\t' || c == '\r' || c == '\n')) { ++s; }

	int32 val = 0;
	int32 sign = 1;
	if (*s == '-') { ++s; sign = -1; }

	while ((c = *s) != 0)
	{
		if (c >= '0' && c <= '9') val = val * 10 + c - '0';
		else break;
		++s;
	}
	val *= sign;
	while ((c = *s) != '\0' && (c != ' ' && c != '\t' && c != '\n')) s++;
	while ((c = *s) != '\0' && (c == ' ' || c == '\t' || c == '\n')) s++;
	*value = s;
	return val;
}

// Convert a fstring to a float and advance the character pointer
#ifdef UNICODE
float FUStringConversion::ToFloat(const fchar** value)
{
	const fchar* s = *value;
	if (s == NULL || *s == 0) return 0.0f;

	// Skip beginning white spaces
	fchar c;
	while ((c = *s) != 0 && (c == ' ' || c == '\t' || c == '\r' || c == '\n')) { ++s; }

	// Skip all the plausible numerical characters
	double val = 0.0;
	int sign = 1;
	if (*s == '-') { ++s; sign = -1; }
	int32 decimals = 0;
	int32 exponent = 0;
	bool infinity = false;
	bool nonValidFound = false;
	while ((c = *s) != 0 && !nonValidFound)
	{
		switch(c)
		{
		case '.': decimals = 1; break;
		case '0': val *= 10.0; decimals *= 10; break;
		case '1': val = val * 10.0 + 1; decimals *= 10; break;
		case '2': val = val * 10.0 + 2; decimals *= 10; break;
		case '3': val = val * 10.0 + 3; decimals *= 10; break;
		case '4': val = val * 10.0 + 4; decimals *= 10; break;
		case '5': val = val * 10.0 + 5; decimals *= 10; break;
		case '6': val = val * 10.0 + 6; decimals *= 10; break;
		case '7': val = val * 10.0 + 7; decimals *= 10; break;
		case '8': val = val * 10.0 + 8; decimals *= 10; break;
		case '9': val = val * 10.0 + 9; decimals *= 10; break;
		case 'e':
		case 'E': ++s; exponent = ToInt32(&s); --s;
		case 'I': nonValidFound = true; infinity = true; --s; break;
		default: nonValidFound = true; --s; break;
		}
		++s;
	}

	if(infinity) // test for infinity
	{
		infinity = false;
		if(*s=='I')
			if(*(++s)=='N')
				if(*(++s)=='F')
				{
					infinity = true;
					decimals=0;
					val = std::numeric_limits<double>::infinity() * (double)sign;
				}
	}
	if(!infinity)
	{
		// Generate the value
		if (decimals != 0) val /= (double)decimals;
		val *= (double)sign;
		if (exponent != 0) val *= pow(10.0, (double) exponent);
	}

	// Find next whitespaces and Skip end whitespaces
	while ((c = *s) != 0 && c != ' ' && c != '\t' && c != '\r' && c != '\n') { ++s; }
	while ((c = *s) != 0 && (c == ' ' || c == '\t' || c == '\r' || c == '\n')) { ++s; }

	*value = s;
	return (float)val;
}
#endif // UNICODE
float FUStringConversion::ToFloat(const char** value)
{
	const char* s = *value;
	if (s == NULL || *s == 0) return 0.0f;

	// Skip beginning white spaces
	char c;
	while ((c = *s) != 0 && (c == ' ' || c == '\t' || c == '\r' || c == '\n')) { ++s; }

	// Skip all the plausible numerical characters
	double val = 0.0;
	int sign = 1;
	if (*s == '-') { ++s; sign = -1; }
	int32 decimals = 0;
	int32 exponent = 0;
	bool infinity = false;
	bool nonValidFound = false;
	while ((c = *s) != 0 && !nonValidFound)
	{
		switch(c)
		{
		case '.': decimals = 1; break;
		case '0': val *= 10.0; decimals *= 10; break;
		case '1': val = val * 10.0 + 1; decimals *= 10; break;
		case '2': val = val * 10.0 + 2; decimals *= 10; break;
		case '3': val = val * 10.0 + 3; decimals *= 10; break;
		case '4': val = val * 10.0 + 4; decimals *= 10; break;
		case '5': val = val * 10.0 + 5; decimals *= 10; break;
		case '6': val = val * 10.0 + 6; decimals *= 10; break;
		case '7': val = val * 10.0 + 7; decimals *= 10; break;
		case '8': val = val * 10.0 + 8; decimals *= 10; break;
		case '9': val = val * 10.0 + 9; decimals *= 10; break;
		case 'e':
		case 'E': ++s; exponent = ToInt32(&s); --s;
		case 'I': nonValidFound = true; infinity = true; --s; break;
		default: nonValidFound = true; --s; break;
		}
		++s;
	}

	if(infinity) // test for infinity
	{
		infinity = false;
		if(*s=='I')
			if(*(++s)=='N')
				if(*(++s)=='F')
				{
					infinity = true;
					decimals=0;
					val = std::numeric_limits<double>::infinity() * (double)sign;
				}
	}
	if(!infinity)
	{
		// Generate the value
		if (decimals != 0) val /= (double)decimals;
		val *= (double)sign;
		if (exponent != 0) val *= pow(10.0, (double) exponent);
	}

	// Find next whitespaces and Skip end whitespaces
	while ((c = *s) != 0 && c != ' ' && c != '\t' && c != '\r' && c != '\n') { ++s; }
	while ((c = *s) != 0 && (c == ' ' || c == '\t' || c == '\r' || c == '\n')) { ++s; }

	*value = s;
	return (float)val;
}

// Convert a fstring to a uint32 and advance the character pointer
#ifdef UNICODE
uint32 FUStringConversion::ToUInt32(const fchar** value)
{
	if (value == NULL || *value == NULL || **value == 0) return 0;

	// Skip beginning white spaces
	const fchar* s = *value;
	fchar c;
	while ((c = *s) != 0 && (c == ' ' || c == '\t' || c == '\r' || c == '\n')) { ++s; }

	uint32 val = 0;
	while ((c = *s) != 0)
	{
		if (c >= '0' && c <= '9') val = val * 10 + c - '0';
		else break;
		++s;
	}

	while ((c = *s) != '\0' && (c != ' ' && c != '\t' && c != '\n')) s++;
	while ((c = *s) != '\0' && (c == ' ' || c == '\t' || c == '\n')) s++;
	*value = s;
	return val;
}
#endif // UNICODE
uint32 FUStringConversion::ToUInt32(const char** value)
{
	if (value == NULL || *value == NULL || **value == 0) return 0;

	// Skip beginning white spaces
	const char* s = *value;
	char c;
	while ((c = *s) != 0 && (c == ' ' || c == '\t' || c == '\r' || c == '\n')) { ++s; }

	uint32 val = 0;
	while ((c = *s) != 0)
	{
		if (c >= '0' && c <= '9') val = val * 10 + c - '0';
		else break;
		++s;
	}

	while ((c = *s) != '\0' && (c != ' ' && c != '\t' && c != '\n')) s++;
	while ((c = *s) != '\0' && (c == ' ' || c == '\t' || c == '\n')) s++;
	*value = s;
	return val;
}

uint32 FUStringConversion::HexToUInt32(const char** value, uint32 count)
{
	if (value == NULL || *value == NULL || **value == 0) return 0;

	const char* s = *value;
	char c; 

	uint32 val = 0;
	for (uint32 i = 0; i < count && (c = *s) != 0; ++i)
	{
		if (c >= '0' && c <= '9') val = val * 16 + c - '0';
		else if (c >= 'A' && c <= 'F') val = val * 16 + c + 10 - 'A';
		else if (c >= 'a' && c <= 'f') val = val * 16 + c + 10 - 'a';
		else break;
		++s;
	}

	*value = s;
	return val;
}
#ifdef UNICODE
uint32 FUStringConversion::HexToUInt32(const fchar** value, uint32 count)
{
	if (value == NULL || *value == NULL || **value == 0) return 0;

	const fchar* s = *value;
	fchar c; 

	uint32 val = 0;
	for (uint32 i = 0; i < count && (c = *s) != 0; ++i)
	{
		if (c >= '0' && c <= '9') val = val * 16 + c - '0';
		else if (c >= 'A' && c <= 'Z') val = val * 16 + c + 10 - 'A';
		else if (c >= 'a' && c <= 'z') val = val * 16 + c + 10 - 'a';
		else break;
		++s;
	}

	*value = s;
	return val;
}
#endif // UNICODE

#ifdef UNICODE
void FUStringConversion::ToMatrix(const fchar** s, FMMatrix44& mx, float lengthFactor)
{
	if (s != NULL && *s != NULL && **s != 0)
	{
		// COLLADA is Column major 
		mx[0][0] = ToFloat(s); mx[1][0] = ToFloat(s); mx[2][0] = ToFloat(s); mx[3][0] = ToFloat(s) * lengthFactor;
		mx[0][1] = ToFloat(s); mx[1][1] = ToFloat(s); mx[2][1] = ToFloat(s); mx[3][1] = ToFloat(s) * lengthFactor;
		mx[0][2] = ToFloat(s); mx[1][2] = ToFloat(s); mx[2][2] = ToFloat(s); mx[3][2] = ToFloat(s) * lengthFactor;
		mx[0][3] = ToFloat(s); mx[1][3] = ToFloat(s); mx[2][3] = ToFloat(s); mx[3][3] = ToFloat(s);
	}
}
#endif // UNICODE
void FUStringConversion::ToMatrix(const char** s, FMMatrix44& mx, float lengthFactor)
{
	if (s != NULL && *s != NULL && **s != 0)
	{
		// COLLADA is Column major 
		mx[0][0] = ToFloat(s); mx[1][0] = ToFloat(s); mx[2][0] = ToFloat(s); mx[3][0] = ToFloat(s) * lengthFactor;
		mx[0][1] = ToFloat(s); mx[1][1] = ToFloat(s); mx[2][1] = ToFloat(s); mx[3][1] = ToFloat(s) * lengthFactor;
		mx[0][2] = ToFloat(s); mx[1][2] = ToFloat(s); mx[2][2] = ToFloat(s); mx[3][2] = ToFloat(s) * lengthFactor;
		mx[0][3] = ToFloat(s); mx[1][3] = ToFloat(s); mx[2][3] = ToFloat(s); mx[3][3] = ToFloat(s);
	}
}

// Convert a matrix to a string
void FUStringConversion::ToString(FUSStringBuilder& builder, const FMMatrix44& m, float lengthFactor)
{
	VAL(m[0][0]); SPACE; VAL(m[1][0]); SPACE; VAL(m[2][0]); SPACE; VAL(m[3][0] * lengthFactor); SPACE;
	VAL(m[0][1]); SPACE; VAL(m[1][1]); SPACE; VAL(m[2][1]); SPACE; VAL(m[3][1] * lengthFactor); SPACE;
	VAL(m[0][2]); SPACE; VAL(m[1][2]); SPACE; VAL(m[2][2]); SPACE; VAL(m[3][2] * lengthFactor); SPACE;
	VAL(m[0][3]); SPACE; VAL(m[1][3]); SPACE; VAL(m[2][3]); SPACE; VAL(m[3][3]);
}

string FUStringConversion::ToString(const FMMatrix44& m, float lengthFactor)
{
	globalSBuilder.clear();
	ToString(globalSBuilder, m, lengthFactor);
	return globalSBuilder.ToString();
}


string FUStringConversion::ToString(const FUDateTime& dateTime)
{
	char sz[21];
	snprintf(sz, 21, "%04d-%02d-%02dT%02d:%02d:%02dZ", dateTime.GetYear(), dateTime.GetMonth(), dateTime.GetDay(), dateTime.GetHour(), dateTime.GetMinutes(), dateTime.GetSeconds());
	sz[20] = 0;
	return string(sz);
}

// Convert a matrix to a fstring
void FUStringConversion::ToFString(FUStringBuilder& builder, const FMMatrix44& m, float lengthFactor)
{
	VAL(m[0][0]); SPACE; VAL(m[1][0]); SPACE; VAL(m[2][0]); SPACE; VAL(m[3][0] * lengthFactor); SPACE;
	VAL(m[0][1]); SPACE; VAL(m[1][1]); SPACE; VAL(m[2][1]); SPACE; VAL(m[3][1] * lengthFactor); SPACE;
	VAL(m[0][2]); SPACE; VAL(m[1][2]); SPACE; VAL(m[2][2]); SPACE; VAL(m[3][2] * lengthFactor); SPACE;
	VAL(m[0][3]); SPACE; VAL(m[1][3]); SPACE; VAL(m[2][3]); SPACE; VAL(m[3][3]);
}

fstring FUStringConversion::ToFString(const FMMatrix44& m, float lengthFactor)
{
	globalBuilder.clear();
	ToFString(globalBuilder, m, lengthFactor);
	return globalBuilder.ToString();
}

fstring FUStringConversion::ToFString(const FUDateTime& dateTime)
{
	fchar sz[21];
	fsnprintf(sz, 21, FC("%04d-%02d-%02dT%02d:%02d:%02dZ"), dateTime.GetYear(), dateTime.GetMonth(), dateTime.GetDay(), dateTime.GetHour(), dateTime.GetMinutes(), dateTime.GetSeconds());
	sz[20] = 0;
	return fstring(sz);
}

void FUStringConversion::ToDateTime(const char* value, FUDateTime& dateTime)
{
	// We're not in the business of checking the string value for UTC correctness: assume "YYYY-MM-DDTHH:MM:SSZ".
	if (strlen(value) == 20)
	{
		dateTime.SetYear(ToUInt32(value)); value += 5;
		dateTime.SetMonth(ToUInt32(value)); value += 3;
		dateTime.SetDay(ToUInt32(value)); value += 3;
		dateTime.SetHour(ToUInt32(value)); value += 3;
		dateTime.SetMinutes(ToUInt32(value)); value += 3;
		dateTime.SetSeconds(ToUInt32(value)); value += 3;
	}
}
#ifdef UNICODE
void FUStringConversion::ToDateTime(const fchar* value, FUDateTime& dateTime)
{
	if (fstrlen(value) == 20)
	{
		dateTime.SetYear(ToUInt32(value)); value += 5;
		dateTime.SetMonth(ToUInt32(value)); value += 3;
		dateTime.SetDay(ToUInt32(value)); value += 3;
		dateTime.SetHour(ToUInt32(value)); value += 3;
		dateTime.SetMinutes(ToUInt32(value)); value += 3;
		dateTime.SetSeconds(ToUInt32(value)); value += 3;
	}
}
#endif

#ifdef HAS_VECTORTYPES

// Split a fstring into multiple substrings
void FUStringConversion::ToFStringList(const fstring& value, FStringList& array)
{
	const fchar* s = value.c_str();

	// Skip beginning white spaces
	fchar c;
	while ((c = *s) != 0 && (c == ' ' || c == '\t' || c == '\r' || c == '\n')) { ++s; }

	size_t index = 0;
	while (*s != 0)
	{
		const fchar* word = s;

		// Find next white space
		while ((c = *s) != 0 && c != ' ' && c != '\t' && c != '\r' && c != '\n') { ++s; }

		if (index < array.size()) array[index++].append(word, s - word);
		else { array.push_back(fstring(word, s - word)); index++; }

		// Skip all white spaces
		while ((c = *s) != 0 && (c == ' ' || c == '\t' || c == '\r' || c == '\n')) { ++s; }
	}
}

void FUStringConversion::ToStringList(const string& value, StringList& array)
{
	const char* s = value.c_str();

	// Skip beginning white spaces
	char c;
	while ((c = *s) != 0 && (c == ' ' || c == '\t' || c == '\r' || c == '\n')) { ++s; }

	size_t index = 0;
	while (*s != 0)
	{
		const char* word = s;

		// Find next white space
		while ((c = *s) != 0 && c != ' ' && c != '\t' && c != '\r' && c != '\n') { ++s; }

		if (index < array.size()) array[index++].append(word, s - word);
		else { array.push_back(string(word, s - word)); index++; }

		// Skip all white spaces
		while ((c = *s) != 0 && (c == ' ' || c == '\t' || c == '\r' || c == '\n')) { ++s; }
	}
}

#ifdef UNICODE
void FUStringConversion::ToStringList(const fchar* value, StringList& array)
{
	// Performance could be improved...
	ToStringList(ToString(value), array);
}
#endif

// Convert a fstring to a 32-bit integer list
#ifdef UNICODE
void FUStringConversion::ToInt32List(const fchar* value, Int32List& array)
{
	if (value != NULL && *value != 0)
	{ 
		size_t length = array.size();
		for (size_t count = 0; count < length && *value != 0; ++count) array[count] = ToInt32(&value);
		while (*value != 0) { int32 i = ToInt32(&value); array.push_back(i); }
	}
}
#endif // UNICODE
void FUStringConversion::ToInt32List(const char* value, Int32List& array)
{
	if (value != NULL && *value != 0)
	{ 
		size_t length = array.size();
		for (size_t count = 0; count < length && *value != 0; ++count) array[count] = ToInt32(&value);		
		while (*value != 0) { int32 i = ToInt32(&value); array.push_back(i); }
	}
}

// Convert a fstring to a 32-bit unsigned integer list
#ifdef UNICODE
void FUStringConversion::ToUInt32List(const fchar* value, UInt32List& array)
{
	if (value != NULL && *value != 0)
	{ 
		size_t length = array.size();
		for (size_t count = 0; count < length && *value != 0; ++count) array[count] = ToUInt32(&value); 		
		while (*value != 0) { int32 i = ToUInt32(&value); array.push_back(i); }
	}
}
#endif // UNICODE
void FUStringConversion::ToUInt32List(const char* value, UInt32List& array)
{
	if (value != NULL && *value != 0)
	{ 
		size_t length = array.size();
		for (size_t count = 0; count < length && *value != 0; ++count) array[count] = ToUInt32(&value); 
		while (*value != 0) { int32 i = ToUInt32(&value); array.push_back(i); }
	}
}

// Convert a fstring to 32-bit floating point list
#ifdef UNICODE
void FUStringConversion::ToFloatList(const fchar* value, FloatList& array)
{
	if (value != NULL && *value != 0)
	{ 
		size_t length = array.size();
		for (size_t count = 0; count < length && *value != 0; ++count) array[count] = ToFloat(&value); 
		while (*value != 0) array.push_back(ToFloat(&value));
	}
}
#endif // UNICODE
void FUStringConversion::ToFloatList(const char* value, FloatList& array)
{
	if (value != NULL && *value != 0)
	{ 
		size_t length = array.size();
		for (size_t count = 0; count < length && *value != 0; ++count) array[count] = ToFloat(&value); 
		while (*value != 0) array.push_back(ToFloat(&value));
	}
}

// Convert a fstring to a list of interleaved floating points
#ifdef UNICODE
void FUStringConversion::ToInterleavedFloatList(const fchar* value, const vector<FloatList*>& arrays)
{
	size_t stride = arrays.size();
	if (value != NULL && *value != 0 && stride > 0)
	{ 
		size_t length = arrays[0]->size();
		for (size_t count = 0; count < length && *value != 0; ++count)
		{
			for (size_t i = 0; i < stride && *value != 0; ++i)
			{
				FloatList* array = arrays[i];
				if (array != NULL) array->at(count) = ToFloat(&value);
				else ToFloat(&value);
			}
		}
		
		while (*value != 0)
		{
			for (size_t i = 0; i < stride && *value != 0; ++i)
			{
				FloatList* array = arrays[i];
				if (array != NULL) array->push_back(ToFloat(&value));
				else ToFloat(&value);
			}
		}
	}	
}
#endif // UNICODE
void FUStringConversion::ToInterleavedFloatList(const char* value, const vector<FloatList*>& arrays)
{
	size_t stride = arrays.size();
	if (value != NULL && *value != 0 && stride > 0)
	{ 
		size_t length = arrays[0]->size();
		for (size_t count = 0; count < length && *value != 0; ++count)
		{
			for (size_t i = 0; i < stride && *value != 0; ++i)
			{
				FloatList* array = arrays[i];
				if (array != NULL) array->at(count) = ToFloat(&value);
				else ToFloat(&value);
			}
		}
		
		while (*value != 0)
		{
			for (size_t i = 0; i < stride && *value != 0; ++i)
			{
				FloatList* array = arrays[i];
				if (array != NULL) array->push_back(ToFloat(&value));
				else ToFloat(&value);
			}
		}
	}	
}

// Convert a fstring to a (X,Y,Z) Point object
#ifdef UNICODE
FMVector3 FUStringConversion::ToPoint(const fchar** value, float lengthFactor)
{
	FMVector3 p;
	if (value != NULL && *value != NULL && **value != 0)
	{
		p.x = ToFloat(value) * lengthFactor;
		p.y = ToFloat(value) * lengthFactor;
		p.z = ToFloat(value) * lengthFactor;
	}
	return p;
}
#endif // UNICODE
FMVector3 FUStringConversion::ToPoint(const char** value, float lengthFactor)
{
	FMVector3 p;
	if (value != NULL && *value != NULL && **value != 0)
	{
		p.x = ToFloat(value) * lengthFactor;
		p.y = ToFloat(value) * lengthFactor;
		p.z = ToFloat(value) * lengthFactor;
	}
	return p;
}

// Convert a point to a string
void FUStringConversion::ToString(FUSStringBuilder& builder, const FMVector3& p, float lengthFactor)
{
	VAL(p.x * lengthFactor); SPACE; VAL(p.y * lengthFactor); SPACE; VAL(p.z * lengthFactor);
}

string FUStringConversion::ToString(const FMVector3& p, float lengthFactor)
{
	globalSBuilder.clear();
	ToString(globalSBuilder, p, lengthFactor);
	return globalSBuilder.ToString();
}

// Convert a vector4 to a string
void FUStringConversion::ToString(FUSStringBuilder& builder, const FMVector4& p, float lengthFactor)
{
	VAL(p.w * lengthFactor); SPACE; VAL(p.x * lengthFactor); SPACE; VAL(p.y * lengthFactor); SPACE; VAL(p.z * lengthFactor);
}

string FUStringConversion::ToString(const FMVector4& p, float lengthFactor)
{
	globalSBuilder.clear();
	ToString(globalSBuilder, p, lengthFactor);
	return globalSBuilder.ToString();
}


// Convert a point to a fstring
void FUStringConversion::ToFString(FUStringBuilder& builder, const FMVector3& p, float lengthFactor)
{
	VAL(p.x * lengthFactor); SPACE; VAL(p.y * lengthFactor); SPACE; VAL(p.z * lengthFactor);
}

fstring FUStringConversion::ToFString(const FMVector3& p, float lengthFactor)
{
	globalBuilder.clear();
	ToFString(globalBuilder, p, lengthFactor);
	return globalBuilder.ToString();
}

#ifdef HAS_VECTORTYPES
void FUStringConversion::ToString(FUSStringBuilder& builder, const FloatList& values, float lengthFactor)
{
	if (values.empty()) return;
	if (!builder.empty()) SPACE;
	FloatList::const_iterator itV = values.begin();
	if (IsEquivalent(lengthFactor, 1.0f))
	{
		builder.append(*itV);
		for (++itV; itV != values.end(); ++itV) { SPACE; VAL(*itV); }
	}
	else
	{
		builder.append(*itV * lengthFactor);
		for (++itV; itV != values.end(); ++itV) { SPACE; VAL(*itV * lengthFactor); }
	}
}

void FUStringConversion::ToString(FUSStringBuilder& builder, const Int32List& values)
{
	if (values.empty()) return;
	if (!builder.empty()) SPACE;
	Int32List::const_iterator itV = values.begin();
	builder.append(*itV);
	for (; itV != values.end(); ++itV) { SPACE; VAL(*itV); }
}

void FUStringConversion::ToString(FUSStringBuilder& builder, const UInt32List& values)
{
	if (values.empty()) return;
	if (!builder.empty()) SPACE;
	UInt32List::const_iterator itV = values.begin();
	builder.append(*itV);
	for (; itV != values.end(); ++itV) { SPACE; VAL(*itV); }
}
#endif // HAS_VECTORTYPES

// Convert a fstring to a list of matrices
#ifdef UNICODE
void FUStringConversion::ToMatrixList(const fchar* value, FMMatrix44List& array, float lengthFactor)
{
	if (value != NULL && *value != 0)
	{ 
		size_t length = array.size();
		for (size_t count = 0; count < length && *value != 0; ++count) 
		{
			ToMatrix(&value, array[count], lengthFactor);
		}
		
		while (*value != 0)
		{
			FMMatrix44List::iterator it = array.insert(array.end(), FMMatrix44::Identity);
			ToMatrix(&value, *it, lengthFactor);
		}
	}
}
#endif // UNICODE
void FUStringConversion::ToMatrixList(const char* value, FMMatrix44List& array, float lengthFactor)
{
	if (value != NULL && *value != 0)
	{ 
		size_t length = array.size();
		for (size_t count = 0; count < length && *value != 0; ++count)
		{
			ToMatrix(&value, array[count], lengthFactor);
		}
		
		while (*value != 0)
		{
			FMMatrix44List::iterator it = array.insert(array.end(), FMMatrix44::Identity);
			ToMatrix(&value, *it, lengthFactor);
		}
	}
}

#ifdef UNICODE
void FUStringConversion::ToPointList(const fchar* value, FMVector3List& array, float lengthFactor)
{
	if (value != NULL && *value != 0)
	{ 
		size_t length = array.size();
		for (size_t count = 0; count < length && *value != 0; ++count) 
		{
			array[count] = ToPoint(&value, lengthFactor);
		}
		
		while (*value != 0) array.push_back(ToPoint(&value, lengthFactor));
	}
}
#endif // UNICODE
void FUStringConversion::ToPointList(const char* value, FMVector3List& array, float lengthFactor)
{
	if (value != NULL && *value != 0)
	{ 
		size_t length = array.size();
		for (size_t count = 0; count < length && *value != 0; ++count)
		{
			array[count] = ToPoint(&value, lengthFactor);
		}
		
		while (*value != 0) array.push_back(ToPoint(&value, lengthFactor));
	}
}

#endif // HAS_VECTORTYPES

// Parasitic: Common string operators
#ifdef UNICODE
fstring operator+(const fstring& sz1, int32 i)
{
	globalBuilder.set(sz1);
	globalBuilder.append(i);
	return globalBuilder.ToString();
}
#endif // UNICODE

string operator+(const string& sz1, int32 i)
{
	globalSBuilder.set(sz1);
	globalSBuilder.append(i);
	return globalSBuilder.ToString();
}

// Parasitic: empty string constants
const string emptyString("");
const fstring emptyFString(FC(""));
