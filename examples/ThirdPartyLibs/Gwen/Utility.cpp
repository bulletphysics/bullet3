/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/ToolTip.h"
#include "Gwen/Utility.h"

using namespace Gwen;

#ifdef _MSC_VER
	#pragma warning(disable:4267)// conversion from 'size_t' to 'int', possible loss of data
#endif

UnicodeString Gwen::Utility::Format( const wchar_t* fmt, ... )
{
	wchar_t strOut[ 2048 ];

	va_list s;
	va_start( s, fmt ); 
	GwenUtil_VSWPrintFSafeSized( strOut, fmt, s );
	va_end(s);

	UnicodeString str = strOut;
	return str;
}



void Gwen::Utility::Strings::Split( const Gwen::String& str, const Gwen::String& seperator, Strings::List& outbits, bool bLeave )
{
	int iOffset = 0;
	int iLength = str.length();
	int iSepLen = seperator.length();

	size_t i = str.find( seperator, 0 );
	while ( i != std::string::npos )
	{
		outbits.push_back( str.substr( iOffset, i-iOffset ) );
		iOffset = i + iSepLen;

		i = str.find( seperator, iOffset );
		if ( bLeave ) iOffset -= iSepLen;
	}

	outbits.push_back( str.substr( iOffset, iLength-iOffset ) );
}

void Gwen::Utility::Strings::Split( const Gwen::UnicodeString& str, const Gwen::UnicodeString& seperator, Strings::UnicodeList& outbits, bool bLeave )
{
	int iOffset = 0;
	int iLength = str.length();
	int iSepLen = seperator.length();

	size_t i = str.find( seperator, 0 );
	while ( i != std::wstring::npos )
	{
		outbits.push_back( str.substr( iOffset, i-iOffset ) );
		iOffset = i + iSepLen;

		i = str.find( seperator, iOffset );
		if ( bLeave ) iOffset -= iSepLen;
	}

	outbits.push_back( str.substr( iOffset, iLength-iOffset ) );
}

int Gwen::Utility::Strings::To::Int( const Gwen::String& str )
{
	if ( str == "" ) return 0;
	return atoi( str.c_str() );
}

float Gwen::Utility::Strings::To::Float( const Gwen::String& str )
{
	if ( str == "" ) return 0.0f;
	return (float)atof( str.c_str() );
}

bool Gwen::Utility::Strings::To::Bool( const Gwen::String& str )
{
	if ( str.size() == 0 ) return false;
	if ( str[0] == 'T' || str[0] == 't' || str[0] == 'y' || str[0] == 'Y' ) return true;
	if ( str[0] == 'F' || str[0] == 'f' || str[0] == 'n' || str[0] == 'N' ) return false;
	if ( str[0] == '0' ) return false;
	return true;
}

bool Gwen::Utility::Strings::To::Floats( const Gwen::String& str, float* f, size_t iCount )
{
	Strings::List lst;
	Strings::Split( str, " ", lst );
	if ( lst.size() != iCount ) return false;

	for ( size_t i=0; i<iCount; i++ )
	{
		f[i] = Strings::To::Float( lst[i] );
	}

	return true;
}








