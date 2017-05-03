//========= Copyright Valve Corporation ============//
#include "strtools.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool StringHasPrefix( const std::string & sString, const std::string & sPrefix )
{
	return 0 == strnicmp( sString.c_str(), sPrefix.c_str(), sPrefix.length() );
}

bool StringHasPrefixCaseSensitive( const std::string & sString, const std::string & sPrefix )
{
	return 0 == strncmp( sString.c_str(), sPrefix.c_str(), sPrefix.length() );
}


bool StringHasSuffix( const std::string &sString, const std::string &sSuffix )
{
	size_t cStrLen = sString.length();
	size_t cSuffixLen = sSuffix.length();

	if ( cSuffixLen > cStrLen )
		return false;

	std::string sStringSuffix = sString.substr( cStrLen - cSuffixLen, cSuffixLen );

	return 0 == stricmp( sStringSuffix.c_str(), sSuffix.c_str() );
}

bool StringHasSuffixCaseSensitive( const std::string &sString, const std::string &sSuffix )
{
	size_t cStrLen = sString.length();
	size_t cSuffixLen = sSuffix.length();

	if ( cSuffixLen > cStrLen )
		return false;

	std::string sStringSuffix = sString.substr( cStrLen - cSuffixLen, cSuffixLen );

	return 0 == strncmp( sStringSuffix.c_str(), sSuffix.c_str(),cSuffixLen );
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
std::string UTF16to8(const wchar_t * in)
{
	std::string out;
	unsigned int codepoint = 0;
	for ( ; in && *in != 0; ++in )
	{
		if (*in >= 0xd800 && *in <= 0xdbff)
			codepoint = ((*in - 0xd800) << 10) + 0x10000;
		else
		{
			if (*in >= 0xdc00 && *in <= 0xdfff)
				codepoint |= *in - 0xdc00;
			else
				codepoint = *in;

			if (codepoint <= 0x7f)
				out.append(1, static_cast<char>(codepoint));
			else if (codepoint <= 0x7ff)
			{
				out.append(1, static_cast<char>(0xc0 | ((codepoint >> 6) & 0x1f)));
				out.append(1, static_cast<char>(0x80 | (codepoint & 0x3f)));
			}
			else if (codepoint <= 0xffff)
			{
				out.append(1, static_cast<char>(0xe0 | ((codepoint >> 12) & 0x0f)));
				out.append(1, static_cast<char>(0x80 | ((codepoint >> 6) & 0x3f)));
				out.append(1, static_cast<char>(0x80 | (codepoint & 0x3f)));
			}
			else
			{
				out.append(1, static_cast<char>(0xf0 | ((codepoint >> 18) & 0x07)));
				out.append(1, static_cast<char>(0x80 | ((codepoint >> 12) & 0x3f)));
				out.append(1, static_cast<char>(0x80 | ((codepoint >> 6) & 0x3f)));
				out.append(1, static_cast<char>(0x80 | (codepoint & 0x3f)));
			}
			codepoint = 0;
		}
	}
	return out;
}

std::wstring UTF8to16(const char * in)
{
	std::wstring out;
	unsigned int codepoint = 0;
	int following = 0;
	for ( ; in && *in != 0; ++in )
	{
		unsigned char ch = *in;
		if (ch <= 0x7f)
		{
			codepoint = ch;
			following = 0;
		}
		else if (ch <= 0xbf)
		{
			if (following > 0)
			{
				codepoint = (codepoint << 6) | (ch & 0x3f);
				--following;
			}
		}
		else if (ch <= 0xdf)
		{
			codepoint = ch & 0x1f;
			following = 1;
		}
		else if (ch <= 0xef)
		{
			codepoint = ch & 0x0f;
			following = 2;
		}
		else
		{
			codepoint = ch & 0x07;
			following = 3;
		}
		if (following == 0)
		{
			if (codepoint > 0xffff)
			{
				out.append(1, static_cast<wchar_t>(0xd800 + (codepoint >> 10)));
				out.append(1, static_cast<wchar_t>(0xdc00 + (codepoint & 0x03ff)));
			}
			else
				out.append(1, static_cast<wchar_t>(codepoint));
			codepoint = 0;
		}
	}
	return out;
}


void strcpy_safe( char *pchBuffer, size_t unBufferSizeBytes, const char *pchSource )
{
	pchBuffer[ unBufferSizeBytes - 1 ] = '\0';
	strncpy( pchBuffer, pchSource, unBufferSizeBytes - 1 );
}


// --------------------------------------------------------------------
// Purpose: converts a string to upper case
// --------------------------------------------------------------------
std::string StringToUpper( const std::string & sString )
{
	std::string sOut;
	sOut.reserve( sString.size() + 1 );
	for( std::string::const_iterator i = sString.begin(); i != sString.end(); i++ )
	{
		sOut.push_back( (char)toupper( *i ) );
	}

	return sOut;
}


// --------------------------------------------------------------------
// Purpose: converts a string to lower case
// --------------------------------------------------------------------
std::string StringToLower( const std::string & sString )
{
	std::string sOut;
	sOut.reserve( sString.size() + 1 );
	for( std::string::const_iterator i = sString.begin(); i != sString.end(); i++ )
	{
		sOut.push_back( (char)tolower( *i ) );
	}

	return sOut;
}


uint32_t ReturnStdString( const std::string & sValue, char *pchBuffer, uint32_t unBufferLen )
{
	uint32_t unLen = (uint32_t)sValue.length() + 1;
	if( !pchBuffer || !unBufferLen )
		return unLen;

	if( unBufferLen < unLen )
	{
		pchBuffer[0] = '\0';
	}
	else
	{
		memcpy( pchBuffer, sValue.c_str(), unLen );
	}

	return unLen;
}

void BufferToStdString( std::string & sDest, const char *pchBuffer, uint32_t unBufferLen )
{
	sDest.resize( unBufferLen + 1 );
	memcpy( const_cast< char* >( sDest.c_str() ), pchBuffer, unBufferLen );
	const_cast< char* >( sDest.c_str() )[ unBufferLen ] = '\0';
}

/** Returns a std::string from a uint64_t */
std::string Uint64ToString( uint64_t ulValue )
{
	char buf[ 22 ];
#if defined( _WIN32 )
	sprintf_s( buf, "%llu", ulValue );
#else
    snprintf( buf, sizeof( buf ), "%llu", (long long unsigned int ) ulValue );
#endif
	return buf;
}


/** returns a uint64_t from a string */
uint64_t StringToUint64( const std::string & sValue )
{
	return strtoull( sValue.c_str(), NULL, 0 );
}

//-----------------------------------------------------------------------------
// Purpose: Helper for converting a numeric value to a hex digit, value should be 0-15.
//-----------------------------------------------------------------------------
char cIntToHexDigit( int nValue )
{
	//Assert( nValue >= 0 && nValue <= 15 );
	return "0123456789ABCDEF"[ nValue & 15 ];
}

//-----------------------------------------------------------------------------
// Purpose: Helper for converting a hex char value to numeric, return -1 if the char
//          is not a valid hex digit.
//-----------------------------------------------------------------------------
int iHexCharToInt( char cValue )
{
	int32_t iValue = cValue;
	if ( (uint32_t)( iValue - '0' ) < 10 )
		return iValue - '0';

	iValue |= 0x20;
	if ( (uint32_t)( iValue - 'a' ) < 6 )
		return iValue - 'a' + 10;

	return -1;
}

//-----------------------------------------------------------------------------
// Purpose: Internal implementation of encode, works in the strict RFC manner, or
//          with spaces turned to + like HTML form encoding.
//-----------------------------------------------------------------------------
void V_URLEncodeInternal( char *pchDest, int nDestLen, const char *pchSource, int nSourceLen, bool bUsePlusForSpace )
{
	//AssertMsg( nDestLen > 3*nSourceLen, "Target buffer for V_URLEncode should be 3x source length, plus one for terminating null\n" );
	
	int iDestPos = 0;
	for ( int i=0; i < nSourceLen; ++i )
	{
		// worst case we need 3 additional chars
		if( (iDestPos+3) > nDestLen  )
		{
			pchDest[0] = '\0';
//			AssertMsg( false, "Target buffer too short\n" );
			return;
		}

		// We allow only a-z, A-Z, 0-9, period, underscore, and hyphen to pass through unescaped.
		// These are the characters allowed by both the original RFC 1738 and the latest RFC 3986.
		// Current specs also allow '~', but that is forbidden under original RFC 1738.
		if ( !( pchSource[i] >= 'a' && pchSource[i] <= 'z' ) && !( pchSource[i] >= 'A' && pchSource[i] <= 'Z' ) && !(pchSource[i] >= '0' && pchSource[i] <= '9' )
			 && pchSource[i] != '-' && pchSource[i] != '_' && pchSource[i] != '.'	
		)
		{
			if ( bUsePlusForSpace && pchSource[i] == ' ' )
			{
				pchDest[iDestPos++] = '+';
			}
			else
			{
				pchDest[iDestPos++] = '%';
				uint8_t iValue = pchSource[i];
				if ( iValue == 0 )
				{
					pchDest[iDestPos++] = '0';
					pchDest[iDestPos++] = '0';
				}
				else
				{
					char cHexDigit1 = cIntToHexDigit( iValue % 16 );
					iValue /= 16;
					char cHexDigit2 = cIntToHexDigit( iValue );
					pchDest[iDestPos++] = cHexDigit2;
					pchDest[iDestPos++] = cHexDigit1;
				}
			}
		}
		else
		{
			pchDest[iDestPos++] = pchSource[i];
		}
	}

	if( (iDestPos+1) > nDestLen )
	{
		pchDest[0] = '\0';
		//AssertMsg( false, "Target buffer too short to terminate\n" );
		return;
	}

	// Null terminate
	pchDest[iDestPos++] = 0;
}


//-----------------------------------------------------------------------------
// Purpose: Internal implementation of decode, works in the strict RFC manner, or
//          with spaces turned to + like HTML form encoding.
//
//			Returns the amount of space used in the output buffer.
//-----------------------------------------------------------------------------
size_t V_URLDecodeInternal( char *pchDecodeDest, int nDecodeDestLen, const char *pchEncodedSource, int nEncodedSourceLen, bool bUsePlusForSpace )
{
	if ( nDecodeDestLen < nEncodedSourceLen )
	{
		//AssertMsg( false, "V_URLDecode needs a dest buffer at least as large as the source" );
		return 0;
	}

	int iDestPos = 0;
	for( int i=0; i < nEncodedSourceLen; ++i )
	{
		if ( bUsePlusForSpace && pchEncodedSource[i] == '+' )
		{
			pchDecodeDest[ iDestPos++ ] = ' ';
		}
		else if ( pchEncodedSource[i] == '%' )
		{
			// Percent signifies an encoded value, look ahead for the hex code, convert to numeric, and use that

			// First make sure we have 2 more chars
			if ( i < nEncodedSourceLen - 2 )
			{
				char cHexDigit1 = pchEncodedSource[i+1];
				char cHexDigit2 = pchEncodedSource[i+2];

				// Turn the chars into a hex value, if they are not valid, then we'll
				// just place the % and the following two chars direct into the string,
				// even though this really shouldn't happen, who knows what bad clients
				// may do with encoding.
				bool bValid = false;
				int iValue = iHexCharToInt( cHexDigit1 );
				if ( iValue != -1 )
				{
					iValue *= 16;
					int iValue2 = iHexCharToInt( cHexDigit2 );
					if ( iValue2 != -1 )
					{
						iValue += iValue2;
						pchDecodeDest[ iDestPos++ ] = (char)iValue;
						bValid = true;
					}
				}

				if ( !bValid )
				{
					pchDecodeDest[ iDestPos++ ] = '%';
					pchDecodeDest[ iDestPos++ ] = cHexDigit1;
					pchDecodeDest[ iDestPos++ ] = cHexDigit2;
				}
			}

			// Skip ahead
			i += 2;
		}
		else
		{
			pchDecodeDest[ iDestPos++ ] = pchEncodedSource[i];
		}
	}

	// We may not have extra room to NULL terminate, since this can be used on raw data, but if we do
	// go ahead and do it as this can avoid bugs.
	if ( iDestPos < nDecodeDestLen )
	{
		pchDecodeDest[iDestPos] = 0;
	}

	return (size_t)iDestPos;
}

//-----------------------------------------------------------------------------
// Purpose: Encodes a string (or binary data) from URL encoding format, see rfc1738 section 2.2.  
//          This version of the call isn't a strict RFC implementation, but uses + for space as is
//          the standard in HTML form encoding, despite it not being part of the RFC.
//
//          Dest buffer should be at least as large as source buffer to guarantee room for decode.
//-----------------------------------------------------------------------------
void V_URLEncode( char *pchDest, int nDestLen, const char *pchSource, int nSourceLen )
{
	return V_URLEncodeInternal( pchDest, nDestLen, pchSource, nSourceLen, true );
}


//-----------------------------------------------------------------------------
// Purpose: Decodes a string (or binary data) from URL encoding format, see rfc1738 section 2.2.  
//          This version of the call isn't a strict RFC implementation, but uses + for space as is
//          the standard in HTML form encoding, despite it not being part of the RFC.
//
//          Dest buffer should be at least as large as source buffer to guarantee room for decode.
//			Dest buffer being the same as the source buffer (decode in-place) is explicitly allowed.
//-----------------------------------------------------------------------------
size_t V_URLDecode( char *pchDecodeDest, int nDecodeDestLen, const char *pchEncodedSource, int nEncodedSourceLen )
{
	return V_URLDecodeInternal( pchDecodeDest, nDecodeDestLen, pchEncodedSource, nEncodedSourceLen, true );
}

//-----------------------------------------------------------------------------
void V_StripExtension( std::string &in )
{
	// Find the last dot. If it's followed by a dot or a slash, then it's part of a 
	// directory specifier like ../../somedir/./blah.
	std::string::size_type test = in.rfind( '.' );
	if ( test != std::string::npos )
	{
		// This handles things like ".\blah" or "c:\my@email.com\abc\def\geh"
		// Which would otherwise wind up with "" and "c:\my@email", respectively.
		if ( in.rfind( '\\' ) < test && in.rfind( '/' ) < test )
		{
			in.resize( test );
		}
	}
}

