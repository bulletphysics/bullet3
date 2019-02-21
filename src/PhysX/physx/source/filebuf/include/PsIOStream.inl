//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.

/*!
\file
\brief PsIOStream inline implementation
*/

PX_INLINE PsIOStream& PsIOStream::operator<<(bool v)
{
	if ( mBinary )
	{
		mStream.storeByte((uint8_t)v);
	}
	else
	{
		char scratch[6];
		storeString( physx::PxAsc::valueToStr(v, scratch, 6) );
	}
	return *this;
}


PX_INLINE PsIOStream& PsIOStream::operator<<(char c)
{
	mStream.storeByte((uint8_t)c);
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator<<(uint8_t c)
{
	if ( mBinary )
	{
		mStream.storeByte((uint8_t)c);
	}
	else
	{
		char scratch[physx::PxAsc::IntStrLen];
		storeString( physx::PxAsc::valueToStr(c, scratch, physx::PxAsc::IntStrLen) );
	}

	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator<<(int8_t c)
{
	if ( mBinary )
	{
		mStream.storeByte((uint8_t)c);
	}
	else
	{
		char scratch[physx::PxAsc::IntStrLen];
		storeString( physx::PxAsc::valueToStr(c, scratch, physx::PxAsc::IntStrLen) );
	}

	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator<<(const char *c)
{
	if ( mBinary )
	{
		c = c ? c : ""; // it it is a null pointer, assign it to an empty string.
		uint32_t len = (uint32_t)strlen(c);
		PX_ASSERT( len < (MAX_STREAM_STRING-1));
		if ( len > (MAX_STREAM_STRING-1) )
		{
			len = MAX_STREAM_STRING-1;
		}
		mStream.storeDword(len);
		if ( len )
			mStream.write(c,len);
	}
	else
	{
		storeString(c);
	}
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator<<(uint64_t v)
{
	if ( mBinary )
	{
		mStream.storeDouble( (double) v );
	}
	else
	{
		char scratch[physx::PxAsc::IntStrLen];
		storeString( physx::PxAsc::valueToStr(v, scratch, physx::PxAsc::IntStrLen) );
	}
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator<<(int64_t v)
{
	if ( mBinary )
	{
		mStream.storeDouble( (double) v );
	}
	else
	{
		char scratch[physx::PxAsc::IntStrLen];
		storeString( physx::PxAsc::valueToStr(v, scratch, physx::PxAsc::IntStrLen) );
	}
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator<<(double v)
{
	if ( mBinary )
	{
		mStream.storeDouble( (double) v );
	}
	else
	{
		char scratch[physx::PxAsc::PxF64StrLen];
		storeString( physx::PxAsc::valueToStr(v, scratch, physx::PxAsc::PxF64StrLen) );
	}
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator<<(float v)
{
	if ( mBinary )
	{
		mStream.storeFloat(v);
	}
	else
	{
		char scratch[physx::PxAsc::PxF32StrLen];
		storeString( physx::PxAsc::valueToStr(v, scratch, physx::PxAsc::PxF32StrLen) );

	}
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator<<(uint32_t v)
{
	if ( mBinary )
	{
		mStream.storeDword(v);
	}
	else
	{
		char scratch[physx::PxAsc::IntStrLen];
		storeString( physx::PxAsc::valueToStr(v, scratch, physx::PxAsc::IntStrLen) );
	}
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator<<(int32_t v)
{
	if ( mBinary )
	{
		mStream.storeDword( (uint32_t) v );
	}
	else
	{
		char scratch[physx::PxAsc::IntStrLen];
		storeString( physx::PxAsc::valueToStr(v, scratch, physx::PxAsc::IntStrLen) );
	}
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator<<(uint16_t v)
{
	if ( mBinary )
	{
		mStream.storeWord(v);
	}
	else
	{
		char scratch[physx::PxAsc::IntStrLen];
		storeString( physx::PxAsc::valueToStr(v, scratch, physx::PxAsc::IntStrLen) );
	}
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator<<(int16_t v)
{
	if ( mBinary )
	{
		mStream.storeWord( (uint16_t) v );
	}
	else
	{
		char scratch[physx::PxAsc::IntStrLen];
		storeString( physx::PxAsc::valueToStr(v, scratch, physx::PxAsc::IntStrLen) );
	}
	return *this;
}


PX_INLINE PsIOStream& PsIOStream::operator>>(uint32_t &v)
{
	if ( mBinary )
	{
		v = mStream.readDword();
	}
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator>>(char &v)
{
	if ( mBinary )
	{
		v = (char)mStream.readByte();
	}
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator>>(uint8_t &v)
{
	if ( mBinary )
	{
		v = mStream.readByte();
	}
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator>>(int8_t &v)
{
	if ( mBinary )
	{
		v = (int8_t)mStream.readByte();
	}
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator>>(int64_t &v)
{
	if ( mBinary )
	{
		v = mStream.readDword();
	}
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator>>(uint64_t &v)
{
	if ( mBinary )
	{
		v = (uint64_t)mStream.readDouble();
	}
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator>>(double &v)
{
	if ( mBinary )
	{
		v = mStream.readDouble();
	}
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator>>(float &v)
{
	if ( mBinary )
	{
		v = mStream.readFloat();
	}
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator>>(int32_t &v)
{
	if ( mBinary )
	{
		v = (int32_t)mStream.readDword();
	}
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator>>(uint16_t &v)
{
	if ( mBinary )
	{
		v = mStream.readWord();
	}
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator>>(int16_t &v)
{
	if ( mBinary )
	{
		v = (int16_t)mStream.readWord();
	}
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator>>(bool &v)
{
	int8_t iv;
	iv = (int8_t)mStream.readByte();
	v = iv ? true : false;
	return *this;
}

#define NX_IOSTREAM_COMMA_SEPARATOR if(!mBinary) *this << ' ';

PX_INLINE PsIOStream& PsIOStream::operator<<(const physx::PxVec3 &v)
{
	*this << v.x; 
	NX_IOSTREAM_COMMA_SEPARATOR;
	*this << v.y;
	NX_IOSTREAM_COMMA_SEPARATOR;
	*this << v.z;
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator<<(const physx::PxQuat &v)
{
	*this << v.x;
	NX_IOSTREAM_COMMA_SEPARATOR;
	*this << v.y;
	NX_IOSTREAM_COMMA_SEPARATOR;
	*this << v.z;
	NX_IOSTREAM_COMMA_SEPARATOR;
	*this << v.w;
	return *this;
}


PX_INLINE PsIOStream& PsIOStream::operator<<(const physx::PxBounds3 &v)
{
	*this << v.minimum;
	NX_IOSTREAM_COMMA_SEPARATOR;
	*this << v.maximum;
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator>>(physx::PxVec3 &v)
{
	*this >> v.x;
	*this >> v.y;
	*this >> v.z;
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator>>(physx::PxQuat &v)
{
	*this>>v.x;
	*this>>v.y;
	*this>>v.z;
	*this>>v.w;
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator>>(physx::PxBounds3 &v)
{
	*this >> v.minimum;
	*this >> v.maximum;
	return *this;
}

PX_INLINE PsIOStream& PsIOStream::operator>>(const char *&str)
{
	str = NULL; // by default no string streamed...
	if ( mBinary )
	{
		uint32_t len=0;
		*this >> len;

		PX_ASSERT( len < (MAX_STREAM_STRING-1) );
		if ( len < (MAX_STREAM_STRING-1) )
		{
			mStream.read(mReadString,len);
			mReadString[len] = 0;
			str = mReadString;
		}
	}
	return *this;
}


PX_INLINE void  PsIOStream::storeString(const char *c,bool zeroTerminate)
{
	while ( *c )
	{
		mStream.storeByte((uint8_t)*c);
		c++;
	}
	if ( zeroTerminate )
	{
		mStream.storeByte(0);
	}
}
