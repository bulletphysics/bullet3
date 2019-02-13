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
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#ifndef SN_FILE_H
#define SN_FILE_H

// fopen_s - returns 0 on success, non-zero on failure

#if PX_MICROSOFT_FAMILY

#include <stdio.h>

namespace physx
{
namespace sn
{
PX_INLINE PxI32 fopen_s(FILE** file, const char* name, const char* mode)
{
	static const PxU32 MAX_LEN = 300; 
	char buf[MAX_LEN+1];

	PxU32 i;
	for(i = 0; i<MAX_LEN && name[i]; i++)
		buf[i] = name[i] == '/' ? '\\' : name[i];
	buf[i] = 0;

	return i == MAX_LEN ? -1 : ::fopen_s(file, buf, mode);
};

} // namespace sn
} // namespace physx

#elif PX_UNIX_FAMILY || PX_PS4 || PX_SWITCH

#include <stdio.h>

namespace physx
{
namespace sn
{
PX_INLINE PxI32 fopen_s(FILE** file, const char* name, const char* mode)
{
	FILE* fp = ::fopen(name, mode);
	if(fp)
	{
		*file = fp;
		return PxI32(0);
	}
	return -1;
}
} // namespace sn
} // namespace physx
#else
#error "Platform not supported!"
#endif

#endif //SN_FILE_H

