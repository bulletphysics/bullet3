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
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "SnSerialUtils.h"
#include "PsString.h"
#include "PxSerialization.h"
#include "PxPhysicsVersion.h"
#include "PsBasicTemplates.h"

using namespace physx;

namespace
{

#define SN_NUM_BINARY_PLATFORMS 14
const PxU32 sBinaryPlatformTags[SN_NUM_BINARY_PLATFORMS] =
{
	PX_MAKE_FOURCC('W','_','3','2'),
	PX_MAKE_FOURCC('W','_','6','4'),
	PX_MAKE_FOURCC('L','_','3','2'),
	PX_MAKE_FOURCC('L','_','6','4'),
	PX_MAKE_FOURCC('M','_','3','2'),
	PX_MAKE_FOURCC('M','_','6','4'),
	PX_MAKE_FOURCC('M','O','C','A'),
	PX_MAKE_FOURCC('A','N','D','R'),
	PX_MAKE_FOURCC('A','I','O','S'),
	PX_MAKE_FOURCC('A','A','6','4'),
	PX_MAKE_FOURCC('X','O','N','E'),
	PX_MAKE_FOURCC('N','X','3','2'),
	PX_MAKE_FOURCC('N','X','6','4'),
	PX_MAKE_FOURCC('L','A','6','4')
};

const char* sBinaryPlatformNames[SN_NUM_BINARY_PLATFORMS] =
{
	"win32",
	"win64",
	"linux32",
	"linux64",
	"mac32",
	"mac64",
	"ps4",
	"android",
	"ios",
	"ios64",
	"xboxone",
	"switch32",
	"switch64",
	"linuxaarch64"
};

#define SN_NUM_BINARY_COMPATIBLE_VERSIONS 1

//
// Important: if you adjust the following structure, please adjust the comment for PX_BINARY_SERIAL_VERSION as well
//
const Ps::Pair<PxU32, PxU32> sBinaryCompatibleVersions[SN_NUM_BINARY_COMPATIBLE_VERSIONS] =
{
	Ps::Pair<PxU32, PxU32>(PX_PHYSICS_VERSION, PX_BINARY_SERIAL_VERSION)	
};

}

namespace physx { namespace Sn {

PxU32 getBinaryPlatformTag()
{
#if PX_WINDOWS && PX_X86
	return sBinaryPlatformTags[0];
#elif PX_WINDOWS && PX_X64
	return sBinaryPlatformTags[1];
#elif PX_LINUX && PX_X86
	return sBinaryPlatformTags[2];
#elif PX_LINUX && PX_X64
	return sBinaryPlatformTags[3];
#elif PX_OSX && PX_X86
	return sBinaryPlatformTags[4];
#elif PX_OSX && PX_X64
	return sBinaryPlatformTags[5];
#elif PX_PS4
	return sBinaryPlatformTags[6];
#elif PX_ANDROID
	return sBinaryPlatformTags[7];
#elif PX_IOS && PX_ARM
	return sBinaryPlatformTags[8];
#elif PX_IOS && PX_A64
	return sBinaryPlatformTags[9];
#elif PX_XBOXONE
	return sBinaryPlatformTags[10];
#elif PX_SWITCH && !PX_A64
	return sBinaryPlatformTags[11];
#elif PX_SWITCH && PX_A64
	return sBinaryPlatformTags[12];
#elif PX_LINUX && PX_A64
	return sBinaryPlatformTags[13];
#else
	#error Unknown binary platform
#endif
}

bool isBinaryPlatformTagValid(physx::PxU32 platformTag)
{
	PxU32 platformIndex = 0;
	while (platformIndex < SN_NUM_BINARY_PLATFORMS && platformTag != sBinaryPlatformTags[platformIndex]) platformIndex++;
	return platformIndex < SN_NUM_BINARY_PLATFORMS;
}

const char* getBinaryPlatformName(physx::PxU32 platformTag)
{
	PxU32 platformIndex = 0;
	while (platformIndex < SN_NUM_BINARY_PLATFORMS && platformTag != sBinaryPlatformTags[platformIndex]) platformIndex++;
	return (platformIndex == SN_NUM_BINARY_PLATFORMS) ? "unknown" : sBinaryPlatformNames[platformIndex];
}

bool checkCompatibility(const PxU32 version, const PxU32 binaryVersion)
{		
	for(PxU32 i =0; i<SN_NUM_BINARY_COMPATIBLE_VERSIONS; i++)
	{
		if(version == sBinaryCompatibleVersions[i].first && binaryVersion == sBinaryCompatibleVersions[i].second)
			return true;
	}
	return false;
}

void getCompatibilityVersionsStr(char* buffer, PxU32 lenght)
{
	size_t len = 0;
	for(PxU32 i =0; i<SN_NUM_BINARY_COMPATIBLE_VERSIONS; i++)
	{
		physx::shdfnd::snprintf(buffer + len,  lenght - len, "%x-%d\n", sBinaryCompatibleVersions[i].first, sBinaryCompatibleVersions[i].second);	
		len = strlen(buffer);
	}	
}

} // Sn
} // physx

