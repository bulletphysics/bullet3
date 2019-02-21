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

#ifndef __NVPHYSXTODRV_H__
#define __NVPHYSXTODRV_H__

// The puprose of this interface is to provide graphics drivers with information 
// about PhysX state to draw PhysX visual indicator 

// We share information between modules using a memory section object. PhysX creates 
// such object, graphics drivers try to open it. The name of the object has 
// fixed part (NvPhysXToDrv_SectionName) followed by the process id. This allows 
// each process to have its own communication channel. 

namespace physx
{

#define NvPhysXToDrv_SectionName "PH71828182845_" 

// Vista apps cannot create stuff in Global\\ namespace when NOT elevated, so use local scope
#define NvPhysXToDrv_Build_SectionName(PID, buf) sprintf(buf, NvPhysXToDrv_SectionName "%x", static_cast<unsigned int>(PID)) 
#define NvPhysXToDrv_Build_SectionNameXP(PID, buf) sprintf(buf, "Global\\" NvPhysXToDrv_SectionName "%x", static_cast<unsigned int>(PID)) 

typedef struct NvPhysXToDrv_Header_ 
{ 
    int signature; // header interface signature 
    int version; // version of the interface 
    int size; // size of the structure 
    int reserved; // reserved, must be zero 
}	NvPhysXToDrv_Header; 

// this structure describes layout of data in the shared memory section 
typedef struct NvPhysXToDrv_Data_V1_ 
{ 
    NvPhysXToDrv_Header header; // keep this member first in all versions of the interface. 

    int bCpuPhysicsPresent; // nonzero if cpu physics is initialized 
    int bGpuPhysicsPresent; // nonzero if gpu physics is initialized 

}	NvPhysXToDrv_Data_V1; 

// some random magic number as our interface signature 
#define NvPhysXToDrv_Header_Signature 0xA7AB 

// use the macro to setup the header to the latest version of the interface 
// update the macro when a new verson of the interface is added 
#define NvPhysXToDrv_Header_Init(header)               \
{                                                      \
    header.signature = NvPhysXToDrv_Header_Signature;  \
    header.version = 1;                                \
    header.size = sizeof(NvPhysXToDrv_Data_V1);        \
    header.reserved = 0;                               \
} 

// validate the header against all known interface versions 
// add validation checks when new interfaces are added 
#define NvPhysXToDrv_Header_Validate(header, curVersion)   \
  (                                                        \
   (header.signature == NvPhysXToDrv_Header_Signature) &&  \
   (header.version   == curVersion) &&                     \
   (curVersion  == 1) &&                                   \
   (header.size == sizeof(NvPhysXToDrv_Data_V1))           \
  )

}

#endif	// __NVPHYSXTODRV_H__
