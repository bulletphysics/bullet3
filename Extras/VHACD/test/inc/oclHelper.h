/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.
 
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 
 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifdef OPENCL_FOUND

#pragma once
#ifndef OCL_HELPER_H
#define OCL_HELPER_H

#include <string>
#include <vector>

#ifdef __MACH__
#include <OpenCL/cl.h>
#else
#include <CL/cl.h>
#endif

class OCLHelper {
public:
    OCLHelper(void){};
    ~OCLHelper(void){};
    bool InitPlatform(const unsigned int platformIndex = 0);
    bool InitDevice(const unsigned int deviceIndex);
    bool GetPlatformsInfo(std::vector<std::string>& info, const std::string& indentation);
    bool GetDevicesInfo(std::vector<std::string>& info, const std::string& indentation);
    cl_platform_id* GetPlatform() { return &m_platform; }
    const cl_platform_id* GetPlatform() const { return &m_platform; }
    cl_device_id* GetDevice() { return &m_device; }
    const cl_device_id* GetDevice() const { return &m_device; }
private:
    cl_platform_id m_platform;
    cl_device_id m_device;
    cl_int m_lastError;
};

#endif // OCL_HELPER_H

#endif //OPENCL_FOUND
