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
#ifndef PXPVDSDK_PXPVDDATASTREAMHELPERS_H
#define PXPVDSDK_PXPVDDATASTREAMHELPERS_H

/** \addtogroup pvd
@{
*/
#include "PxPvdObjectModelBaseTypes.h"

#if !PX_DOXYGEN
namespace physx
{
namespace pvdsdk
{
#endif

class PvdPropertyDefinitionHelper
{
  protected:
	virtual ~PvdPropertyDefinitionHelper()
	{
	}

  public:
	/**
	    Push a name c such that it appends such as a.b.c.
	*/
	virtual void pushName(const char* inName, const char* inAppendStr = ".") = 0;
	/**
	    Push a name c such that it appends like a.b[c]
	*/
	virtual void pushBracketedName(const char* inName, const char* leftBracket = "[", const char* rightBracket = "]") = 0;
	/**
	 *	Pop the current name
	 */
	virtual void popName() = 0;

	virtual void clearNameStack() = 0;
	/**
	 *	Get the current name at the top of the name stack.
	 *	Would return "a.b.c" or "a.b[c]" in the above examples.
	 */
	virtual const char* getTopName() = 0;

	virtual void addNamedValue(const char* name, uint32_t value) = 0;
	virtual void clearNamedValues() = 0;
	virtual DataRef<NamedValue> getNamedValues() = 0;

	/**
	 *	Define a property using the top of the name stack and the passed-in semantic
	 */
	virtual void createProperty(const NamespacedName& clsName, const char* inSemantic, const NamespacedName& dtypeName,
	                            PropertyType::Enum propType = PropertyType::Scalar) = 0;

	template <typename TClsType, typename TDataType>
	void createProperty(const char* inSemantic = "", PropertyType::Enum propType = PropertyType::Scalar)
	{
		createProperty(getPvdNamespacedNameForType<TClsType>(), inSemantic, getPvdNamespacedNameForType<TDataType>(),
		               propType);
	}

	// The datatype used for instances needs to be pointer unless you actually have pvdsdk::InstanceId members on your
	// value structs.
	virtual void addPropertyMessageArg(const NamespacedName& inDatatype, uint32_t inOffset, uint32_t inSize) = 0;

	template <typename TDataType>
	void addPropertyMessageArg(uint32_t offset)
	{
		addPropertyMessageArg(getPvdNamespacedNameForType<TDataType>(), offset, static_cast<uint32_t>(sizeof(TDataType)));
	}
	virtual void addPropertyMessage(const NamespacedName& clsName, const NamespacedName& msgName,
	                                uint32_t inStructSizeInBytes) = 0;
	template <typename TClsType, typename TMsgType>
	void addPropertyMessage()
	{
		addPropertyMessage(getPvdNamespacedNameForType<TClsType>(), getPvdNamespacedNameForType<TMsgType>(),
		                   static_cast<uint32_t>(sizeof(TMsgType)));
	}
	virtual void clearPropertyMessageArgs() = 0;

	void clearBufferedData()
	{
		clearNameStack();
		clearPropertyMessageArgs();
		clearNamedValues();
	}
};

#if !PX_DOXYGEN
} // pvdsdk
} // physx
#endif

/** @} */
#endif // PXPVDSDK_PXPVDDATASTREAMHELPERS_H
