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
#ifndef PX_REPX_UPGRADER_H
#define PX_REPX_UPGRADER_H

#include "foundation/PxSimpleTypes.h"

namespace physx { namespace Sn {
	class RepXCollection;

	class RepXUpgrader
	{
	public:
		//If a new collection is created, the source collection is destroyed.
		//Thus you only need to release the new collection.
		//This holds for all of the upgrade functions.
		//So be aware, that the argument to these functions may not be valid
		//after they are called, but the return value always will be valid.
		static RepXCollection& upgradeCollection( RepXCollection& src );
		static RepXCollection& upgrade10CollectionTo3_1Collection( RepXCollection& src );
		static RepXCollection& upgrade3_1CollectionTo3_2Collection( RepXCollection& src );
		static RepXCollection& upgrade3_2CollectionTo3_3Collection( RepXCollection& src );
		static RepXCollection& upgrade3_3CollectionTo3_4Collection( RepXCollection& src );
		static RepXCollection& upgrade3_4CollectionTo4_0Collection( RepXCollection& src );
	};
} }

#endif
