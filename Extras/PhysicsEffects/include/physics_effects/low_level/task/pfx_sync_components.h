/*
Physics Effects Copyright(C) 2010 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#ifndef _SCE_PFX_SYNC_COMPONENTS_H
#define _SCE_PFX_SYNC_COMPONENTS_H

#include "../../base_level/base/pfx_common.h"

//J スレッド間の同期をとるための同期コンポネント
//E Components for threads sychronization
namespace sce {
namespace PhysicsEffects {

class PfxBarrier {
public:
	PfxBarrier() {}
	virtual ~PfxBarrier() {}

	virtual void sync() = 0;
	virtual void setMaxCount(int n) = 0;
	virtual int  getMaxCount() = 0;
};

class PfxCriticalSection {
public:
	PfxCriticalSection() {}
	virtual ~PfxCriticalSection() {}

PfxUInt32 m_commonBuff[32];

	virtual PfxUInt32 getSharedParam(int i) = 0;
	virtual void setSharedParam(int i,PfxUInt32 p) = 0;

	virtual void lock() = 0;
	virtual void unlock() = 0;
};

} //namespace PhysicsEffects
} //namespace sce
#endif // _SCE_PFX_SYNC_COMPONENTS_H
