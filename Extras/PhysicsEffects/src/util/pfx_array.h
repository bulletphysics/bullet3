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

#ifndef _SCE_PFX_ARRAY_H
#define _SCE_PFX_ARRAY_H

#include "../../include/physics_effects/base_level/base/pfx_common.h"
#include "pfx_util_common.h"

namespace sce {
namespace PhysicsEffects {

///////////////////////////////////////////////////////////////////////////////
// 基本的なコンテナクラス

///////////////////////////////////////////////////////////////////////////////
// PfxArray

/*
	可変配列
*/

template <class T>
class PfxArray
{
private:
	PfxUInt32 m_numData;
	PfxUInt32 m_maxData;
	SCE_PFX_PADDING(1,8)
	T SCE_PFX_ALIGNED(16) *m_data;
	SCE_PFX_PADDING(2,12)
	
public:
	PfxArray() : m_numData(0),m_maxData(100)
	{
		m_data = (T*)SCE_PFX_UTIL_ALLOC(16,sizeof(T)*m_maxData);
	}
	
	PfxArray(PfxUInt32 maxData) : m_numData(0),m_maxData(maxData)
	{
		m_data = (T*)SCE_PFX_UTIL_ALLOC(16,sizeof(T)*m_maxData);
	}
	
	~PfxArray()
	{
		SCE_PFX_UTIL_FREE(m_data);
	}
	
	PfxUInt32 size() const {return m_numData;}
	
	inline T& operator[](PfxUInt32 i);
	
	inline const T& operator[](PfxUInt32 i) const;
	
	inline const PfxArray& operator=(const PfxArray &array);
	
	// 指定数分のデータを準備する
	inline void assign(PfxUInt32 num,const T &initData);
	
	// データを追加
	inline PfxUInt32 push(const T& data);
	
	// i番目のデータを消去
	inline bool remove(PfxUInt32 i);
	
	inline bool empty() const {return m_numData==0;}
	
	inline void clear() {m_numData = 0;}
};

///////////////////////////////////////////////////////////////////////////////
// PfxQueue

/*
	キュー（最大数固定）
*/

template <class T>
class PfxQueue
{
private:
	PfxUInt32 m_numData;
	PfxUInt32 m_maxData;
	PfxUInt32 m_head;
	PfxUInt32 m_tail;
	T SCE_PFX_ALIGNED(16) *m_data;
	SCE_PFX_PADDING(1,12)
	
	PfxQueue() {}
	
public:
	
	PfxQueue(PfxUInt32 maxData) : m_numData(0),m_maxData(maxData),m_head(0),m_tail(0)
	{
		m_data = (T*)SCE_PFX_UTIL_ALLOC(16,sizeof(T)*m_maxData);
	}
	
	~PfxQueue()
	{
		SCE_PFX_UTIL_FREE(m_data);
	}
	
	PfxUInt32 size() const {return m_numData;}
	
	// データをキューに入れる
	inline PfxUInt32 push(const T& data);
	
	// 最後尾のデータを消去
	inline void pop();
	
	// 先頭のデータを参照
	inline T& front();
	
	// 先頭のデータを参照
	inline const T& front() const;
	
	inline bool empty() const {return m_numData==0;}
	
	inline void clear() {m_numData = 0;}
};

///////////////////////////////////////////////////////////////////////////////
// PfxStack

/*
	スタック（最大数固定）
*/

template <class T>
class PfxStack
{
private:
	PfxUInt32 m_numData;
	PfxUInt32 m_maxData;
	T SCE_PFX_ALIGNED(16) *m_data;
	
	PfxStack() {}
	
public:

	PfxStack(PfxUInt32 maxData) : m_numData(0),m_maxData(maxData)
	{
		m_data = (T*)SCE_PFX_UTIL_ALLOC(16,sizeof(T)*m_maxData);
	}
	
	~PfxStack()
	{
		SCE_PFX_UTIL_FREE(m_data);
	}
	
	PfxUInt32 size() const {return m_numData;}
	
	// データをスタックに入れる
	inline PfxUInt32 push(const T& data);
	
	// 末尾のデータを消去
	inline void pop();
	
	// 末尾のデータを参照
	inline T& top();
	
	// 末尾のデータを参照
	inline const T& top() const;
	
	inline bool empty() const {return m_numData==0;}
	
	inline void clear() {m_numData = 0;}
};

} //namespace PhysicsEffects
} //namespace sce
#include "pfx_array_implementation.h"

#endif // _SCE_PFX_ARRAY_H
