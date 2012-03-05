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

///////////////////////////////////////////////////////////////////////////////
// PfxArray

namespace sce {
namespace PhysicsEffects {
template <class T>
inline T& PfxArray<T>::operator[](PfxUInt32 i)
{
	return m_data[i];
}

template <class T>
inline const T& PfxArray<T>::operator[](PfxUInt32 i) const
{
	return m_data[i];
}

template <class T>
inline const PfxArray<T>& PfxArray<T>::operator=(const PfxArray &array)
{
	clear();
	if(array.size() > m_maxData) {
		m_maxData = array.size();
		m_data = (T*)SCE_PFX_UTIL_REALLOC(m_data,16,sizeof(T)*m_maxData);
		SCE_PFX_ASSERT(m_data);
	}
	for(PfxUInt32 i=0;i<array.size();i++) {
		push(array[i]);
	}
	return *this;
}

template <class T>
inline void PfxArray<T>::assign(PfxUInt32 num,const T &initData)
{
	clear();
	if(num > m_maxData) {
		m_maxData = num;
		m_data = (T*)SCE_PFX_UTIL_REALLOC(m_data,16,sizeof(T)*m_maxData);
		SCE_PFX_ASSERT(m_data);
	}
	for(PfxUInt32 i=0;i<num;i++) {
		push(initData);
	}
}

template <class T>
inline PfxUInt32 PfxArray<T>::push(const T& data)
{
	if(m_numData == m_maxData) {
		m_maxData<<=1;
		m_data = (T*)SCE_PFX_UTIL_REALLOC(m_data,16,sizeof(T)*m_maxData);
		SCE_PFX_ASSERT(m_data);
	}
	
	PfxUInt32 id = m_numData++;
	m_data[id] = data;
	return id;
}

template <class T>
inline bool PfxArray<T>::remove(PfxUInt32 i)
{
	if(i>=m_numData) {
		return false;
	}
	
	m_numData--;
	m_data[i] = m_data[m_numData];
	
	return true;
}

///////////////////////////////////////////////////////////////////////////////
// PfxQueue

template <class T>
inline PfxUInt32 PfxQueue<T>::push(const T& data)
{
	SCE_PFX_ASSERT(m_numData<m_maxData);
	PfxUInt32 id = m_tail;
	m_data[id] = data;
	m_tail = (m_tail+1)%m_maxData;
	m_numData++;
	return id;
}

template <class T>
inline void PfxQueue<T>::pop()
{
	SCE_PFX_ASSERT(m_numData>0);
	m_head = (m_head+1)%m_maxData;
	m_numData--;
}

template <class T>
inline T& PfxQueue<T>::front()
{
	SCE_PFX_ASSERT(m_numData>0);
	return m_data[m_head];
}

template <class T>
inline const T& PfxQueue<T>::front() const
{
	SCE_PFX_ASSERT(m_numData>0);
	return m_data[m_head];
}

///////////////////////////////////////////////////////////////////////////////
// PfxStack

template <class T>
inline PfxUInt32 PfxStack<T>::push(const T& data)
{
	SCE_PFX_ASSERT(m_numData<m_maxData);
	PfxUInt32 id = m_numData++;
	m_data[id] = data;
	return id;
}

template <class T>
inline void PfxStack<T>::pop()
{
	SCE_PFX_ASSERT(m_numData>0);
	m_numData--;
}

template <class T>
inline T& PfxStack<T>::top()
{
	SCE_PFX_ASSERT(m_numData>0);
	return m_data[m_numData-1];
}

template <class T>
inline const T& PfxStack<T>::top() const
{
	SCE_PFX_ASSERT(m_numData>0);
	return m_data[m_numData-1];
}
} //namespace PhysicsEffects
} //namespace sce
