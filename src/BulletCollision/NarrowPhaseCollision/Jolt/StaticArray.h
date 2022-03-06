// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once
#include <assert.h>
namespace BTJPH {

/// Simple variable length array backed by a fixed size buffer
template <class T, uint N>
class [[nodiscard]] StaticArray
{
public:
	using value_type = T;

	using size_type = uint;

	/// Default constructor
						StaticArray()
	{
	}

#if 0
	/// Constructor from initializer list
	explicit			StaticArray(initializer_list<T> inList)
	{
		assert(inList.size() <= N);
		for (typename initializer_list<T>::iterator i = inList.begin(); i != inList.end(); ++i)
			new (reinterpret_cast<T *>(&mElements[mSize++])) T(*i);
	}
#endif
	/// Copy constructor
						StaticArray(const StaticArray<T, N> &inRHS)
	{
		while (mSize < inRHS.mSize)
		{
			new (&mElements[mSize]) T(inRHS[mSize]);
			++mSize;
		}
	}

	/// Destruct all elements
						~StaticArray()
	{
		//if (!is_trivially_destructible<T>())
			for (T *e = reinterpret_cast<T *>(mElements), *end = e + mSize; e < end; ++e)
				e->~T();
	}

	/// Destruct all elements and set length to zero
	void				clear()
	{
		if (!is_trivially_destructible<T>())
			for (T *e = reinterpret_cast<T *>(mElements), *end = e + mSize; e < end; ++e)
				e->~T();
		mSize = 0;
	}

	/// Add element to the back of the array
	void				push_back(const T &inElement)
	{
		assert(mSize < N);
		new (&mElements[mSize++]) T(inElement);
	}

	/// Construct element at the back of the array
	template <class... A>
	void				emplace_back(A &&... inElement)
	{	
		assert(mSize < N);
		new (&mElements[mSize++]) T(forward<A>(inElement)...);
	}

	/// Remove element from the back of the array
	void				pop_back()
	{
		assert(mSize > 0);
		reinterpret_cast<T &>(mElements[--mSize]).~T();
	}

	/// Returns true if there are no elements in the array
	bool				empty() const
	{
		return mSize == 0;
	}

	/// Returns amount of elements in the array
	size_type			size() const
	{
		return mSize;
	}

	/// Returns maximum amount of elements the array can hold
	size_type			capacity() const
	{
		return N;
	}

	/// Resize array to new length
	void				resize(size_type inNewSize)
	{
		assert(inNewSize <= N);
		if (!is_trivially_constructible<T>() && mSize < inNewSize)
			for (T *element = reinterpret_cast<T *>(mElements) + mSize, *element_end = reinterpret_cast<T *>(mElements) + inNewSize; element < element_end; ++element)
				new (element) T;
		else if (!is_trivially_destructible<T>() && mSize > inNewSize)
			for (T *element = reinterpret_cast<T *>(mElements) + inNewSize, *element_end = reinterpret_cast<T *>(mElements) + mSize; element < element_end; ++element)
				element->~T();
		mSize = inNewSize;
	}

	using const_iterator = const T *;

	/// Iterators
	const_iterator		begin() const
	{
		return reinterpret_cast<const T *>(mElements);
	}

	const_iterator		end() const
	{
		return reinterpret_cast<const T *>(mElements + mSize);
	}

	using iterator = T *;

	iterator			begin()
	{
		return reinterpret_cast<T *>(mElements);
	}

	iterator			end()
	{
		return reinterpret_cast<T *>(mElements + mSize);
	}

	const T *			data() const
	{
		return reinterpret_cast<const T *>(mElements);
	}

	T *					data()
	{
		return reinterpret_cast<T *>(mElements);
	}

	/// Access element
	T &					operator [] (size_type inIdx)
	{
		assert(inIdx < mSize);
		return reinterpret_cast<T &>(mElements[inIdx]);
	}

	const T &			operator [] (size_type inIdx) const
	{
		assert(inIdx < mSize);
		return reinterpret_cast<const T &>(mElements[inIdx]);
	}

	/// First element in the array
	const T &			front() const
	{
		assert(mSize > 0);
		return reinterpret_cast<const T &>(mElements[0]);
	}

	T &					front()
	{
		assert(mSize > 0);
		return reinterpret_cast<T &>(mElements[0]);
	}

	/// Last element in the array
	const T &			back() const
	{
		assert(mSize > 0);
		return reinterpret_cast<const T &>(mElements[mSize - 1]);
	}

	T &					back()
	{
		assert(mSize > 0);
		return reinterpret_cast<T &>(mElements[mSize - 1]);
	}

	/// Remove one element from the array
	void				erase(const_iterator inIter)
	{
		size_type p = size_type(inIter - begin());
		assert(p < mSize);
		reinterpret_cast<T &>(mElements[p]).~T();
		if (p + 1 < mSize)
			memmove(mElements + p, mElements + p + 1, (mSize - p - 1) * sizeof(T));
		--mSize;
	}

	/// Remove multiple element from the array
	void				erase(const_iterator inBegin, const_iterator inEnd)
	{
		size_type p = size_type(inBegin - begin());
		size_type n = size_type(inEnd - inBegin);
		assert(inEnd <= end());
		for (size_type i = 0; i < n; ++i)
			reinterpret_cast<T &>(mElements[p + i]).~T();
		if (p + n < mSize)
			memmove(mElements + p, mElements + p + n, (mSize - p - n) * sizeof(T));
		mSize -= n;
	}

	/// Assignment operator
	StaticArray<T, N> &	operator = (const StaticArray<T, N> &inRHS)
	{
		size_type rhs_size = inRHS.size();

		if ((void *)this != (void *)&inRHS)
		{
			clear();

			while (mSize < rhs_size)
			{
				new (&mElements[mSize]) T(inRHS[mSize]);
				++mSize;
			}
		}

		return *this;
	}

	/// Assignment operator with static array of different max length
	template <uint M>
	StaticArray<T, N> &	operator = (const StaticArray<T, M> &inRHS)
	{
		size_type rhs_size = inRHS.size();
		assert(rhs_size <= N);

		if ((void *)this != (void *)&inRHS)
		{
			clear();

			while (mSize < rhs_size)
			{
				new (&mElements[mSize]) T(inRHS[mSize]);
				++mSize;
			}
		}

		return *this;
	}
	
	/// Comparing arrays
	bool				operator == (const StaticArray<T, N> &inRHS) const
	{
		if (mSize != inRHS.mSize)
			return false;
		for (size_type i = 0; i < mSize; ++i)
			if (!(reinterpret_cast<const T &>(mElements[i]) == reinterpret_cast<const T &>(inRHS.mElements[i])))
				return false;
		return true;
	}

	bool				operator != (const StaticArray<T, N> &inRHS) const
	{
		if (mSize != inRHS.mSize)
			return true;
		for (size_type i = 0; i < mSize; ++i)
			if (reinterpret_cast<const T &>(mElements[i]) != reinterpret_cast<const T &>(inRHS.mElements[i]))
				return true;
		return false;
	}
	
protected:
	struct alignas(T) Storage
	{
		jUint8			mData[sizeof(T)];
	};

	static_assert(sizeof(T) == sizeof(Storage), "Mismatch in size");
	static_assert(alignof(T) == alignof(Storage), "Mismatch in alignment");

	size_type			mSize = 0;
	Storage				mElements[N];
};

} // BTJPH

namespace std
{
	/// Declare std::hash for StaticArray
	template <class T, BTJPH::uint N>
	struct hash<BTJPH::StaticArray<T, N>>
	{
		size_t operator () (const BTJPH::StaticArray<T, N> &inRHS) const
		{
			std::size_t ret = 0;

			// Hash length first
            BTJPH::hash_combine(ret, inRHS.size());

			// Then hash elements
			for (const T &t : inRHS)
	            BTJPH::hash_combine(ret, t);

            return ret;
		}
	};
}
