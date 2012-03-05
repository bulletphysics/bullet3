/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Takahiro Harada

template<>
class Fill<TYPE_HOST>
{
	public:
		struct Data
		{
		};

		static
		Data* allocate(const Device* deviceData)
		{
			return 0;
		}

		static
		void deallocate(Data* data)
		{

		}

		template<typename T>
		static
		void executeImpl(Data* data, Buffer<T>& src, const T& value, int n, int offset = 0)
		{
			ADLASSERT( src.getType() == TYPE_HOST );
			ADLASSERT( src.m_size >= offset+n );
			HostBuffer<T>& hSrc = (HostBuffer<T>&)src;

			for(int idx=offset; idx<offset+n; idx++)
			{
				hSrc[idx] = value;
			}
		}

		static
		void execute(Data* data, Buffer<int>& src, const int& value, int n, int offset = 0)
		{
			executeImpl( data, src, value, n, offset );
		}

		static
		void execute(Data* data, Buffer<int2>& src, const int2& value, int n, int offset = 0)
		{
			executeImpl( data, src, value, n, offset );
		}

		static
		void execute(Data* data, Buffer<int4>& src, const int4& value, int n, int offset = 0)
		{
			executeImpl( data, src, value, n, offset );
		}

/*
		static
		void execute(Data* data, Buffer<int>& src, int value, int n, int offset = 0)
		{
			ADLASSERT( src.getType() == TYPE_HOST );
			ADLASSERT( src.m_size <= offset+n );
			HostBuffer<u32>& hSrc = (HostBuffer<u32>&)src;

			for(int idx=offset; idx<offset+n; idx++)
			{
				src[i] = value;
			}
		}

		static
		void execute(Data* data, Buffer<int2>& src, const int2& value, int n, int offset = 0)
		{
			ADLASSERT( src.getType() == TYPE_HOST );
			ADLASSERT( src.m_size <= offset+n );

		}

		static
		void execute(Data* data, Buffer<int4>& src, const int4& value, int n, int offset = 0)
		{
			ADLASSERT( src.getType() == TYPE_HOST );
			ADLASSERT( src.m_size <= offset+n );

		}
*/
};

