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
class PrefixScan<TYPE_HOST> : public PrefixScanBase
{
	public:
		struct Data
		{
			Option m_option;
		};

		static
		Data* allocate(const Device* deviceData, int maxSize, Option option = EXCLUSIVE)
		{
			ADLASSERT( deviceData->m_type == TYPE_HOST );

			Data* data = new Data;
			data->m_option = option;
			return data;
		}

		static
		void deallocate(Data* data)
		{
			delete data;
		}

		static
		void execute(Data* data, Buffer<u32>& src, Buffer<u32>& dst, int n, u32* sum = 0)
		{
			ADLASSERT( src.getType() == TYPE_HOST && dst.getType() == TYPE_HOST );
			HostBuffer<u32>& hSrc = (HostBuffer<u32>&)src;
			HostBuffer<u32>& hDst = (HostBuffer<u32>&)dst;

			u32 s = 0;
			if( data->m_option == EXCLUSIVE )
			{
				for(int i=0; i<n; i++)
				{
					hDst[i] = s;
					s += hSrc[i];
				}
			}
			else
			{
				for(int i=0; i<n; i++)
				{
					s += hSrc[i];
					hDst[i] = s;
				}
			}

			if( sum )
			{
				*sum = hDst[n-1];
			}
		}


};
