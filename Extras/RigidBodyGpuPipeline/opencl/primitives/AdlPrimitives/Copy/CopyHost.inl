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
class Copy<TYPE_HOST> : public CopyBase
{
	public:
		typedef Launcher::BufferInfo BufferInfo;

		struct Data
		{
		};

		static
		Data* allocate(const Device* deviceData)
		{
			ADLASSERT( TYPE_HOST == deviceData->m_type );
			return 0;
		}

		static
		void deallocate(Data* data)
		{
			return;
		}

		static
		void execute( Data* data, Buffer<float4>& dst, Buffer<float4>& src, int n, Option option = PER_WI_1)
		{
			ADLASSERT( TYPE_HOST == dst.getType() );
			ADLASSERT( TYPE_HOST == src.getType() );

			HostBuffer<float4>& dstH = (HostBuffer<float4>&)dst;
			HostBuffer<float4>& srcH = (HostBuffer<float4>&)src;

			for(int i=0; i<n; i++)
			{
				dstH[i] = srcH[i];
			}
		}

		static
		void execute( Data* data, Buffer<float2>& dst, Buffer<float2>& src, int n)
		{
			ADLASSERT( TYPE_HOST == dst.getType() );
			ADLASSERT( TYPE_HOST == src.getType() );

			HostBuffer<float2>& dstH = (HostBuffer<float2>&)dst;
			HostBuffer<float2>& srcH = (HostBuffer<float2>&)src;

			for(int i=0; i<n; i++)
			{
				dstH[i] = srcH[i];
			}
		}

		static
		void execute( Data* data, Buffer<float>& dst, Buffer<float>& src, int n)
		{
			ADLASSERT( TYPE_HOST == dst.getType() );
			ADLASSERT( TYPE_HOST == src.getType() );

			HostBuffer<float>& dstH = (HostBuffer<float>&)dst;
			HostBuffer<float>& srcH = (HostBuffer<float>&)src;

			for(int i=0; i<n; i++)
			{
				dstH[i] = srcH[i];
			}
		}
};

