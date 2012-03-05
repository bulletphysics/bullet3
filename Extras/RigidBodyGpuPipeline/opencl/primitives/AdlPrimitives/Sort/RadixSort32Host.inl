/*
		2011 Takahiro Harada
*/

template<>
class RadixSort32<TYPE_HOST> : public RadixSort32Base
{
	public:
		typedef Launcher::BufferInfo BufferInfo;

		enum
		{
			BITS_PER_PASS = 8,
			NUM_TABLES = (1<<BITS_PER_PASS),
		};

		struct Data
		{
			HostBuffer<u32>* m_workBuffer;
		};

		static
		Data* allocate(const Device* device, int maxSize)
		{
			ADLASSERT( device->m_type == TYPE_HOST );

			Data* data = new Data;
			data->m_workBuffer = new HostBuffer<u32>( device, maxSize );
			return data;
		}

		static
		void deallocate(Data* data)
		{
			delete data->m_workBuffer;
			delete data;
		}

		static
		void execute(Data* data, Buffer<u32>& inout, int n, int sortBits = 32)
		{
			ADLASSERT( inout.getType() == TYPE_HOST );

			int tables[NUM_TABLES];
			int counter[NUM_TABLES];

			u32* src = inout.m_ptr;
			u32* dst = data->m_workBuffer->m_ptr;

			for(int startBit=0; startBit<sortBits; startBit+=BITS_PER_PASS)
			{
				for(int i=0; i<NUM_TABLES; i++)
				{
					tables[i] = 0;
				}

				for(int i=0; i<n; i++)
				{
					int tableIdx = (src[i] >> startBit) & (NUM_TABLES-1);
					tables[tableIdx]++;
				}

				//	prefix scan
				int sum = 0;
				for(int i=0; i<NUM_TABLES; i++)
				{
					int iData = tables[i];
					tables[i] = sum;
					sum += iData;
					counter[i] = 0;
				}

				//	distribute
				for(int i=0; i<n; i++)
				{
					int tableIdx = (src[i] >> startBit) & (NUM_TABLES-1);

					dst[tables[tableIdx] + counter[tableIdx]] = src[i];
					counter[tableIdx] ++;
				}

				swap2( src, dst );
			}

			{
				if( src != inout.m_ptr )
				{
					memcpy( dst, src, sizeof(u32)*n );
				}
			}

		}

		static
		void execute(Data* data, Buffer<u32>& keyInout, const Buffer<u32>& valueInout, int n, int sortBits = 32)
		{
			ADLASSERT( keyInout.getType() == TYPE_HOST );

			int tables[NUM_TABLES];
			int counter[NUM_TABLES];

			u32* src = keyInout.m_ptr;
			u32* dst = data->m_workBuffer->m_ptr;

			HostBuffer<u32> bufVal(valueInout.m_device, valueInout.m_size);
			bufVal.write(valueInout.m_ptr, valueInout.m_size);

			u32* srcVal = valueInout.m_ptr;
			u32* dstVal = bufVal.m_ptr;

			for(int startBit=0; startBit<sortBits; startBit+=BITS_PER_PASS)
			{
				for(int i=0; i<NUM_TABLES; i++)
				{
					tables[i] = 0;
				}

				for(int i=0; i<n; i++)
				{
					int tableIdx = (src[i] >> startBit) & (NUM_TABLES-1);
					tables[tableIdx]++;
				}

				//	prefix scan
				int sum = 0;
				for(int i=0; i<NUM_TABLES; i++)
				{
					int iData = tables[i];
					tables[i] = sum;
					sum += iData;
					counter[i] = 0;
				}

				//	distribute
				for(int i=0; i<n; i++)
				{
					int tableIdx = (src[i] >> startBit) & (NUM_TABLES-1);
					int newIdx = tables[tableIdx] + counter[tableIdx];
					dst[newIdx] = src[i];
					dstVal[newIdx] = srcVal[i];
					counter[tableIdx]++;
				}

				swap2( src, dst );
				swap2( srcVal, dstVal );
			}

			{
				if( src != keyInout.m_ptr )
				{
					memcpy( dst, src, sizeof(u32)*n );
				}

				if( srcVal != valueInout.m_ptr )
				{
					memcpy( dstVal, srcVal, sizeof(u32)*n );
				}
			}

		}
};


