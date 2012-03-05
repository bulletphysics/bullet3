/*
		2011 Takahiro Harada
*/

template<>
class RadixSort<TYPE_HOST> : public RadixSortBase
{
	public:
		struct Data
		{
			HostBuffer<SortData>* m_workBuffer;
		};

		enum
		{
			BITS_PER_PASS = 8, 
			NUM_TABLES = (1<<BITS_PER_PASS),
		};
		
		static
		Data* allocate(const Device* deviceData, int maxSize, Option option = SORT_STANDARD)
		{
			ADLASSERT( deviceData->m_type == TYPE_HOST );

			Data* data = new Data;
			data->m_workBuffer = new HostBuffer<SortData>( deviceData, maxSize );
			return data;
		}

		static
		void deallocate(Data* data)
		{
			delete data->m_workBuffer;
			delete data;
		}

		static
		void execute(Data* data, Buffer<SortData>& inout, int n, int sortBits = 32)
		{
			ADLASSERT( inout.getType() == TYPE_HOST );

			int tables[NUM_TABLES];
			int counter[NUM_TABLES];

			SortData* src = inout.m_ptr;
			SortData* dst = data->m_workBuffer->m_ptr;

			int count=0;
			for(int startBit=0; startBit<sortBits; startBit+=BITS_PER_PASS)
			{
				for(int i=0; i<NUM_TABLES; i++)
				{
					tables[i] = 0;
				}

				for(int i=0; i<n; i++)
				{
					int tableIdx = (src[i].m_key >> startBit) & (NUM_TABLES-1);
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
					int tableIdx = (src[i].m_key >> startBit) & (NUM_TABLES-1);
			
					dst[tables[tableIdx] + counter[tableIdx]] = src[i];
					counter[tableIdx] ++;
				}

				swap2( src, dst );
				count++;
			}

			{
				if (count&1)
				//if( src != inout.m_ptr )
				{
					memcpy( dst, src, sizeof(SortData)*n );
				}
			}
		}
};
