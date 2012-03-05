/*
		2011 Takahiro Harada
*/

#include <AdlPrimitives/Sort/RadixSortSimple.inl>
#include <AdlPrimitives/Sort/RadixSortStandard.inl>
#include <AdlPrimitives/Sort/RadixSortAdvanced.inl>


#define DISPATCH_IMPL(x) \
	switch( data->m_option ) \
	{ \
		case SORT_SIMPLE: RadixSortSimple<TYPE>::x; break; \
		case SORT_STANDARD: RadixSortStandard<TYPE>::x; break; \
		case SORT_ADVANCED: RadixSortAdvanced<TYPE>::x; break; \
		default:ADLASSERT(0);break; \
	}

template<DeviceType TYPE>
typename RadixSort<TYPE>::Data* RadixSort<TYPE>::allocate(const Device* deviceData, int maxSize, Option option)
{
	ADLASSERT( TYPE == deviceData->m_type );

	void* dataOut;
	switch( option )
	{
	case SORT_SIMPLE:
		dataOut = RadixSortSimple<TYPE>::allocate( deviceData, maxSize, option );
		break;
	case SORT_STANDARD:
		dataOut = RadixSortStandard<TYPE>::allocate( deviceData, maxSize, option );
		break;
	case SORT_ADVANCED:
		dataOut = RadixSortAdvanced<TYPE>::allocate( deviceData, maxSize, option );
		break;
	default:
		ADLASSERT(0);
		break;
	}
	return (typename RadixSort<TYPE>::Data*)dataOut;
}

template<DeviceType TYPE>
void RadixSort<TYPE>::deallocate(Data* data)
{
	DISPATCH_IMPL( deallocate( data ) );
}

template<DeviceType TYPE>
void RadixSort<TYPE>::execute(Data* data, Buffer<SortData>& inout, int n, int sortBits)
{
	DISPATCH_IMPL( execute( data, inout, n, sortBits ) );
}


#undef DISPATCH_IMPL


