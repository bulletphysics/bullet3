
template<DeviceType TYPE>
void run( Device* device, int minSize = 512, int maxSize = 64*1024 )//, int increment = 512 )
{
	ADLASSERT( TYPE == device->m_type );

	Stopwatch sw( device );

//	RadixSort<TYPE>::Data* data0 = RadixSort<TYPE>::allocate( device, maxSize, RadixSortBase::SORT_SIMPLE );
	RadixSort<TYPE>::Data* data0 = RadixSort<TYPE>::allocate( device, maxSize, RadixSortBase::SORT_STANDARD );
	RadixSort<TYPE>::Data* data1 = RadixSort<TYPE>::allocate( device, maxSize, RadixSortBase::SORT_STANDARD );
	RadixSort<TYPE>::Data* data2 = RadixSort<TYPE>::allocate( device, maxSize, RadixSortBase::SORT_ADVANCED );

	Buffer<SortData> buf0( device, maxSize );
	Buffer<SortData> buf1( device, maxSize );
	Buffer<SortData> buf2( device, maxSize );

	SortData* input = new SortData[ maxSize ];

//	for(int iter = minSize; iter<=maxSize; iter+=increment)
	for(int iter = minSize; iter<=maxSize; iter*=2)
	{
		int size = NEXTMULTIPLEOF( iter, 512 );

		for(int i=0; i<size; i++) input[i] = SortData( getRandom(0,0xff), i );

		buf0.write( input, size );
		buf1.write( input, size );
		buf2.write( input, size );
		DeviceUtils::waitForCompletion( device );


		sw.start();

		RadixSort<TYPE>::execute( data0, buf0, size );

		sw.split();

		RadixSort<TYPE>::execute( data1, buf1, size );

		sw.split();

		RadixSort<TYPE>::execute( data2, buf2, size );

		sw.stop();


		float t[3];
		sw.getMs( t, 3 );
//		printf("	%d	%3.2f	%3.2f	%3.2f\n", size, t[0], t[1], t[2]);
		printf("	%d	%3.2f	%3.2f\n", size, t[1], t[2]);
	}

	RadixSort<TYPE>::deallocate( data0 );
	RadixSort<TYPE>::deallocate( data1 );
	RadixSort<TYPE>::deallocate( data2 );

	delete [] input;
}

template<DeviceType TYPE>
void run32( Device* device, int size )
{
	//Cayman: 4194.30Keys:    373.05MKeys/s
	//Cypress: 4194.30Keys:    315.13MKeys/s
	ADLASSERT( TYPE == device->m_type );

	Stopwatch sw( device );

	RadixSort32<TYPE>::Data* data = RadixSort32<TYPE>::allocate( device, size );
	Copy<TYPE>::Data* copyData = Copy<TYPE>::allocate( device );

	Buffer<u32> inputMaster( device, size );
	Buffer<u32> input( device, size );
	Buffer<u32> output( device, size );
	{
		u32* host = new u32[size];
		for(int i=0; i<size; i++) host[i] = getRandom(0u, 0xffffffffu);
		inputMaster.write( host, size );
		DeviceUtils::waitForCompletion( device );
		delete [] host;
	}

	int nIter = 100;
	sw.start();
	for(int iter=0; iter<nIter; iter++)
	{
//		Copy<TYPE>::execute( copyData, (Buffer<float>&)input, (Buffer<float>&)inputMaster, size );
//		RadixSort32<TYPE>::execute( data, input, size );
		RadixSort32<TYPE>::execute( data, input, output, size );
	}
	sw.stop();

	{
		float tInS = sw.getMs()/1000.f/(float)nIter;
		float mKeysPerS = size/1000.f/1000.f/tInS;
		printf("%3.2fMKeys:	%3.2fMKeys/s\n", size/1000.f, mKeysPerS);
	}

	RadixSort32<TYPE>::deallocate( data );
	Copy<TYPE>::deallocate( copyData );
}

template<DeviceType TYPE>
void radixSortBenchmark()
{

	Device* device;
	{
		DeviceUtils::Config cfg;
		device = DeviceUtils::allocate( TYPE, cfg );
	}

	run32<TYPE>( device, 256*1024*8*2 );
//	run32<TYPE>( device, 256*20*6 );

//	run<TYPE>( device, 512, 1024*128*4 );

	DeviceUtils::deallocate( device );

}
