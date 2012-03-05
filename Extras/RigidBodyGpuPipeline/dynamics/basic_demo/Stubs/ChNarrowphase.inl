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


//#define PATH "..\\..\\dynamics\\basic_demo\\Stubs\\ChNarrowphaseKernels"
#define PATH "..\\..\\dynamics\\basic_demo\\Stubs\\ChNarrowphaseKernels"
#define KERNEL0 "SupportCullingKernel"
#define KERNEL1 "NarrowphaseKernel"

#include "ChNarrowphaseKernels.h"

class ChNarrowphaseImp
{
public:
	static
	__inline
	u32 u32Pack(u8 x, u8 y, u8 z, u8 w)
	{
		return (x) | (y<<8) | (z<<16) | (w<<24);
	}

};

template<DeviceType TYPE>
typename ChNarrowphase<TYPE>::Data* ChNarrowphase<TYPE>::allocate( const Device* device )
{
	char options[100];
	
	const char* src[] = 
#if defined(ADL_LOAD_KERNEL_FROM_STRING)
		{narrowphaseKernelsCL, 0};
#else
		{0,0};
#endif

	


	//sprintf(options, "-I ..\\..\\ -Wf,--c++");
	sprintf(options, "-I .\\NarrowPhaseCL\\");

	Data* data = new Data;
	data->m_device = device;
	data->m_supportCullingKernel = device->getKernel( PATH, KERNEL0, options,src[TYPE] );
	data->m_narrowphaseKernel = device->getKernel( PATH, KERNEL1, options, src[TYPE]);
	data->m_narrowphaseWithPlaneKernel = device->getKernel( PATH, "NarrowphaseWithPlaneKernel", options,src[TYPE]);
	data->m_counterBuffer = new Buffer<u32>( device, 1 );

	return data;
}


template<DeviceType TYPE>
void ChNarrowphase<TYPE>::deallocate( Data* data )
{
	delete data->m_counterBuffer;

	delete data;
}

template<DeviceType TYPE>
ShapeDataType ChNarrowphase<TYPE>::allocateShapeBuffer( const Device* device, int capacity )
{
	ADLASSERT( device->m_type == TYPE );

	return new Buffer<ShapeData>( device, capacity );
}

template<DeviceType TYPE>
void ChNarrowphase<TYPE>::deallocateShapeBuffer( ShapeDataType shapeBuf )
{
	Buffer<ShapeData>* s = (Buffer<ShapeData>*)shapeBuf;
	delete s;
}

template<DeviceType TYPE>
void ChNarrowphase<TYPE>::setShape( ShapeDataType shapeBuf, ShapeBase* shape, int idx, float collisionMargin )
{
	ConvexHeightField* cvxShape = new ConvexHeightField( shape );
	Buffer<ShapeData>* dst = (Buffer<ShapeData>*)shapeBuf;
	cvxShape->m_aabb.expandBy( make_float4( collisionMargin ) );
	{
		ShapeData s;
		{
			for(int j=0; j<HEIGHT_RES*HEIGHT_RES*6; j++)
			{
				s.m_normal[j] = cvxShape->m_normal[j];
			}
			for(int j=0; j<HEIGHT_RES*HEIGHT_RES*6/4; j++)
			{
				s.m_height4[j] = ChNarrowphaseImp::u32Pack( cvxShape->m_data[4*j], cvxShape->m_data[4*j+1], cvxShape->m_data[4*j+2], cvxShape->m_data[4*j+3] );
				s.m_supportHeight4[j] = ChNarrowphaseImp::u32Pack( cvxShape->m_supportHeight[4*j], cvxShape->m_supportHeight[4*j+1], cvxShape->m_supportHeight[4*j+2], cvxShape->m_supportHeight[4*j+3] );
			}
			s.m_scale = cvxShape->m_scale;
		}
		dst->write( &s, 1, idx );
		DeviceUtils::waitForCompletion( dst->m_device );
	}
	delete cvxShape;
}

template<DeviceType TYPE>
void ChNarrowphase<TYPE>::setShape( ShapeDataType shapeBuf, ConvexHeightField* cvxShape, int idx, float collisionMargin )
{
	Buffer<ShapeData>* dst = (Buffer<ShapeData>*)shapeBuf;
	cvxShape->m_aabb.expandBy( make_float4( collisionMargin ) );
	{
		ShapeData s;
		{
			for(int j=0; j<HEIGHT_RES*HEIGHT_RES*6; j++)
			{
				s.m_normal[j] = cvxShape->m_normal[j];
			}
			for(int j=0; j<HEIGHT_RES*HEIGHT_RES*6/4; j++)
			{
				s.m_height4[j] = ChNarrowphaseImp::u32Pack( cvxShape->m_data[4*j], cvxShape->m_data[4*j+1], cvxShape->m_data[4*j+2], cvxShape->m_data[4*j+3] );
				s.m_supportHeight4[j] = ChNarrowphaseImp::u32Pack( cvxShape->m_supportHeight[4*j], cvxShape->m_supportHeight[4*j+1], cvxShape->m_supportHeight[4*j+2], cvxShape->m_supportHeight[4*j+3] );
			}
			s.m_scale = cvxShape->m_scale;
		}
		dst->write( &s, 1, idx );
		DeviceUtils::waitForCompletion( dst->m_device );
	}
}

// Run NarrowphaseKernel
template<DeviceType TYPE>
//template<bool USE_OMP>
void ChNarrowphase<TYPE>::execute( Data* data, const Buffer<int2>* pairs, int nPairs, const Buffer<RigidBodyBase::Body>* bodyBuf,
			const ShapeDataType shapeBuf,
			Buffer<Contact4>* contactOut, int& nContacts, const Config& cfg )
{
	if( nPairs == 0 ) return;

	Buffer<ShapeData>* shapeBuffer = (Buffer<ShapeData>*)shapeBuf;
	ADLASSERT( shapeBuffer->getType() == TYPE );

	const Device* device = data->m_device;

	Buffer<int2>* gPairsInNative 
		= BufferUtils::map<TYPE, true>( data->m_device, pairs );
	Buffer<RigidBodyBase::Body>* gBodyInNative 
		= BufferUtils::map<TYPE, true>( data->m_device, bodyBuf );
	Buffer<Contact4>* gContactOutNative 
		= BufferUtils::map<TYPE, true>( data->m_device, contactOut );	//	this might not be empty

	Buffer<ConstData> constBuffer( device, 1, BufferBase::BUFFER_CONST );

	ConstData cdata;
	cdata.m_nPairs = nPairs;
	cdata.m_collisionMargin = cfg.m_collisionMargin;
	cdata.m_capacity = contactOut->getSize() - nContacts;

	u32 n = nContacts;
	data->m_counterBuffer->write( &n, 1 );
//	DeviceUtils::waitForCompletion( device );

	{
		BufferInfo bInfo[] = { BufferInfo( gPairsInNative, true ), BufferInfo( shapeBuffer ), BufferInfo( gBodyInNative ), 
			BufferInfo( gContactOutNative ),
			BufferInfo( data->m_counterBuffer ) };
		Launcher launcher( data->m_device, data->m_narrowphaseKernel );
		launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
		launcher.setConst( constBuffer, cdata );
		launcher.launch1D( nPairs*64, 64 );
	}

	data->m_counterBuffer->read( &n, 1 );
	DeviceUtils::waitForCompletion( device );

	BufferUtils::unmap<false>( gPairsInNative, pairs );
	BufferUtils::unmap<false>( gBodyInNative, bodyBuf );
	BufferUtils::unmap<true>( gContactOutNative, contactOut );

	nContacts = min2((int)n, contactOut->getSize() );
}

// Run NarrowphaseWithPlaneKernel
template<DeviceType TYPE>
//template<bool USE_OMP>
void ChNarrowphase<TYPE>::execute( Data* data, const Buffer<int2>* pairs, int nPairs, 
			const Buffer<RigidBodyBase::Body>* bodyBuf, const ShapeDataType shapeBuf,
			const Buffer<float4>* vtxBuf, const Buffer<int4>* idxBuf,
			Buffer<Contact4>* contactOut, int& nContacts, const Config& cfg )
{
	if( nPairs == 0 ) return;

	Buffer<ShapeData>* shapeBuffer = (Buffer<ShapeData>*)shapeBuf;
	ADLASSERT( shapeBuffer->getType() == TYPE );

	const Device* device = data->m_device;

	Buffer<int2>* gPairsInNative 
		= BufferUtils::map<TYPE, true>( data->m_device, pairs );
	Buffer<RigidBodyBase::Body>* gBodyInNative 
		= BufferUtils::map<TYPE, true>( data->m_device, bodyBuf );	
	Buffer<Contact4>* gContactOutNative 
		= BufferUtils::map<TYPE, true>( data->m_device, contactOut );	//	this might not be empty

	Buffer<ConstData> constBuffer( device, 1, BufferBase::BUFFER_CONST );

	ConstData cdata;
	cdata.m_nPairs = nPairs;
	cdata.m_collisionMargin = cfg.m_collisionMargin;
	cdata.m_capacity = contactOut->getSize() - nContacts;

	u32 n = nContacts;
	data->m_counterBuffer->write( &n, 1 );
//	DeviceUtils::waitForCompletion( device );

	{
		BufferInfo bInfo[] = { BufferInfo( gPairsInNative, true ), BufferInfo( shapeBuffer ), BufferInfo( gBodyInNative ), 
			BufferInfo( gContactOutNative ),
			BufferInfo( data->m_counterBuffer ) };
		Launcher launcher( data->m_device, data->m_narrowphaseWithPlaneKernel );
		launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
		launcher.setConst( constBuffer, cdata );
		launcher.launch1D( nPairs*64, 64 );
	}

	data->m_counterBuffer->read( &n, 1 );
	DeviceUtils::waitForCompletion( device );

	BufferUtils::unmap<false>( gPairsInNative, pairs );
	BufferUtils::unmap<false>( gBodyInNative, bodyBuf );
	BufferUtils::unmap<true>( gContactOutNative, contactOut );

	nContacts = min2((int)n, contactOut->getSize() );
}

// Run SupportCullingKernel
template<DeviceType TYPE>
//template<bool USE_OMP>
int ChNarrowphase<TYPE>::culling( Data* data, const Buffer<int2>* pairs, int nPairs, const Buffer<RigidBodyBase::Body>* bodyBuf,
			const ShapeDataType shapeBuf, const Buffer<int2>* pairsOut, const Config& cfg )
{
	if( nPairs == 0 ) return 0;

	Buffer<ShapeData>* shapeBuffer = (Buffer<ShapeData>*)shapeBuf;
	ADLASSERT( shapeBuffer->getType() == TYPE );

	const Device* device = data->m_device;

	Buffer<int2>* gPairsInNative 
		= BufferUtils::map<TYPE, true>( data->m_device, pairs );
	Buffer<RigidBodyBase::Body>* gBodyInNative 
		= BufferUtils::map<TYPE, true>( data->m_device, bodyBuf );	
	Buffer<int2>* gPairsOutNative 
		= BufferUtils::map<TYPE, false>( data->m_device, pairsOut );

	//
	Buffer<ConstData> constBuffer( device, 1, BufferBase::BUFFER_CONST );

	ConstData cdata;
	cdata.m_nPairs = nPairs;
	cdata.m_collisionMargin = cfg.m_collisionMargin;
	cdata.m_capacity = pairsOut->getSize();

	u32 n = 0;
	data->m_counterBuffer->write( &n, 1 );
//	DeviceUtils::waitForCompletion( device );
	{
		BufferInfo bInfo[] = { BufferInfo( gPairsInNative, true ), BufferInfo( shapeBuffer ), BufferInfo( gBodyInNative ), 
			BufferInfo( gPairsOutNative ), BufferInfo( data->m_counterBuffer ) };
		Launcher launcher( data->m_device, data->m_supportCullingKernel );
		launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(Launcher::BufferInfo) );
		launcher.setConst( constBuffer, cdata );
		launcher.launch1D( nPairs, 64 );
	}
	data->m_counterBuffer->read( &n, 1 );
	DeviceUtils::waitForCompletion( device );
/*
	if( gPairsInNative != pairs ) delete gPairsInNative;
	if( gBodyInNative != bodyBuf ) delete gBodyInNative;
	if( gPairsOutNative != pairsOut ) 
	{
		gPairsOutNative->read( pairsOut->m_ptr, n );
		DeviceUtils::waitForCompletion( device );
		delete gPairsOutNative;
	}
*/
	BufferUtils::unmap<false>( gPairsInNative, pairs );
	BufferUtils::unmap<false>( gBodyInNative, bodyBuf );
	BufferUtils::unmap<true>( gPairsOutNative, pairsOut );

	return min2((int)n, pairsOut->getSize() );
}

#undef PATH
#undef KERNEL0
#undef KERNEL1