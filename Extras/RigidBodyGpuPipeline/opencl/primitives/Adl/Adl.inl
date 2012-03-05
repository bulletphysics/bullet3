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


namespace adl
{

int DeviceUtils::getNDevices( DeviceType type )
{
	switch( type )
	{
#if defined(ADL_ENABLE_CL)
	case TYPE_CL:
		return DeviceCL::getNDevices();
#endif
#if defined(ADL_ENABLE_DX11)
	case TYPE_DX11:
		return DeviceDX11::getNDevices();
#endif
	default:
		return 1;
	};
}

Device* DeviceUtils::allocate( DeviceType type, Config& cfg )
{
	Device* deviceData;
	switch( type )
	{
#if defined(ADL_ENABLE_CL)
	case TYPE_CL:
		deviceData = new DeviceCL();
		break;
#endif
#if defined(ADL_ENABLE_DX11)
	case TYPE_DX11:
		deviceData = new DeviceDX11();
		break;
#endif
	case TYPE_HOST:
		deviceData = new DeviceHost();
		break;
	default:
		ADLASSERT( 0 );
		break;
	};
	deviceData->initialize( cfg );
	return deviceData;
}

void DeviceUtils::deallocate( Device* deviceData )
{
	ADLASSERT( deviceData->getUsedMemory() == 0 );
	deviceData->release();
	delete deviceData;
}

void DeviceUtils::waitForCompletion( const Device* deviceData )
{
	deviceData->waitForCompletion();
}

#if defined(ADL_ENABLE_DX11)
	#if defined(ADL_ENABLE_CL)
	#define SELECT_DEVICEDATA( type, func ) \
		switch( type ) \
		{ \
		case TYPE_CL: ((DeviceCL*)m_device)->func; break; \
		case TYPE_DX11: ((DeviceDX11*)m_device)->func; break; \
		case TYPE_HOST: ((DeviceHost*)m_device)->func; break; \
		default: ADLASSERT(0); break; \
		}

	#define SELECT_DEVICEDATA1( deviceData, func ) \
		switch( deviceData->m_type ) \
		{ \
		case TYPE_CL: ((DeviceCL*)deviceData)->func; break; \
		case TYPE_DX11: ((DeviceDX11*)deviceData)->func; break; \
		case TYPE_HOST: ((DeviceHost*)deviceData)->func; break; \
		default: ADLASSERT(0); break; \
		}
	#else
	#define SELECT_DEVICEDATA( type, func ) \
		switch( type ) \
		{ \
		case TYPE_DX11: ((DeviceDX11*)m_device)->func; break; \
		case TYPE_HOST: ((DeviceHost*)m_device)->func; break; \
		default: ADLASSERT(0); break; \
		}

	#define SELECT_DEVICEDATA1( deviceData, func ) \
		switch( deviceData->m_type ) \
		{ \
		case TYPE_DX11: ((DeviceDX11*)deviceData)->func; break; \
		case TYPE_HOST: ((DeviceHost*)deviceData)->func; break; \
		default: ADLASSERT(0); break; \
		}
	#endif
#else
	#if defined(ADL_ENABLE_CL)
	#define SELECT_DEVICEDATA( type, func ) \
		switch( type ) \
		{ \
		case TYPE_CL: ((DeviceCL*)m_device)->func; break; \
		case TYPE_HOST: ((DeviceHost*)m_device)->func; break; \
		default: ADLASSERT(0); break; \
		}

	#define SELECT_DEVICEDATA1( deviceData, func ) \
		switch( deviceData->m_type ) \
		{ \
		case TYPE_CL: ((DeviceCL*)deviceData)->func; break; \
		case TYPE_HOST: ((DeviceHost*)deviceData)->func; break; \
		default: ADLASSERT(0); break; \
		}
	#else
	#define SELECT_DEVICEDATA( type, func ) \
		switch( type ) \
		{ \
		case TYPE_HOST: ((DeviceHost*)m_device)->func; break; \
		default: ADLASSERT(0); break; \
		}

	#define SELECT_DEVICEDATA1( deviceData, func ) \
		switch( deviceData->m_type ) \
		{ \
		case TYPE_HOST: ((DeviceHost*)deviceData)->func; break; \
		default: ADLASSERT(0); break; \
		}
	#endif
#endif

template<typename T>
Buffer<T>::Buffer()
{
	m_device = 0;
	m_size = 0;
	m_ptr = 0;

	m_uav = 0;
	m_srv = 0;

	m_allocated = false;
}

template<typename T>
Buffer<T>::Buffer(const Device* deviceData, int nElems, BufferType type )
{
	m_device = 0;
	allocate( deviceData, nElems, type );
}

template<typename T>
Buffer<T>::~Buffer()
{
	if( m_allocated )
	{
		if( m_device )
			SELECT_DEVICEDATA( m_device->m_type, deallocate( this ) );
	}

	m_device = 0;
	m_ptr = 0;
	m_size = 0;
}

template<typename T>
void Buffer<T>::setRawPtr( const Device* device, T* ptr, int size, BufferType type )
{
	ADLASSERT( m_device == 0 );
	ADLASSERT( type == BUFFER );	//	todo. implement
	ADLASSERT( device->m_type != TYPE_DX11 );	//	todo. implement set srv, uav

	m_device = device;
	m_ptr = ptr;
	m_size = size;
}

template<typename T>
void Buffer<T>::allocate(const Device* deviceData, int nElems, BufferType type )
{
	ADLASSERT( m_device == 0 );
	m_device = deviceData;
	m_size = 0;
	m_ptr = 0;

	m_uav = 0;
	m_srv = 0;

	SELECT_DEVICEDATA( m_device->m_type, allocate( this, nElems, type ) );
	m_allocated = true;
}

template<typename T>
void Buffer<T>::write(T* hostPtr, int nElems, int offsetNElems)
{
	ADLASSERT( nElems+offsetNElems <= m_size );
	SELECT_DEVICEDATA( m_device->m_type, copy(this, hostPtr, nElems, offsetNElems) );
}

template<typename T>
void Buffer<T>::read(T* hostPtr, int nElems, int offsetNElems) const
{
	SELECT_DEVICEDATA( m_device->m_type, copy(hostPtr,this, nElems, offsetNElems) );
}

template<typename T>
void Buffer<T>::write(Buffer<T>& src, int nElems)
{
	ADLASSERT( nElems <= m_size );
	SELECT_DEVICEDATA( m_device->m_type, copy(this, &src, nElems) );
}

template<typename T>
void Buffer<T>::read(Buffer<T>& dst, int nElems) const
{
	SELECT_DEVICEDATA( m_device->m_type, copy(&dst, this, nElems) );
}
/*
template<typename T>
Buffer<T>& Buffer<T>::operator = ( const Buffer<T>& buffer )
{
//	ADLASSERT( buffer.m_size <= m_size );

	SELECT_DEVICEDATA( m_device->m_type, copy(this, &buffer, min2( m_size, buffer.m_size) ) );

	return *this;
}
*/

template<DeviceType TYPE, bool COPY, typename T>
__inline
static
typename Buffer<T>* BufferUtils::map(const Device* device, const Buffer<T>* in, int copySize)
{
	Buffer<T>* native;
	ADLASSERT( device->m_type == TYPE );

	if( in->getType() == TYPE )
		native = (Buffer<T>*)in;
	else
	{
		ADLASSERT( copySize <= in->getSize() );
		copySize = (copySize==-1)? in->getSize() : copySize;

		native = new Buffer<T>( device, copySize );
		if( COPY )
		{
			if( in->getType() == TYPE_HOST )
				native->write( in->m_ptr, copySize );
			else if( native->getType() == TYPE_HOST )
			{
				in->read( native->m_ptr, copySize );
				DeviceUtils::waitForCompletion( in->m_device );
			}
			else
			{
				T* tmp = new T[copySize];
				in->read( tmp, copySize );
				DeviceUtils::waitForCompletion( in->m_device );
				native->write( tmp, copySize );
				DeviceUtils::waitForCompletion( native->m_device );
				delete [] tmp;
			}
		}
	}
	return native;
}

template<bool COPY, typename T>
__inline
static
void BufferUtils::unmap( Buffer<T>* native, const Buffer<T>* orig, int copySize )
{
	if( native != orig )
	{
		if( COPY ) 
		{
			copySize = (copySize==-1)? orig->getSize() : copySize;
			ADLASSERT( copySize <= orig->getSize() );
			if( orig->getType() == TYPE_HOST )
			{
				native->read( orig->m_ptr, copySize );
				DeviceUtils::waitForCompletion( native->m_device );
			}
			else if( native->getType() == TYPE_HOST )
			{
				Buffer<T>* dst = (Buffer<T>*)orig;
				dst->write( native->m_ptr, copySize );
				DeviceUtils::waitForCompletion( dst->m_device );
			}
			else
			{
				T* tmp = new T[copySize];
				native->read( tmp, copySize );
				DeviceUtils::waitForCompletion( native->m_device );
				Buffer<T>* dst = (Buffer<T>*)orig;
				dst->write( tmp, copySize );
				DeviceUtils::waitForCompletion( dst->m_device );
				delete [] tmp;
			}
		}
		delete native;
	}
}


template<typename T>
T& HostBuffer<T>::operator[](int idx)
{
	return m_ptr[idx];
}

template<typename T>
const T& HostBuffer<T>::operator[](int idx) const
{
	return m_ptr[idx];
}

template<typename T>
HostBuffer<T>& HostBuffer<T>::operator = ( const Buffer<T>& device )
{
	ADLASSERT( device.m_size <= m_size );

	SELECT_DEVICEDATA1( device.m_device, copy( m_ptr, &device, device.m_size ) );

	return *this;
}

#undef SELECT_DEVICEDATA

};
