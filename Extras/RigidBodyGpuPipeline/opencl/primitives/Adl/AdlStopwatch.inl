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

void Stopwatch::init( const Device* deviceData )
{
	ADLASSERT( m_impl == 0 );

	if( deviceData )
	{
		switch( deviceData->m_type )
		{
#if defined(ADL_ENABLE_CL)
		case TYPE_CL:
			m_impl = new StopwatchHost;//StopwatchCL
			break;
#endif
#if defined(ADL_ENABLE_DX11)
		case TYPE_DX11:
			m_impl = new StopwatchHost;//StopwatchDX11;
			break;
#endif
		case TYPE_HOST:
			m_impl = new StopwatchHost;
			break;
		default:
			ADLASSERT(0);
			break;
		};
	}
	else
	{
		m_impl = new StopwatchHost;
	}
	m_impl->init( deviceData );
}

Stopwatch::~Stopwatch()
{
	if( m_impl == 0 ) return;
	delete m_impl;
}

};