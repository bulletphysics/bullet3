#ifndef TYPE_DEFINITIONS_H
#define TYPE_DEFINITIONS_H



///This file provides some platform/compiler checks for common definitions

#ifdef WIN32
		#if defined(__MINGW32__) || defined(__CYGWIN__) || (defined (_MSC_VER) && _MSC_VER < 1300)
		#else
		#endif //__MINGW32__

		typedef unsigned char     uint8_t;
		typedef unsigned long int uint64_t;
		typedef unsigned int      uint32_t;
		typedef unsigned short    uint16_t;

		#include <malloc.h>
		#define memalign(alignment, size) malloc(size);
			
#include <string.h> //memcpy

		#define USE_WIN32_THREADING 1

#else
#if defined	(__CELLOS_LV2__)
				///Playstation 3 Cell SDK
	#include <spu_printf.h>
#else
	//non-windows systems

	#define USE_PTHREADS 1

	
#endif	//__CELLOS_LV2__

	#include <stdint.h>
	#include <stdlib.h>
	#include <string.h> //for memcpy
	
#endif


#endif //TYPE_DEFINITIONS_H