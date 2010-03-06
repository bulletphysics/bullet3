/*
  FLUIDS v.1 - SPH Fluid Simulator for CPU and GPU
  Copyright (C) 2008. Rama Hoetzlein, http://www.rchoetzlein.com

  ZLib license
  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

#ifndef DEF_GEOM
	#define DEF_GEOM

	#include <vector>

	#define	HEAP_MAX			2147483640	// largest heap size (range of hpos)

	#define	ELEM_MAX			2147483640	// largest number of elements in a buffer (range of hval)
	//#define ELEM_MAX			32768		// largest number of elements in a buffer (range of hval)

	#define BUF_UNDEF			255

	#define FPOS				2			// free position offsets
	typedef unsigned char		uchar;
	typedef unsigned short		ushort;
	typedef signed int			hpos;		// pointers into heap
	typedef signed int			hval;		// values in heap	
	typedef hval				href;		// values are typically references 
	struct hList {
		ushort		cnt;
		ushort		max;
		hpos		pos;
	};
	
	class GeomAttr {
	public:
		GeomAttr()	{ name = ""; buf = 0; stride = 0; offset = 0; }
		std::string	name;
		ushort		buf;
		ushort		stride;
		ushort		offset;
	};

	class GeomBuf {
	public:
		GeomBuf()	{ dtype = 0; num = 0; max = 0; stride = 0; data = 0x0; }		
		uchar		dtype;
		hval		num;
		hval		max;
		long		size;
		ushort		stride;		
		char*		data;
	};

	class GeomX {
	public:
		GeomX ();
	
	//	virtual objType GetType ()			{ return 'geom'; }
	
		// Basic geometry setup	
		void FreeBuffers ();		
		void ClearAttributes ();
		void AddHeap ( int max );
		int CopyBuffer ( uchar bdest, uchar bsrc, GeomX& src );
		void CopyBuffers ( GeomX& src );
		void CopyAttributes ( GeomX& src );
		void CopyHeap ( GeomX& src );
		void ResetBuffer ( uchar b, int n );
		void ResetHeap ();
		int AddBuffer ( uchar typ, ushort stride, int max );
		int AddAttribute ( uchar b, std::string name, ushort stride );
		int AddAttribute ( uchar b, std::string name, ushort stride, bool bExtend );
		int GetAttribute ( std::string name );
		int GetAttrOffset ( std::string name );
		int NumElem ( uchar b )				{ if ( b==BUF_UNDEF) return 0; else return mBuf[b].num; }
		int MaxElem ( uchar b )				{ if ( b==BUF_UNDEF) return 0; else return mBuf[b].max; } 		
		int GetStride ( uchar b )			{ return mBuf[b].stride; }
		char* GetElem ( uchar b, int n )	{ return mBuf[b].data + n*mBuf[b].stride; }
		char* RandomElem ( uchar b, href& ndx );
		char* AddElem ( uchar b, href& pos );
		int AddElem ( uchar b, char* data );		
		bool DelElem ( uchar b, int n );
		char* GetStart ( uchar b )			{ return mBuf[b].data; }
		char* GetEnd ( uchar b )			{ return mBuf[b].data + mBuf[b].num*mBuf[b].stride; }
		GeomBuf* GetBuffer ( uchar b )		{ return &mBuf[b]; }
		GeomAttr* GetAttribute ( int n )		{ return &mAttribute[n]; }
		int GetNumBuf ()					{ return (int) mBuf.size(); }
		int GetNumAttr ()					{ return (int) mAttribute.size(); }
		hval* GetHeap ( hpos& num, hpos& max, hpos& free );

		int GetSize ();

		/*int AddRef ( int b, int n, int ref, ushort listpos );
		int AddRef ( int b, int n, int ref, bool bUseHeap, ushort listpos, ushort width );*/
		void ClearRefs ( hList& list );
		hval AddRef ( hval r, hList& list, hval delta  );
		hpos HeapAlloc ( ushort size, ushort& ret );
		hpos HeapExpand ( ushort size, ushort& ret  );
		void HeapAddFree ( hpos pos, int size );
		

	protected:
		std::vector< GeomBuf >		mBuf;	
		std::vector< GeomAttr >		mAttribute;

		hpos						mHeapNum;
		hpos						mHeapMax;
		hpos						mHeapFree;
		hval*						mHeap;
	};

#endif
