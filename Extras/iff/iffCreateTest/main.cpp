/*
Test read and write IFF-85, Interchange Format File
Copyright (c) 2008 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
* IFF Support routines for writing IFF-85 files.          12/02/85
* (IFF is Interchange Format File.)
* By Jerry Morrison and Steve Shaw, Electronic Arts.
* This software is in the public domain.
* 
*/

#include <stdio.h>

#include "iff.h"

IFFP MyReadICat(GroupContext *parent) 
{
	printf("Found and skipped a CAT\n");
	return 0;
}

IFFP MySkipGroup( GroupContext * )
{
	printf("Found and skipped a LIST\n");
	return 0;
}

typedef UBYTE Masking;          /* Choice of masking technique.*/
typedef UBYTE Compression;      /* Choice of compression algorithm applied to

								/* A BitMapHeader is stored in a BMHD chunk. */
typedef struct {

	UWORD w, h;                 /* raster width & height in pixels */

	WORD  x, y;                 /* position for this image */
	UBYTE nPlanes;              /* # source bitplanes */
	Masking masking;            /* masking technique */
	Compression compression;    /* compression algoithm */
	UBYTE pad1;                 /* UNUSED.  For consistency, put 0 here.*/
	UWORD transparentColor;     /* transparent "color number" */
	UBYTE xAspect, yAspect;     /* aspect ratio, a rational number x/y */
	WORD  pageWidth, pageHeight;  /* source "page" size in pixels */
	
} BitMapHeader;


#define bufSz 512
BYTE bodyBuffer[bufSz];

static void btSwap(char* a, char* b)
{
	char tmp = *a;
	*a = *b;
	*b = tmp;
};

#define ID_ILBM MakeID('I','L','B','M')
#define ID_BMHD MakeID('B','M','H','D')
#define ID_CMAP MakeID('C','M','A','P')
#define ID_BODY MakeID('B','O','D','Y')

#define ID_DYNAWORLD MakeID('B','T','D','W')
#define ID_RIGIDBODY MakeID('B','T','R','B')
#define ID_SID MakeID('S','I','D',' ')
#define ID_MASS MakeID('M','A','S','S')
#define ID_SHAPE MakeID('S','H','A','P')


#define ID_COLOBJ MakeID('C','O','B','J')
#define ID_CUBE MakeID('C','U','B','E')
#define ID_DIMENSIONS MakeID('D','I','M','E')


IFFP MyProcessGroup(GroupContext *parent)
{
	/*compilerBug register*/ IFFP iffp;
	GroupContext rigidbodyContext;

	BitMapHeader bmHeader;
	bool foundBMHD = false;


	if (parent->subtype != ID_ILBM)
		return(IFF_OKAY); /* just continue scaning the file */

	iffp = OpenRGroup(parent, &rigidbodyContext);
	CheckIFFP();

	do {
		iffp = GetFChunkHdr(&rigidbodyContext);
		if (iffp == ID_BMHD) {
			printf("found ID_BMHD\n");
			foundBMHD = true;

			iffp = IFFReadBytes(&rigidbodyContext, (BYTE *)&bmHeader, (long)sizeof(BitMapHeader));
			//do endian swap
			bmHeader.w = endianSwap16(bmHeader.w);
			bmHeader.h = endianSwap16(bmHeader.h);
			bmHeader.pageWidth = endianSwap16(bmHeader.pageWidth);
			bmHeader.pageHeight = endianSwap16(bmHeader.pageHeight);
		}

		else if (iffp == ID_CMAP) {
			printf("found ID_CMAP\n");

			//      ilbmFrame.nColorRegs = maxColorReg;  /* we have room for this many */
			//       iffp = GetCMAP(
			//        &rigidbodyContext, (WORD *)&ilbmFrame.colorMap, &ilbmFrame.nColorRegs);
		}

		else if (iffp == ID_BODY) 
		{
			printf("found ID_BODY\n");
			if (!foundBMHD)
				return BAD_FORM;
			//     if (!ilbmFrame.foundBMHD)  return(BAD_FORM);   /* No BMHD chunk! */

			int moreBytes = ChunkMoreBytes(&rigidbodyContext);
			while (moreBytes>0)
			{
				int curRead = moreBytes > bufSz? bufSz : moreBytes;
				//read
				iffp = IFFReadBytes(&rigidbodyContext, bodyBuffer, curRead);
				moreBytes -= curRead;

			}
			printf("remaining=%d\n",moreBytes);
			if (iffp == IFF_OKAY) 
				iffp = IFF_DONE;      /* Eureka */



			//         nPlanes = MIN(ilbmFrame.bmHdr.nPlanes, EXDepth);
		}

		else if (iffp == END_MARK)
			iffp = BAD_FORM;

	} while (iffp >= IFF_OKAY);  /* loop if valid ID of ignored chunk or a
								 * subroutine returned IFF_OKAY (no errors).*/

	if (iffp != IFF_DONE)  return(iffp);

	/* If we get this far, there were no errors. */
	CloseRGroup(&rigidbodyContext);
	return(iffp);
}



#define CkErr(expression)  {if (ifferr == IFF_OKAY) ifferr = (expression);}


int main(int argc, char* argv[])
{
	FILE* file = 0;

	{
		//Create and write an IFF file from scratch
		file = fopen("test.iff","wb");

		GroupContext fileContext;
		GroupContext catContext;

		IFFP ifferr=0;

		ifferr = OpenWIFF(file, &fileContext, szNotYetKnown) ;
		//ifferr = StartWGroup(&fileContext, CAT, szNotYetKnown, ID_DYNAWORLD, &catContext);
		ifferr = StartWGroup(&fileContext, LIST, szNotYetKnown, ID_DYNAWORLD, &catContext);

		{
			GroupContext rigidbodyPropContext;
			ifferr = StartWGroup(&catContext, PROP, szNotYetKnown, ID_MASS, &rigidbodyPropContext);
			float mass = 0.1f;
			PutCk(&rigidbodyPropContext, ID_MASS, 4,(char*)&mass);
			ifferr = EndWGroup(&rigidbodyPropContext) ;

			for (int i=0;i<3;i++)
			{
				GroupContext rigidbodyContext;
				ifferr = StartWGroup(&catContext, FORM, szNotYetKnown, ID_RIGIDBODY, &rigidbodyContext);
				char sidbuffer[]="rb1";
				
				float dimensions[3] = {2,2,2};
				PutCk(&rigidbodyContext, ID_SID, 3,sidbuffer);
				{
					GroupContext shapeContext;
					ifferr = StartWGroup(&rigidbodyContext, FORM, szNotYetKnown, ID_SHAPE, &shapeContext);
					PutCk(&shapeContext, ID_CUBE, 4,(char*)&mass);
					PutCk(&shapeContext, ID_DIMENSIONS, sizeof(dimensions),(char*)&dimensions);
					ifferr = EndWGroup(&shapeContext) ;
				}
				ifferr = EndWGroup(&rigidbodyContext) ;
			}
			
		}
		ifferr = EndWGroup(&catContext) ;
		ifferr = CloseWGroup(&fileContext);

		fclose(file);
	}

	{
		//show a very simple way to skim through an ILBM or general IFF file
		//for more verbose feedback, use iffcheck.c
		IFFP result;
		//file = fopen("pe_3000_fall.iff","rb");
		file = fopen("test.iff","rb");

		ClientFrame clientFrame;

		clientFrame.getList = MySkipGroup;
		clientFrame.getProp = MySkipGroup;
		clientFrame.getForm = MyProcessGroup;
		clientFrame.getCat  = MyReadICat ;

		result = ReadIFF(file,&clientFrame);
		fclose(file);
	}


	return 0;
}
