/*
bParse
Copyright (c) 2006-2009 Charlie C & Erwin Coumans  http://gamekit.googlecode.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "bBlenderFile.h"
#include "bMain.h"
#include "bDefines.h"
#include "bDNA.h"
#include <stdio.h>
#include <string.h>

// 32 && 64 bit versions
extern unsigned char DNAstr[];
extern int DNAlen;

extern unsigned char DNAstr64[];
extern int DNAlen64;


using namespace bParse;

bBlenderFile::bBlenderFile(const char* fileName)
:bFile(fileName, "BLENDER")
{
	mMain= new bMain(this, fileName, mVersion);
}



bBlenderFile::bBlenderFile(char *memoryBuffer, int len)
:bFile(memoryBuffer,len, "BLENDER"),
mMain(0)
{
	mMain= new bMain(this, "memoryBuf", mVersion);
}


bBlenderFile::~bBlenderFile()
{
	delete mMain;
}


bMain* bBlenderFile::getMain()
{
	return mMain;
}

// ----------------------------------------------------- //
void bBlenderFile::parseData()
{
	printf ("Building datablocks\n");
	printf ("Chunk size = %d\n",CHUNK_HEADER_LEN);
	printf ("File chunk size = %d\n", ChunkUtils::getOffset(mFlags));

	const bool swap = (mFlags&FD_ENDIAN_SWAP)!=0;
	


	char *dataPtr = mFileBuffer+mDataStart;

	bChunkInd dataChunk;
	dataChunk.code = 0;


	//dataPtr += ChunkUtils::getNextBlock(&dataChunk, dataPtr, mFlags);
	int seek = getNextBlock(&dataChunk, dataPtr, mFlags);
	//dataPtr += ChunkUtils::getOffset(mFlags);
	char *dataPtrHead = 0;

	while (dataChunk.code != DNA1)
	{
		



		// one behind
		if (dataChunk.code == SDNA) break;
		//if (dataChunk.code == DNA1) break;

		// same as (BHEAD+DATA dependency)
		dataPtrHead = dataPtr+ChunkUtils::getOffset(mFlags);
		char *id = readStruct(dataPtrHead, dataChunk);

		// lookup maps
		if (id)
		{
            m_chunkPtrPtrMap.insert(dataChunk.oldPtr, dataChunk);
			mLibPointers.insert(dataChunk.oldPtr, (bStructHandle*)id);

			m_chunks.push_back(dataChunk);
			// block it
			bListBasePtr *listID = mMain->getListBasePtr(dataChunk.code);
			if (listID)
				listID->push_back((bStructHandle*)id);
		}

		if (dataChunk.code == GLOB)
		{
			m_glob = (bStructHandle*) id;
		}

		// next please!
		dataPtr += seek;

		seek =  getNextBlock(&dataChunk, dataPtr, mFlags);
		if (seek < 0)
			break;
	}

}

void	bBlenderFile::addDataBlock(char* dataBlock)
{
	mMain->addDatablock(dataBlock);
}





// 32 && 64 bit versions
extern unsigned char DNAstr[];
extern int DNAlen;

//unsigned char DNAstr[]={0};
//int DNAlen=0;


extern unsigned char DNAstr64[];
extern int DNAlen64;


void	bBlenderFile::writeDNA(FILE* fp)
{

	bChunkInd dataChunk;
	dataChunk.code = DNA1;
	dataChunk.dna_nr = 0;
	dataChunk.nr = 1;
	
	if (VOID_IS_8)
	{
		dataChunk.len = DNAlen64;
		dataChunk.oldPtr = DNAstr64;
		fwrite(&dataChunk,sizeof(bChunkInd),1,fp);
		fwrite(DNAstr64, DNAlen64,1,fp);
	}
	else
	{
		dataChunk.len = DNAlen;
		dataChunk.oldPtr = DNAstr;
		fwrite(&dataChunk,sizeof(bChunkInd),1,fp);
		fwrite(DNAstr, DNAlen,1,fp);
	}
}

void	bBlenderFile::parse(bool verboseDumpAllTypes)
{
	if (VOID_IS_8)
	{
		parseInternal(verboseDumpAllTypes,(char*)DNAstr64,DNAlen64);
	}
	else
	{
		parseInternal(verboseDumpAllTypes,(char*)DNAstr,DNAlen);
	}
}

// experimental
int		bBlenderFile::write(const char* fileName, bool fixupPointers)
{
	FILE *fp = fopen(fileName, "wb");
	if (fp)
	{
		char header[SIZEOFBLENDERHEADER] ;
		memcpy(header, m_headerString, 7);
		int endian= 1;
		endian= ((char*)&endian)[0];

		if (endian)
		{
			header[7] = '_';
		} else
		{
			header[7] = '-';
		}
		if (VOID_IS_8)
		{
			header[8]='V';
		} else
		{
			header[8]='v';
		}

		header[9] = '2';
		header[10] = '4';
		header[11] = '9';
		
		fwrite(header,SIZEOFBLENDERHEADER,1,fp);

		writeChunks(fp, fixupPointers);

		writeDNA(fp);

		fclose(fp);
		
	} else
	{
		printf("Error: cannot open file %s for writing\n",fileName);
		return 0;
	}
	return 1;
}