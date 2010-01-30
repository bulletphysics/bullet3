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
#include "bFile.h"
#include "bCommon.h"
#include "bChunk.h"
#include "bDNA.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#define SIZEOFBLENDERHEADER 12
#define MAX_ARRAY_LENGTH 512
using namespace bParse;


int numallocs = 0;

// ----------------------------------------------------- //
bFile::bFile(const char *filename, const char headerString[7])
	:	mOwnsBuffer(true),
		mFileBuffer(0),
		mFileLen(0),
		mVersion(0),
		mDataStart(0),
		mFileDNA(0),
		mMemoryDNA(0),
		mFlags(FD_INVALID)
{
	for (int i=0;i<7;i++)
	{
		m_headerString[i] = headerString[i];
	}

	FILE *fp = fopen(filename, "rb");
	if (fp)
	{
		fseek(fp, 0L, SEEK_END);
		mFileLen = ftell(fp);
		fseek(fp, 0L, SEEK_SET);

		mFileBuffer = (char*)malloc(mFileLen+1);
		fread(mFileBuffer, mFileLen, 1, fp);

		fclose(fp);

		//
		parseHeader();
		
	}
}

// ----------------------------------------------------- //
bFile::bFile( char *memoryBuffer, int len, const char headerString[7])
:	mOwnsBuffer(false),
	mFileBuffer(0),
		mFileLen(0),
		mVersion(0),
		mDataStart(0),
		mFileDNA(0),
		mMemoryDNA(0),
		mFlags(FD_INVALID)
{
	for (int i=0;i<7;i++)
	{
		m_headerString[i] = headerString[i];
	}
	mFileBuffer = memoryBuffer;
	mFileLen = len;
	
	parseHeader();
	
}


// ----------------------------------------------------- //
bFile::~bFile()
{
	if (mOwnsBuffer && mFileBuffer)
	{
		free(mFileBuffer);
		mFileBuffer = 0;
	}


	delete mMemoryDNA;
	delete mFileDNA;
}





// ----------------------------------------------------- //
void bFile::parseHeader()
{
	char *blenderBuf = mFileBuffer;
	char header[SIZEOFBLENDERHEADER+1] ;
	memcpy(header, blenderBuf, SIZEOFBLENDERHEADER);
	header[SIZEOFBLENDERHEADER]='\0';

	if (strncmp(header, m_headerString, 6)!=0)
	{
		memcpy(header, m_headerString, SIZEOFBLENDERHEADER);
		printf ("Invalid %s file...",header);
		return;
	}

	if (header[6] == 'd')
	{
		mFlags |= FD_DOUBLE_PRECISION;
	}

	char *ver = header+9;
	mVersion = atoi(ver);
	if (mVersion <= 241)
		printf ("Warning, %d not fully tested : <= 242\n", mVersion);


	int littleEndian= 1;
	littleEndian= ((char*)&littleEndian)[0];

	// swap ptr sizes...
	if (header[7]=='-')
	{
		mFlags |= FD_FILE_64;
		if (!VOID_IS_8)
			mFlags |= FD_BITS_VARIES;
	}
	else if (VOID_IS_8) mFlags |= FD_BITS_VARIES;

	// swap endian...
	if (header[8]=='V')
	{
		if (littleEndian ==1)
			mFlags |= FD_ENDIAN_SWAP;
	}
	else
		if (littleEndian==0)
			mFlags |= FD_ENDIAN_SWAP;


	printf ("%s\n",header);
	printf ("\nsizeof(void*) == %d\n",int(sizeof(void*)));
	const char* endStr = ((mFlags & FD_ENDIAN_SWAP)!=0) ? "yes" : "no";
	printf ("Swapping endian? %s\n",endStr);
	const char* bitStr = (mFlags &FD_FILE_64)!=0 ? "64 bit" : "32bit";
	printf ("File format is %s\n",bitStr);
	const char* varStr = (mFlags & FD_BITS_VARIES)!=0 ? "yes" : "no";
	printf ("Varing pointer sizes? %s\n",varStr);


	mFlags |= FD_OK;
}

// ----------------------------------------------------- //
bool bFile::ok()
{
	return (mFlags &FD_OK)!=0;
}

// ----------------------------------------------------- //
void bFile::parseInternal(bool verboseDumpAllTypes, char* memDna,int memDnaLength)
{
	if ( (mFlags &FD_OK) ==0)
		return;

	char *blenderData = mFileBuffer;
	int sdnaPos=0;

	char *tempBuffer = blenderData;
	for (int i=0; i<mFileLen; i++)
	{
		// looking for the data's starting position
		// and the start of SDNA decls

		if (!mDataStart && strncmp(tempBuffer, "REND", 4)==0)
			mDataStart = i;
		if (!sdnaPos && strncmp(tempBuffer, "SDNA", 4)==0)
			sdnaPos = i;

		if (mDataStart && sdnaPos) break;
		tempBuffer++;
	}

	mFileDNA = new bDNA();
	mFileDNA->init(blenderData+sdnaPos, mFileLen-sdnaPos, (mFlags & FD_ENDIAN_SWAP)!=0);

	if (verboseDumpAllTypes)
	{
		mFileDNA->dumpTypeDefinitions();
	}

	mMemoryDNA = new bDNA();
	int littleEndian= 1;
	littleEndian= ((char*)&littleEndian)[0];
			
	mMemoryDNA->init(memDna,memDnaLength,littleEndian==0);




	///@todo we need a better version check, add version/sub version info from FileGlobal into memory DNA/header files
	if (mMemoryDNA->getNumNames() != mFileDNA->getNumNames())
	{
		mFlags |= FD_VERSION_VARIES;
		printf ("Warning, file DNA is different than built in, performance is reduced. Best to re-export file with a matching version/platform");
	}

	// as long as it kept up to date it will be ok!!
	if (mMemoryDNA->lessThan(mFileDNA))
	{
		printf ("Warning, file DNA is newer than built in.");
	}

	mFileDNA->initCmpFlags(mMemoryDNA);
	
	parseData();
	
	resolvePointers(verboseDumpAllTypes);//verboseDumpAllBlocks);

	updateOldPointers();

	printf("numAllocs = %d\n",numallocs);
}



// ----------------------------------------------------- //
void bFile::swap(char *head, bChunkInd& dataChunk)
{
	char *data = head;
	short *strc = mFileDNA->getStruct(dataChunk.dna_nr);
	int len = mFileDNA->getLength(strc[0]);

	for (int i=0; i<dataChunk.nr; i++)
	{
		swapStruct(dataChunk.dna_nr, data);
		data+=len;
	}
}



// ----------------------------------------------------- //
char* bFile::readStruct(char *head, bChunkInd&  dataChunk)
{
	if (mFlags & FD_ENDIAN_SWAP)
		swap(head, dataChunk);

	

	if (!mFileDNA->flagEqual(dataChunk.dna_nr))
	{
		// Ouch! need to rebuild the struct
		short *oldStruct,*curStruct;
		char *oldType, *newType;
		int oldLen, curLen, reverseOld;


		oldStruct = mFileDNA->getStruct(dataChunk.dna_nr);
		oldType = mFileDNA->getType(oldStruct[0]);
		oldLen = mFileDNA->getLength(oldStruct[0]);

		///don't try to convert Link block data, just memcpy it. Other data can be converted.
		if (strcmp("Link",oldType)!=0)
		{
			reverseOld = mMemoryDNA->getReverseType(oldType);

			if ((reverseOld!=-1))
			{
				// make sure it's here
				//assert(reverseOld!= -1 && "getReverseType() returned -1, struct required!");

				//
				curStruct = mMemoryDNA->getStruct(reverseOld);
				newType = mMemoryDNA->getType(curStruct[0]);
				curLen = mMemoryDNA->getLength(curStruct[0]);



				// make sure it's the same
				assert((strcmp(oldType, newType)==0) && "internal error, struct mismatch!");


				numallocs++;
				// numBlocks * length
    			char *dataAlloc = new char[(dataChunk.nr*curLen)+1];
				memset(dataAlloc, 0, (dataChunk.nr*curLen)+1);

				// track allocated
				addDataBlock(dataAlloc);

				char *cur = dataAlloc;
				char *old = head;
				for (int block=0; block<dataChunk.nr; block++)
				{
					bool fixupPointers = true;
					parseStruct(cur, old, dataChunk.dna_nr, reverseOld, fixupPointers);
					mLibPointers.insert(old,(bStructHandle*)cur);

					cur += curLen;
					old += oldLen;
				}
				return dataAlloc;
			}
		} else
		{
			//printf("Link found\n");
		}
	} else
	{
//#define DEBUG_EQUAL_STRUCTS
#ifdef DEBUG_EQUAL_STRUCTS
		short *oldStruct;
		char *oldType;
		oldStruct = mFileDNA->getStruct(dataChunk.dna_nr);
		oldType = mFileDNA->getType(oldStruct[0]);
		printf("%s equal structure, just memcpy\n",oldType);
#endif //
	}


	char *dataAlloc = new char[(dataChunk.len)+1];
	memset(dataAlloc, 0, dataChunk.len+1);


	// track allocated
	addDataBlock(dataAlloc);

	memcpy(dataAlloc, head, dataChunk.len);
	return dataAlloc;

}


// ----------------------------------------------------- //
void bFile::parseStruct(char *strcPtr, char *dtPtr, int old_dna, int new_dna, bool fixupPointers)
{
	if (old_dna == -1) return;
	if (new_dna == -1) return;

	//disable this, because we need to fixup pointers/ListBase
	if (0)//mFileDNA->flagEqual(old_dna))
	{
		short *strc = mFileDNA->getStruct(old_dna);
		int len = mFileDNA->getLength(strc[0]);

		memcpy(strcPtr, dtPtr, len);
		return;
	}

	// Ok, now build the struct
	char *memType, *memName, *cpc, *cpo;
	short *fileStruct, *filePtrOld, *memoryStruct, *firstStruct;
	int elementLength, size, revType, old_nr, new_nr, fpLen;
	short firstStructType;


	// File to memory lookup
	memoryStruct = mMemoryDNA->getStruct(new_dna);
	fileStruct = mFileDNA->getStruct(old_dna);
	firstStruct = fileStruct;


	filePtrOld = fileStruct;
	firstStructType = mMemoryDNA->getStruct(0)[0];

	// Get number of elements
	elementLength = memoryStruct[1];
	memoryStruct+=2;

	cpc = strcPtr; cpo = 0;
	for (int ele=0; ele<elementLength; ele++, memoryStruct+=2)
	{
		memType = mMemoryDNA->getType(memoryStruct[0]);
		memName = mMemoryDNA->getName(memoryStruct[1]);


		size = mMemoryDNA->getElementSize(memoryStruct[0], memoryStruct[1]);
		revType = mMemoryDNA->getReverseType(memoryStruct[0]);

		if (revType != -1 && memoryStruct[0]>=firstStructType && memName[0] != '*')
		{
			cpo = getFileElement(firstStruct, memName, memType, dtPtr, &filePtrOld);
			if (cpo)
			{
				int arrayLen = mFileDNA->getArraySizeNew(filePtrOld[1]);
				old_nr = mFileDNA->getReverseType(memType);
				new_nr = revType;
				fpLen = mFileDNA->getElementSize(filePtrOld[0], filePtrOld[1]);
				if (arrayLen==1)
				{
					parseStruct(cpc, cpo, old_nr, new_nr,fixupPointers);
				} else
				{
					char* tmpCpc = cpc;
					char* tmpCpo = cpo;

					for (int i=0;i<arrayLen;i++)
					{
						parseStruct(tmpCpc, tmpCpo, old_nr, new_nr,fixupPointers);
						tmpCpc += size/arrayLen;
						tmpCpo += fpLen/arrayLen;
					}
				}
				cpc+=size;
				cpo+=fpLen;
			}
			else
				cpc+=size;
		}
		else
		{
			getMatchingFileDNA(fileStruct, memName, memType, cpc, dtPtr,fixupPointers);
			cpc+=size;
		}

	}
}


// ----------------------------------------------------- //
static void getElement(int arrayLen, const char *cur, const char *old, char *oldPtr, char *curData)
{
#define getEle(value, current, type, cast, size, ptr)\
	if (strcmp(current, type)==0)\
	{\
		value = (*(cast*)ptr);\
		ptr += size;\
	}

#define setEle(value, current, type, cast, size, ptr)\
	if (strcmp(current, type)==0)\
	{\
		(*(cast*)ptr) = (cast)value;\
		ptr += size;\
	}
	double value = 0.0;

	for (int i=0; i<arrayLen; i++)
	{
		getEle(value, old, "char",   char,   sizeof(char),   oldPtr);
		setEle(value, cur, "char",   char,   sizeof(char),   curData);
		getEle(value, old, "short",  short,  sizeof(short),  oldPtr);
		setEle(value, cur, "short",  short,  sizeof(short),  curData);
		getEle(value, old, "ushort",  unsigned short,  sizeof(unsigned short),  oldPtr);
		setEle(value, cur, "ushort",  unsigned short,  sizeof(unsigned short),  curData);
		getEle(value, old, "int",    int,    sizeof(int),    oldPtr);
		setEle(value, cur, "int",    int,    sizeof(int),    curData);
		getEle(value, old, "long",   int,    sizeof(int),    oldPtr);
		setEle(value, cur, "long",   int,    sizeof(int),    curData);
		getEle(value, old, "float",  float,  sizeof(float),  oldPtr);
		setEle(value, cur, "float",  float,  sizeof(float),  curData);
		getEle(value, old, "double", double, sizeof(double), oldPtr);
		setEle(value, cur, "double", double, sizeof(double), curData);
	}
}


// ----------------------------------------------------- //
void bFile::swapData(char *data, short type, int arraySize)
{
	if (mFlags &FD_ENDIAN_SWAP)
	{
		if (type == 2 || type == 3)
		{
			short *sp = (short*)data;
			for (int i=0; i<arraySize; i++)
			{
				sp[0] = ChunkUtils::swapShort(sp[0]);
				sp++;
			}
		}
		if (type>3 && type <8)
		{
			char c;
			char *cp = data;
			for (int i=0; i<arraySize; i++)
			{
				c = cp[0];
				cp[0] = cp[3];
				cp[3] = c;
				c = cp[1];
				cp[1] = cp[2];
				cp[2] = c;
				cp+=4;
			}
		}
	}
}


void bFile::swapPtr(char *dst, char *src)
{
	int ptrFile = mFileDNA->getPointerSize();
	int ptrMem = mMemoryDNA->getPointerSize();

	if (!src && !dst)
		return;

	if (ptrFile == ptrMem)
		memcpy(dst, src, ptrMem);
	else if (ptrMem==4 && ptrFile==8)
	{
		long64 longValue = *((long64*)src);
		*((int*)dst) = (int)(longValue>>3);
	}
	else if (ptrMem==8 && ptrFile==4)
		*((long64*)dst)= *((int*)src);
	else
	{
		printf ("%d %d\n", ptrFile,ptrMem);
		assert(0 && "Invalid pointer len");
	}
}

// ----------------------------------------------------- //
void bFile::getMatchingFileDNA(short* dna_addr, const char* lookupName,  const char* lookupType, char *strcData, char *data, bool fixupPointers)
{
	// find the matching memory dna data
	// to the file being loaded. Fill the
	// memory with the file data...

	int len = dna_addr[1];
	dna_addr+=2;

	for (int i=0; i<len; i++, dna_addr+=2)
	{
		const char* type = mFileDNA->getType(dna_addr[0]);
		const char* name = mFileDNA->getName(dna_addr[1]);

		int eleLen = mFileDNA->getElementSize(dna_addr[0], dna_addr[1]);
		

		if (strcmp(lookupName, name)==0)
		{
			//int arrayLenold = mFileDNA->getArraySize((char*)name.c_str());
			int arrayLen = mFileDNA->getArraySizeNew(dna_addr[1]);
			//assert(arrayLenold == arrayLen);
			
			if (name[0] == '*')
			{
				// cast pointers
				//int ptrFile = mFileDNA->getPointerSize();
				//int ptrMem = mMemoryDNA->getPointerSize();

				swapPtr(strcData, data);

				if (fixupPointers)
				{
					if (arrayLen > 1)
					{
						void **sarray = (void**)strcData;
						void **darray = (void**)data;

						for (int a=0; a<arrayLen; a++)
						{
							swapPtr((char*)&sarray[a], (char*)&darray[a]);
							m_pointerFixupArray.push_back((char*)&sarray[a]);
						}
					}
					else
					{
						if (name[1] == '*')
							m_pointerPtrFixupArray.push_back(strcData);
						else
							m_pointerFixupArray.push_back(strcData);
					}
				}
				else
				{
//					printf("skipped %s %s : %x\n",type.c_str(),name.c_str(),strcData);
				}

			}

			else if (strcmp(type, lookupType)==0)
				memcpy(strcData, data, eleLen);
			else
				getElement(arrayLen, lookupType, type, data, strcData);

			// --
			return;
		}
		data+=eleLen;
	}
}


// ----------------------------------------------------- //
char* bFile::getFileElement(short *firstStruct, char *lookupName, char *lookupType, char *data, short **foundPos)
{
	short *old = firstStruct;//mFileDNA->getStruct(old_nr);
	int elementLength = old[1];
	old+=2;

	for (int i=0; i<elementLength; i++, old+=2)
	{
		char* type = mFileDNA->getType(old[0]);
		char* name = mFileDNA->getName(old[1]);
		int len = mFileDNA->getElementSize(old[0], old[1]);

		if (strcmp(lookupName, name)==0)
		{
			if (strcmp(type, lookupType)==0)
			{
				if (foundPos)
					*foundPos = old;
				return data;
			}
			return 0;
		}
		data+=len;
	}
	return 0;
}


// ----------------------------------------------------- //
void bFile::swapStruct(int dna_nr, char *data)
{
	if (dna_nr == -1) return;

	short *strc = mFileDNA->getStruct(dna_nr);
	//short *firstStrc = strc;

	int elementLen= strc[1];
	strc+=2;

	short first = mFileDNA->getStruct(0)[0];

	char *buf = data;
	for (int i=0; i<elementLen; i++, strc+=2)
	{
		char *type = mFileDNA->getType(strc[0]);
		char *name = mFileDNA->getName(strc[1]);

		int size = mFileDNA->getElementSize(strc[0], strc[1]);
		if (strc[0] >= first && name[0]!='*')
		{
			int old_nr = mFileDNA->getReverseType(type);
			int arrayLen = mFileDNA->getArraySizeNew(strc[1]);
			if (arrayLen==1)
			{
				swapStruct(old_nr,buf);
			} else
			{
				char* tmpBuf = buf;
				for (int i=0;i<arrayLen;i++)
				{
					swapStruct(old_nr,tmpBuf);
					tmpBuf+=size/arrayLen;
				}
			}
		}
		else
		{
			//int arrayLenOld = mFileDNA->getArraySize(name);
			int arrayLen = mFileDNA->getArraySizeNew(strc[1]);
			//assert(arrayLenOld == arrayLen);
			swapData(buf, strc[0], arrayLen);
		}
		buf+=size;
	}
}



void bFile::resolvePointersMismatch()
{
//	printf("resolvePointersStructMismatch\n");

	int i;

	for (i=0;i<	m_pointerFixupArray.size();i++)
	{
		char* cur = m_pointerFixupArray.at(i);
		void** ptrptr = (void**) cur;
		void* ptr = *ptrptr;
		ptr = findLibPointer(ptr);
		if (ptr)
		{
			//printf("Fixup pointer!\n");
			*(ptrptr) = ptr;
		} else
		{
//			printf("pointer not found: %x\n",cur);
		}
	}

	for (i=0;i<	m_pointerPtrFixupArray.size();i++)
	{
		char* cur= m_pointerPtrFixupArray.at(i);
		void** ptrptr = (void**)cur;
		void *ptr = findLibPointer(*ptrptr);
		if (ptr)
		{
			(*ptrptr) = ptr;

			void **array= (void**)(*(ptrptr));

			int n=0, n2=0;
			int swapoffs = mFileDNA->getPointerSize() > mMemoryDNA->getPointerSize() ? 2 : 1;
			void *np = array[n];
			while(np)
			{
				if (mFlags & FD_BITS_VARIES)
					swapPtr((char*)&array[n], (char*)&array[n2]);

				np = findLibPointer(array[n]);
				if (np)
					array[n] = np;
				++n;
				n2 += swapoffs;
			}
		}
	}
}


///this loop only works fine if the Blender DNA structure of the file matches the headerfiles
void bFile::resolvePointersChunk(const bChunkInd& dataChunk, bool verboseDumpAllBlocks)
{
	bParse::bDNA* fileDna = mFileDNA ? mFileDNA : mMemoryDNA;

	short int* oldStruct = fileDna->getStruct(dataChunk.dna_nr);
	short oldLen = fileDna->getLength(oldStruct[0]);
	//char* structType = fileDna->getType(oldStruct[0]);

	char* cur	= (char*)findLibPointer(dataChunk.oldPtr);
	for (int block=0; block<dataChunk.nr; block++)
	{
		resolvePointersStructRecursive(cur,dataChunk.dna_nr, verboseDumpAllBlocks,1);
		cur += oldLen;
	}
}


void bFile::resolvePointersStructRecursive(char *strcPtr, int dna_nr, bool verboseDumpAllBlocks,int recursion)
{
	
	bParse::bDNA* fileDna = mFileDNA ? mFileDNA : mMemoryDNA;

	char* memType;
	char* memName;
	short	firstStructType = fileDna->getStruct(0)[0];


	char* elemPtr= strcPtr;

	short int* oldStruct = fileDna->getStruct(dna_nr);
	
	int elementLength = oldStruct[1];
	oldStruct+=2;


	for (int ele=0; ele<elementLength; ele++, oldStruct+=2)
	{

		memType = fileDna->getType(oldStruct[0]);
		memName = fileDna->getName(oldStruct[1]);
		
		

		int arrayLen = fileDna->getArraySizeNew(oldStruct[1]);
		if (memName[0] == '*')
		{
			if (arrayLen > 1)
			{
				void **array= (void**)elemPtr;
				for (int a=0; a<arrayLen; a++)
					array[a] = findLibPointer(array[a]);
			}
			else
			{
				void** ptrptr = (void**) elemPtr;
				void* ptr = *ptrptr;
				ptr = findLibPointer(ptr);
				if (ptr)
				{
	//				printf("Fixup pointer at 0x%x from 0x%x to 0x%x!\n",ptrptr,*ptrptr,ptr);
					*(ptrptr) = ptr;
					if (memName[1] == '*' && ptrptr && *ptrptr)
					{
						// This	will only work if the given	**array	is continuous
						void **array= (void**)*(ptrptr);
						void *np= array[0];
						int	n=0;
						while (np)
						{
							np= findLibPointer(array[n]);
							if (np) array[n]= np;
							n++;
						}
					}
				} else
				{
	//				printf("Cannot fixup pointer at 0x%x from 0x%x to 0x%x!\n",ptrptr,*ptrptr,ptr);
				}
			}
		} else
		{
			int revType = fileDna->getReverseType(oldStruct[0]);
			if (oldStruct[0]>=firstStructType) //revType != -1 && 
			{
				if (verboseDumpAllBlocks)
				{
					for (int i=0;i<recursion;i++)
					{
						printf("  ");
					}
					printf("<%s type=\"%s\">\n",memName,memType);
				}
				resolvePointersStructRecursive(elemPtr,revType, verboseDumpAllBlocks,recursion+1);
				if (verboseDumpAllBlocks)
				{
					for (int i=0;i<recursion;i++)
					{
						printf("  ");
					}
					printf("</%s>\n",memName);
				}
			} else
			{
				//export a simple type
				if (verboseDumpAllBlocks)
				{

					if (arrayLen>MAX_ARRAY_LENGTH)
					{
						printf("too long\n");
					} else
					{
						//printf("%s %s\n",memType,memName);

						bool isIntegerType = (strcmp(memType,"char")==0) || (strcmp(memType,"int")==0) || (strcmp(memType,"short")==0);

						if (isIntegerType)
						{
							const char* newtype="int";
							int dbarray[MAX_ARRAY_LENGTH];
							int* dbPtr = 0;
							char* tmp = elemPtr;
							dbPtr = &dbarray[0];
							if (dbPtr)
							{
								int i;
								getElement(arrayLen, newtype,memType, tmp, (char*)dbPtr);
								for (i=0;i<recursion;i++)
									printf("  ");
								if (arrayLen==1)
									printf("<%s type=\"%s\">",memName,memType);
								else
									printf("<%s type=\"%s\" count=%d>",memName,memType,arrayLen);
								for (i=0;i<arrayLen;i++)
									printf(" %d ",dbPtr[i]);
								printf("</%s>\n",memName);
							}
						} else
						{
							const char* newtype="double";
							double dbarray[MAX_ARRAY_LENGTH];
					 		double* dbPtr = 0;
							char* tmp = elemPtr;
							dbPtr = &dbarray[0];
							if (dbPtr)
							{
								int i;
								getElement(arrayLen, newtype,memType, tmp, (char*)dbPtr);
								for (i=0;i<recursion;i++)
									printf("  ");
								if (arrayLen==1)
									printf("<%s type=\"%s\">",memName,memType);
								else
									printf("<%s type=\"%s\" count=%d>",memName,memType,arrayLen);
								for (i=0;i<arrayLen;i++)
									printf(" %f ",dbPtr[i]);
								printf("</%s>\n",memName);
							}
						}
					}
					
				}
			}
		}

		int size = fileDna->getElementSize(oldStruct[0], oldStruct[1]);
		elemPtr+=size;
		
	}
}


///Resolve pointers replaces the original pointers in structures, and linked lists by the new in-memory structures
void bFile::resolvePointers(bool verboseDumpAllBlocks)
{
	bParse::bDNA* fileDna = mFileDNA ? mFileDNA : mMemoryDNA;

	printf("resolvePointers start\n");
	//char *dataPtr = mFileBuffer+mDataStart;

	if (1) //mFlags & (FD_BITS_VARIES | FD_VERSION_VARIES))
	{
		resolvePointersMismatch();	
	}
	
	{
		for (int i=0;i<m_chunks.size();i++)
		{
			const bChunkInd& dataChunk = m_chunks.at(i);

			if (!mFileDNA || fileDna->flagEqual(dataChunk.dna_nr))
			{
				//dataChunk.len
				short int* oldStruct = fileDna->getStruct(dataChunk.dna_nr);
				char* oldType = fileDna->getType(oldStruct[0]);
				
				if (verboseDumpAllBlocks)
					printf("<%s>\n",oldType);
				
				resolvePointersChunk(dataChunk, verboseDumpAllBlocks);

				if (verboseDumpAllBlocks)
					printf("</%s>\n",oldType);
			} else
			{
				//printf("skipping mStruct\n");
			}
		}
	}
	
	printf("resolvePointers end\n");
}


// ----------------------------------------------------- //
void* bFile::findLibPointer(void *ptr)
{

	bStructHandle** ptrptr = getLibPointers().find(ptr);
	if (ptrptr)
		return *ptrptr;
	return 0;
}


void	bFile::updateOldPointers()
{
	int i;

	for (i=0;i<m_chunks.size();i++)
	{
		bChunkInd& dataChunk = m_chunks[i];
		dataChunk.oldPtr = findLibPointer(dataChunk.oldPtr);
	}
}
void	bFile::dumpChunks(bParse::bDNA* dna)
{
	int i;

	for (i=0;i<m_chunks.size();i++)
	{
		bChunkInd& dataChunk = m_chunks[i];
		char* codeptr = (char*)&dataChunk.code;
		char codestr[5] = {codeptr[0],codeptr[1],codeptr[2],codeptr[3],0};
		
		short* newStruct = dna->getStruct(dataChunk.dna_nr);
		char* typeName = dna->getType(newStruct[0]);
		printf("%3d: %s  ",i,typeName);

		printf("code=%s  ",codestr);
		
		printf("ptr=%p  ",dataChunk.oldPtr);
		printf("len=%d  ",dataChunk.len);
		printf("nr=%d  ",dataChunk.nr);
		if (dataChunk.nr!=1)
		{
			printf("not 1\n");
		}
		printf("\n");

		
		

	}

#if 0
	IDFinderData ifd;
	ifd.success = 0;
	ifd.IDname = NULL;
	ifd.just_print_it = 1;
	for (i=0; i<bf->m_blocks.size(); ++i) 
	{
		BlendBlock* bb = bf->m_blocks[i];
		printf("tag='%s'\tptr=%p\ttype=%s\t[%4d]",		bb->tag, bb,bf->types[bb->type_index].name,bb->m_array_entries_.size());
		block_ID_finder(bb, bf, &ifd);
		printf("\n");
	}
#endif

}


void	bFile::writeChunks(FILE* fp, bool fixupPointers)
{
	bParse::bDNA* fileDna = mFileDNA ? mFileDNA : mMemoryDNA;

	for (int i=0;i<m_chunks.size();i++)
	{
		bChunkInd& dataChunk = m_chunks.at(i);
		
		// Ouch! need to rebuild the struct
		short *oldStruct,*curStruct;
		char *oldType, *newType;
		int oldLen, curLen, reverseOld;

		oldStruct = fileDna->getStruct(dataChunk.dna_nr);
		oldType = fileDna->getType(oldStruct[0]);
		oldLen = fileDna->getLength(oldStruct[0]);
		///don't try to convert Link block data, just memcpy it. Other data can be converted.
		reverseOld = mMemoryDNA->getReverseType(oldType);
		

		if ((reverseOld!=-1))
		{
			// make sure it's here
			//assert(reverseOld!= -1 && "getReverseType() returned -1, struct required!");
			//
			curStruct = mMemoryDNA->getStruct(reverseOld);
			newType = mMemoryDNA->getType(curStruct[0]);
			// make sure it's the same
			assert((strcmp(oldType, newType)==0) && "internal error, struct mismatch!");

			
			curLen = mMemoryDNA->getLength(curStruct[0]);
			dataChunk.dna_nr = reverseOld;
			if (strcmp("Link",oldType)!=0)
			{
				dataChunk.len = curLen * dataChunk.nr;
			} else
			{
//				printf("keep length of link = %d\n",dataChunk.len);
			}
		
			//write the structure header
			fwrite(&dataChunk,sizeof(bChunkInd),1,fp);
			


			short int* curStruct1 = mMemoryDNA->getStruct(dataChunk.dna_nr);
			assert(curStruct1 == curStruct);

			char* cur	= fixupPointers  ?  (char*)findLibPointer(dataChunk.oldPtr) : (char*)dataChunk.oldPtr;

			//write the actual contents of the structure(s)
			fwrite(cur,dataChunk.len,1,fp);
		} else
		{
			printf("serious error, struct mismatch: don't write\n");
		}
	}
	
}


//eof

