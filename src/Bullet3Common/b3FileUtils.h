#ifndef B3_FILE_UTILS_H
#define B3_FILE_UTILS_H

#include <stdio.h>
#include "b3Scalar.h"
#include <stddef.h>//ptrdiff_h

struct b3FileUtils
{
	b3FileUtils()
	{
	}
	virtual ~b3FileUtils()
	{
	}

	bool findFile(const char* orgFileName, char* relativeFileName, int maxRelativeFileNameMaxLen)
	{
		
			const char* prefix[]={"","./","./data/","../data/","../../data/","../../../data/","../../../../data/"};
			int numPrefixes = sizeof(prefix)/sizeof(const char*);
	
			FILE* f=0;
			bool fileFound = false;
			int result = 0;

			for (int i=0;!f && i<numPrefixes;i++)
			{
#ifdef _WIN32
				sprintf_s(relativeFileName,maxRelativeFileNameMaxLen,"%s%s",prefix[i],orgFileName);
#else
				sprintf(relativeFileName,"%s%s",prefix[i],orgFileName);
#endif
				f = fopen(relativeFileName,"rb");
				if (f)
				{
					fileFound = true;
					break;
				}
			}
			if (f)
			{
				fclose(f);
			}
	
		return fileFound;
	}

	static const char* strip2(const char* name, const char* pattern)
	{
		size_t const patlen = strlen(pattern);
		size_t patcnt = 0;
		const char * oriptr;
		const char * patloc;
		// find how many times the pattern occurs in the original string
		for (oriptr = name; patloc = strstr(oriptr, pattern); oriptr = patloc + patlen)
		{
			patcnt++;
		}
		return oriptr;
	}


	void extractPath(const char* fileName, char* path, int maxPathLength)
	{
		const char* stripped = strip2(fileName, "/");
		stripped = strip2(stripped, "\\");

		ptrdiff_t len = stripped-fileName;
		b3Assert((len+1)<maxPathLength);

		if (len && ((len+1)<maxPathLength))
		{

			for (int i=0;i<len;i++)
			{
				path[i] = fileName[i];
			}
			path[len]=0;
		} else
		{
#ifdef _WIN32
		sprintf_s(path, maxPathLength,"");
#else
			sprintf(path, "");
#endif
		}
	}

	/*static const char* strip2(const char* name, const char* pattern)
	{
		size_t const patlen = strlen(pattern);
		size_t patcnt = 0;
		const char * oriptr;
		const char * patloc;
		// find how many times the pattern occurs in the original string
		for (oriptr = name; patloc = strstr(oriptr, pattern); oriptr = patloc + patlen)
		{
			patcnt++;
		}
		return oriptr;
	}
	*/

};
#endif //B3_FILE_UTILS_H