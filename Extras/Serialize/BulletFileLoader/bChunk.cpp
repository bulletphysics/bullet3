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
#include <memory.h>
#include "bChunk.h"
#include "bDefines.h"
#include "bFile.h"

using namespace bParse;


// ----------------------------------------------------- //
short ChunkUtils::swapShort(short sht)
{
	SWITCH_SHORT(sht);
	return sht;
}

// ----------------------------------------------------- //
int ChunkUtils::swapInt(int inte)
{
	SWITCH_INT(inte);
	return inte;
}

// ----------------------------------------------------- //
long64 ChunkUtils::swapLong64(long64 lng)
{
	SWITCH_LONGINT(lng);
	return lng;
}

// ----------------------------------------------------- //
int ChunkUtils::getOffset(int flags)
{
	// if the file is saved in a
	// different format, get the
	// file's chunk size
	int res = CHUNK_HEADER_LEN;

	if (VOID_IS_8)
	{
		if (flags &FD_BITS_VARIES)
			res = sizeof(bChunkPtr4);
	}
	else
	{
		if (flags &FD_BITS_VARIES)
			res = sizeof(bChunkPtr8);
	}
	return res;
}


// ----------------------------------------------------- //
int ChunkUtils::getNextBlock(bChunkInd *dataChunk,  const char *dataPtr, const int flags)
{
	bool swap = false;
	bool varies = false;

	if (flags &FD_ENDIAN_SWAP) swap = true;
	if (flags &FD_BITS_VARIES) varies = true;

	if (VOID_IS_8)
	{
		if (varies)
		{
			bChunkPtr4 head;
			memcpy(&head, dataPtr, sizeof(bChunkPtr4));


			bChunkPtr8 chunk;

			chunk.code		= head.code;
			chunk.len		= head.len;
			chunk.old		= head.old;
			chunk.dna_nr	= head.dna_nr;
			chunk.nr		= head.nr;

			if (swap)
			{
				if ((chunk.code & 0xFFFF)==0)
					chunk.code >>=16;

				SWITCH_INT(chunk.len);
				SWITCH_INT(chunk.dna_nr);
				SWITCH_INT(chunk.nr);
			}


			memcpy(dataChunk, &chunk, sizeof(bChunkInd));
		}
		else
		{
			bChunkPtr8 c;
			memcpy(&c, dataPtr, sizeof(bChunkPtr8));

			if (swap)
			{
				if ((c.code & 0xFFFF)==0)
					c.code >>=16;

				SWITCH_INT(c.len);
				SWITCH_INT(c.dna_nr);
				SWITCH_INT(c.nr);
			}

			memcpy(dataChunk, &c, sizeof(bChunkInd));
		}
	}
	else
	{
		if (varies)
		{
			bChunkPtr8 head;
			memcpy(&head, dataPtr, sizeof(bChunkPtr8));


			bChunkPtr4 chunk;
			chunk.code = head.code;
			chunk.len = head.len;

			long64 oldPtr =0;

			memcpy(&oldPtr, &head.old, 8);
			chunk.old = (int)(oldPtr >> 3);

			chunk.dna_nr = head.dna_nr;
			chunk.nr = head.nr;

			if (swap)
			{
				if ((chunk.code & 0xFFFF)==0)
					chunk.code >>=16;

				SWITCH_INT(chunk.len);
				SWITCH_INT(chunk.dna_nr);
				SWITCH_INT(chunk.nr);
			}

			memcpy(dataChunk, &chunk, sizeof(bChunkInd));
		}
		else
		{
			bChunkPtr4 c;
			memcpy(&c, dataPtr, sizeof(bChunkPtr4));

			if (swap)
			{
				if ((c.code & 0xFFFF)==0)
					c.code >>=16;

				SWITCH_INT(c.len);
				SWITCH_INT(c.dna_nr);
				SWITCH_INT(c.nr);
			}
			memcpy(dataChunk, &c, sizeof(bChunkInd));
		}
	}

	if (dataChunk->len < 0)
		return -1;

#if 0
	print ("----------");
	print (dataChunk->code);
	print (dataChunk->len);
	print (dataChunk->old);
	print (dataChunk->dna_nr);
	print (dataChunk->nr);
#endif
	return (dataChunk->len+getOffset(flags));
}



//eof
