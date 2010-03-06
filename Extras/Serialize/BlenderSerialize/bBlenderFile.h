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

#ifndef B_BLENDER_FILE_H
#define B_BLENDER_FILE_H


#include "bFile.h"

namespace bParse {

	// ----------------------------------------------------- //
	class bBlenderFile : public bFile
	{

	protected:
		bMain*				mMain;

		bStructHandle*		m_glob;

		
	public:

		bBlenderFile(const char* fileName);

		bBlenderFile(char *memoryBuffer, int len);

		virtual ~bBlenderFile();

		bMain* getMain();

		virtual	void	addDataBlock(char* dataBlock);

		bStructHandle*		getFileGlobal()
		{
			return m_glob;
		}

		// experimental
		virtual int		write(const char* fileName, bool fixupPointers = false);

		virtual	void	parse(bool verboseDumpAllTypes);

		virtual	void parseData();

		virtual	void	writeDNA(FILE* fp);

	};
};

#endif //B_BLENDER_FILE_H
