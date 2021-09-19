/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2011 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include <stdio.h>
#include "../BulletFileLoader/btBulletFile.h"
#include "BulletDataExtractor.h"

///This ReadBulletSample is kept as simple as possible without dependencies to the Bullet SDK.
///It can be used to load .bullet data for other physics SDKs
///For a more complete example how to load and convert Bullet data using the Bullet SDK check out
///the Bullet/Demos/SerializeDemo and Bullet/Serialize/BulletWorldImporter

int main(int argc, char** argv)
{
	const char* fileName = "testFile.bullet";
	bool verboseDumpAllTypes = false;

	bParse::btBulletFile* bulletFile2 = new bParse::btBulletFile(fileName);

	bool ok = (bulletFile2->getFlags() & bParse::FD_OK) != 0;

	if (ok)
		bulletFile2->parse(verboseDumpAllTypes);
	else
	{
		printf("Error loading file %s.\n", fileName);
		exit(0);
	}
	ok = (bulletFile2->getFlags() & bParse::FD_OK) != 0;
	if (!ok)
	{
		printf("Error parsing file %s.\n", fileName);
		exit(0);
	}

	if (verboseDumpAllTypes)
	{
		bulletFile2->dumpChunks(bulletFile2->getFileDNA());
	}

	btBulletDataExtractor extractor;

	extractor.convertAllObjects(bulletFile2);

	delete bulletFile2;

	return 0;
}
