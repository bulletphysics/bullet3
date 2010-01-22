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

#ifndef __BMAIN_H__
#define __BMAIN_H__

#include "bCommon.h"
#include "bChunk.h"
#include "LinearMath/btHashMap.h"


namespace bParse
{
	class bDNA;

	class bBlenderFile;
};



namespace bParse {


	// ----------------------------------------------------- //
	
	typedef	btHashMap<btHashInt,bListBasePtr> bMainDataMap;



	// ----------------------------------------------------- //
	class bMain
	{
	//private:
	public:
		bBlenderFile*			mFP;
		bListBasePtr	mPool;

		int				mVersion;
		const char*		mName;

		bMainDataMap	mData;

	


		bListBasePtr *_findCode(int code);

	public:
		bMain(bBlenderFile  *filePtr, const char *baseName, int fileVersion);
		~bMain();

		int getVersion();
		const char *getName();

		bListBasePtr *getListBasePtr(int listBaseCode);


		bListBasePtr *getScene();
		bListBasePtr *getLibrary();
		bListBasePtr *getObject();
		bListBasePtr *getMesh();
		bListBasePtr *getCurve();
		bListBasePtr *getMball();
		bListBasePtr *getMat();
		bListBasePtr *getTex();
		bListBasePtr *getImage();
		bListBasePtr *getWave();
		bListBasePtr *getLatt();
		bListBasePtr *getLamp();
		bListBasePtr *getCamera();
		bListBasePtr *getIpo();
		bListBasePtr *getKey();
		bListBasePtr *getWorld();
		bListBasePtr *getScreen();
		bListBasePtr *getScript();
		bListBasePtr *getVfont();
		bListBasePtr *getText();
		bListBasePtr *getSound();
		bListBasePtr *getGroup();
		bListBasePtr *getArmature();
		bListBasePtr *getAction();
		bListBasePtr *getNodetree();
		bListBasePtr *getBrush();


		
		// tracking allocated memory
		void addDatablock(void *allocated);


		// --
		
		void linkList(void *listBasePtr);
	};
}


#endif//__BMAIN_H__
