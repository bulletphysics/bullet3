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

#include "bMain.h"
#include "bBlenderFile.h"
#include "bDefines.h"
#include "bChunk.h"
#include "bDNA.h"

using namespace bParse;

// ----------------------------------------------------- //
bMain::bMain(bBlenderFile *filePtr, const char *baseName, int fileVersion)
	: mFP(filePtr),
	  mVersion(fileVersion),
	  mName(baseName)
{
	mData.insert(ID_SCE, bListBasePtr());
	mData.insert(ID_LI, bListBasePtr());
	mData.insert(ID_OB, bListBasePtr());
	mData.insert(ID_ME, bListBasePtr());
	mData.insert(ID_CU, bListBasePtr());
	mData.insert(ID_MB, bListBasePtr());
	mData.insert(ID_MA, bListBasePtr());
	mData.insert(ID_TE, bListBasePtr());
	mData.insert(ID_IM, bListBasePtr());
	mData.insert(ID_WV, bListBasePtr());
	mData.insert(ID_LT, bListBasePtr());
	mData.insert(ID_LA, bListBasePtr());
	mData.insert(ID_CA, bListBasePtr());
	mData.insert(ID_IP, bListBasePtr());
	mData.insert(ID_KE, bListBasePtr());
	mData.insert(ID_WO, bListBasePtr());
	mData.insert(ID_SCR, bListBasePtr());
	mData.insert(ID_VF, bListBasePtr());
	mData.insert(ID_TXT, bListBasePtr());
	mData.insert(ID_SO, bListBasePtr());
	mData.insert(ID_GR, bListBasePtr());
	mData.insert(ID_AR, bListBasePtr());
	mData.insert(ID_AC, bListBasePtr());
	mData.insert(ID_NT, bListBasePtr());
	mData.insert(ID_BR, bListBasePtr());
	mData.insert(ID_SCRIPT, bListBasePtr());
}

// ----------------------------------------------------- //
bMain::~bMain()
{
	// allocated data blocks!

	int sz = mPool.size();
	for (int i = 0; i < sz; i++)
	{
		delete[] mPool[i];
	}
}

// ----------------------------------------------------- //
int bMain::getVersion()
{
	return mVersion;
}

// ----------------------------------------------------- //
const char *bMain::getName()
{
	return mName;
}

// ----------------------------------------------------- //
void bMain::addDatablock(void *allocated)
{
	assert(allocated);
	mPool.push_back((bStructHandle *)allocated);
}

// ------------------------------------------------------------//
void bMain::linkList(void *listBasePtr)
{
	struct ListBase  // local Blender::ListBase
	{
		void *first;
		void *last;
	};

	struct Link  // local Blender::Link
	{
		void *next;
		void *prev;
	};

	ListBase *base = (ListBase *)listBasePtr;

	if (!base || !base->first)
		return;

	base->first = mFP->findLibPointer(base->first);
	if (!base->first)
	{
		base->last = 0;
		return;
	}

	void *prev = 0;
	Link *l = (Link *)base->first;
	while (l)
	{
		l->next = mFP->findLibPointer(l->next);
		l->prev = l->next;
		prev = l->next;
		l = (Link *)l->next;
	}
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getListBasePtr(int listBaseCode)
{
	bListBasePtr *ptr = _findCode(listBaseCode);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::_findCode(int code)
{
	bListBasePtr *lbPtr = mData.find(code);
	return lbPtr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getScene()
{
	bListBasePtr *ptr = _findCode(ID_SCE);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getLibrary()
{
	bListBasePtr *ptr = _findCode(ID_LI);
	if (!ptr)
		return 0;
	return ptr;
}
// ------------------------------------------------------------//
bListBasePtr *bMain::getObject()
{
	bListBasePtr *ptr = _findCode(ID_OB);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getMesh()
{
	bListBasePtr *ptr = _findCode(ID_ME);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getCurve()
{
	bListBasePtr *ptr = _findCode(ID_CU);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getMball()
{
	bListBasePtr *ptr = _findCode(ID_MB);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getMat()
{
	bListBasePtr *ptr = _findCode(ID_MA);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getTex()
{
	bListBasePtr *ptr = _findCode(ID_TE);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getImage()
{
	bListBasePtr *ptr = _findCode(ID_IM);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getWave()
{
	bListBasePtr *ptr = _findCode(ID_WV);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getLatt()
{
	bListBasePtr *ptr = _findCode(ID_LT);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getLamp()
{
	bListBasePtr *ptr = _findCode(ID_LA);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getCamera()
{
	bListBasePtr *ptr = _findCode(ID_CA);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getIpo()
{
	bListBasePtr *ptr = _findCode(ID_IP);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getKey()
{
	bListBasePtr *ptr = _findCode(ID_KE);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getWorld()
{
	bListBasePtr *ptr = _findCode(ID_WO);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getScreen()
{
	bListBasePtr *ptr = _findCode(ID_SCR);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getScript()
{
	bListBasePtr *ptr = _findCode(ID_SCRIPT);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getVfont()
{
	bListBasePtr *ptr = _findCode(ID_VF);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getText()
{
	bListBasePtr *ptr = _findCode(ID_TXT);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getSound()
{
	bListBasePtr *ptr = _findCode(ID_SO);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getGroup()
{
	bListBasePtr *ptr = _findCode(ID_GR);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getArmature()
{
	bListBasePtr *ptr = _findCode(ID_AR);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getAction()
{
	bListBasePtr *ptr = _findCode(ID_AC);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getNodetree()
{
	bListBasePtr *ptr = _findCode(ID_NT);
	if (!ptr)
		return 0;
	return ptr;
}

// ------------------------------------------------------------//
bListBasePtr *bMain::getBrush()
{
	bListBasePtr *ptr = _findCode(ID_BR);
	if (!ptr)
		return 0;
	return ptr;
}

//eof
