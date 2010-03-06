/*
 *	OPCODE - Optimized Collision Detection
 * http://www.codercorner.com/Opcode.htm
 * 
 * Copyright (c) 2001-2008 Pierre Terdiman,  pierre@codercorner.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains an implementation of the sweep-and-prune algorithm (moved from Z-Collide)
 *	\file		OPC_SweepAndPrune.h
 *	\author		Pierre Terdiman
 *	\date		January, 29, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __OPC_SWEEPANDPRUNE_H__
#define __OPC_SWEEPANDPRUNE_H__

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 *	User-callback, called by OPCODE for each colliding pairs.
	 *	\param		id0			[in] id of colliding object
	 *	\param		id1			[in] id of colliding object
	 *	\param		user_data	[in] user-defined data
	 *	\return		TRUE to continue enumeration
	 */
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	typedef BOOL	(*PairCallback)	(udword id0, udword id1, void* user_data);

	class SAP_Element;
	class SAP_EndPoint;
	class SAP_Box;

	class OPCODE_API SAP_PairData
	{
		public:
								SAP_PairData();
								~SAP_PairData();

				bool			Init(udword nb_objects);

				void			AddPair(udword id1, udword id2);
				void			RemovePair(udword id1, udword id2);

				void			DumpPairs(Pairs& pairs)								const;
				void			DumpPairs(PairCallback callback, void* user_data)	const;
		private:
				udword			mNbElements;		//!< Total number of elements in the pool
				udword			mNbUsedElements;	//!< Number of used elements
				SAP_Element*	mElementPool;		//!< Array of mNbElements elements
				SAP_Element*	mFirstFree;			//!< First free element in the pool

				udword			mNbObjects;			//!< Max number of objects we can handle
				SAP_Element**	mArray;				//!< Pointers to pool
		// Internal methods
				SAP_Element*	GetFreeElem(udword id, SAP_Element* next, udword* remap=null);
		inline_	void			FreeElem(SAP_Element* elem);
				void			Release();
	};

	class OPCODE_API SweepAndPrune
	{
		public:
								SweepAndPrune();
								~SweepAndPrune();

				bool			Init(udword nb_objects, const AABB** boxes);
				bool			UpdateObject(udword i, const AABB& box);

				void			GetPairs(Pairs& pairs)								const;
				void			GetPairs(PairCallback callback, void* user_data)	const;
		private:
				SAP_PairData	mPairs;

				udword			mNbObjects;
				SAP_Box*		mBoxes;
				SAP_EndPoint*	mList[3];
		// Internal methods
				bool			CheckListsIntegrity();
	};

#endif //__OPC_SWEEPANDPRUNE_H__