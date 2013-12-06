///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 *	OPCODE - Optimized Collision Detection
 *	Copyright (C) 2001 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/Opcode.htm
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains an array-based version of the sweep-and-prune algorithm
 *	\file		OPC_ArraySAP.h
 *	\author		Pierre Terdiman
 *	\date		December, 2, 2007
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef OPC_ARRAYSAP_H
#define OPC_ARRAYSAP_H

#pragma pack(1)
	struct OPCODE_API ASAP_Pair
	{
		uword		id0;
		uword		id1;
		const void*	object0;
		const void*	object1;
//#ifdef PAIR_USER_DATA
		void*		userData;
//#endif
		inline_		const void*	GetObject0()		const	{ return (const void*)(size_t(object0) & ~3);		}
		inline_		const void*	GetObject1()		const	{ return (const void*)(size_t(object1) & ~3);		}
		inline_		size_t		IsInArray()			const	{ return size_t(object0) & 1;						}
		inline_		size_t		IsRemoved()			const	{ return size_t(object1) & 1;						}
		inline_		size_t		IsNew()				const	{ return size_t(object0) & 2;						}
		private:
		inline_		void		SetInArray()				{ object0 = (const void*)(size_t(object0) | 1);		}
		inline_		void		SetRemoved()				{ object1 = (const void*)(size_t(object1) | 1);		}
		inline_		void		SetNew()					{ object0 = (const void*)(size_t(object0) | 2);		}
		inline_		void		ClearInArray()				{ object0 = (const void*)(size_t(object0) & ~1);	}
		inline_		void		ClearRemoved()				{ object1 = (const void*)(size_t(object1) & ~1);	}
		inline_		void		ClearNew()					{ object0 = (const void*)(size_t(object0) & ~2);	}

		friend class ArraySAP;
	};
#pragma pack()

	class OPCODE_API ASAP_PairManager
	{
		public:
									ASAP_PairManager();
									~ASAP_PairManager();

				void				Purge();
				void				ShrinkMemory();

				const ASAP_Pair*	AddPair		(uword id0, uword id1, const void* object0, const void* object1);
				bool				RemovePair	(uword id0, uword id1);
				bool				RemovePairs	(const BitArray& array);
				const ASAP_Pair*	FindPair	(uword id0, uword id1)	const;
		inline_	udword				GetPairIndex(const ASAP_Pair* pair)	const
									{
										return ((udword)((size_t(pair) - size_t(mActivePairs)))/sizeof(ASAP_Pair));
									}

				udword				mHashSize;
				udword				mMask;
				udword				mNbActivePairs;
				udword*				mHashTable;
				udword*				mNext;
				ASAP_Pair*			mActivePairs;
		inline_	ASAP_Pair*			FindPair(uword id0, uword id1, udword hash_value) const;
				void				RemovePair(uword id0, uword id1, udword hash_value, udword pair_index);
				void				ReallocPairs();
	};

	typedef void* (*SAP_CreatePair)(const void* object0, const void* object1, void* user_data);
	typedef void  (*SAP_DeletePair)(const void* object0, const void* object1, void* user_data, void* pair_user_data);

	// Forward declarations
	class ASAP_EndPoint;
	class ASAP_Box;
	struct IAABB;
	struct CreateData;

	class OPCODE_API ArraySAP : public Allocateable
	{
		public:
									ArraySAP();
									~ArraySAP();

				udword				AddObject(void* object, uword guid, const AABB& box);
				bool				RemoveObject(udword handle);
				bool				UpdateObject(udword handle, const AABB& box);

				udword				DumpPairs(SAP_CreatePair create_cb, SAP_DeletePair delete_cb, void* cb_user_data, ASAP_Pair** pairs=null);
		private:
				Container			mData;
				ASAP_PairManager	mPairs;

		inline_	void				AddPair(const void* object0, const void* object1, uword id0, uword id1)
									{
										ASSERT(object0);
										ASAP_Pair* UP = (ASAP_Pair*)mPairs.AddPair(id0, id1, null, null);
										ASSERT(UP);

										if(UP->object0)
										{
											// Persistent pair
										}
										else
										{
											// New pair
											ASSERT(!(int(object0)&1));
											ASSERT(!(int(object1)&1));
											UP->object0 = object0;
											UP->object1 = object1;
											UP->SetInArray();
											mData.Add(mPairs.GetPairIndex(UP));
											UP->SetNew();
										}
										UP->ClearRemoved();
									}

		inline_	void				RemovePair(const void* object0, const void* object1, uword id0, uword id1)
									{
										ASAP_Pair* UP = (ASAP_Pair*)mPairs.FindPair(id0, id1);
										if(UP)
										{
											if(!UP->IsInArray())
											{
												UP->SetInArray();
												mData.Add(mPairs.GetPairIndex(UP));
											}
											UP->SetRemoved();
										}
									}

				udword				mNbBoxes;
				udword				mMaxNbBoxes;
				ASAP_Box*			mBoxes;
				ASAP_EndPoint*		mEndPoints[3];
				udword				mFirstFree;

				void				ResizeBoxArray();
			// For batch creation
				Container			mCreated;
				void				BatchCreate();
				void				InsertEndPoints(udword axis, const ASAP_EndPoint* end_points, udword nb_endpoints);
				bool				CompleteBoxPruning2(udword nb, const IAABB* array, const Axes& axes, const CreateData* batched);
				bool				BipartiteBoxPruning2(udword nb0, const IAABB* array0, udword nb1, const IAABB* array1, const Axes& axes, const CreateData* batched, const udword* box_indices);
			// For batch removal
				Container			mRemoved;
				void				BatchRemove();
	};

#endif // OPC_ARRAYSAP_H
