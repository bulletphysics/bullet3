/*
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*

***************************************************************************************************
**
** profile.cpp
**
** Real-Time Hierarchical Profiling for Game Programming Gems 3
**
** by Greg Hjelstrom & Byon Garrabrant
**
***************************************************************************************************/

// Credits: The Clock class was inspired by the Timer classes in
// Ogre (www.ogre3d.org).

#include "Bullet3Common/b3MinMax.h"
#include "b3Quickprof.h"

#ifndef B3_NO_PROFILE

static b3Clock b3s_profileClock;

inline void b3Profile_Get_Ticks(unsigned long int* ticks)
{
	*ticks = b3s_profileClock.getTimeMicroseconds();
}

inline float b3Profile_Get_Tick_Rate(void)
{
	//	return 1000000.f;
	return 1000.f;
}

/***************************************************************************************************
**
** b3ProfileNode
**
***************************************************************************************************/

/***********************************************************************************************
 * INPUT:                                                                                      *
 * name - pointer to a static string which is the name of this profile node                    *
 * parent - parent pointer                                                                     *
 *                                                                                             *
 * WARNINGS:                                                                                   *
 * The name is assumed to be a static pointer, only the pointer is stored and compared for     *
 * efficiency reasons.                                                                         *
 *=============================================================================================*/
b3ProfileNode::b3ProfileNode(const char* name, b3ProfileNode* parent) : Name(name),
																		TotalCalls(0),
																		TotalTime(0),
																		StartTime(0),
																		RecursionCounter(0),
																		Parent(parent),
																		Child(NULL),
																		Sibling(NULL),
																		m_userPtr(0)
{
	Reset();
}

void b3ProfileNode::CleanupMemory()
{
	delete (Child);
	Child = NULL;
	delete (Sibling);
	Sibling = NULL;
}

b3ProfileNode::~b3ProfileNode(void)
{
	delete (Child);
	delete (Sibling);
}

/***********************************************************************************************
 * INPUT:                                                                                      *
 * name - static string pointer to the name of the node we are searching for                   *
 *                                                                                             *
 * WARNINGS:                                                                                   *
 * All profile names are assumed to be static strings so this function uses pointer compares   *
 * to find the named node.                                                                     *
 *=============================================================================================*/
b3ProfileNode* b3ProfileNode::Get_Sub_Node(const char* name)
{
	// Try to find this sub node
	b3ProfileNode* child = Child;
	while (child)
	{
		if (child->Name == name)
		{
			return child;
		}
		child = child->Sibling;
	}

	// We didn't find it, so add it

	b3ProfileNode* node = new b3ProfileNode(name, this);
	node->Sibling = Child;
	Child = node;
	return node;
}

void b3ProfileNode::Reset(void)
{
	TotalCalls = 0;
	TotalTime = 0.0f;

	if (Child)
	{
		Child->Reset();
	}
	if (Sibling)
	{
		Sibling->Reset();
	}
}

void b3ProfileNode::Call(void)
{
	TotalCalls++;
	if (RecursionCounter++ == 0)
	{
		b3Profile_Get_Ticks(&StartTime);
	}
}

bool b3ProfileNode::Return(void)
{
	if (--RecursionCounter == 0 && TotalCalls != 0)
	{
		unsigned long int time;
		b3Profile_Get_Ticks(&time);
		time -= StartTime;
		TotalTime += (float)time / b3Profile_Get_Tick_Rate();
	}
	return (RecursionCounter == 0);
}

/***************************************************************************************************
**
** b3ProfileIterator
**
***************************************************************************************************/
b3ProfileIterator::b3ProfileIterator(b3ProfileNode* start)
{
	CurrentParent = start;
	CurrentChild = CurrentParent->Get_Child();
}

void b3ProfileIterator::First(void)
{
	CurrentChild = CurrentParent->Get_Child();
}

void b3ProfileIterator::Next(void)
{
	CurrentChild = CurrentChild->Get_Sibling();
}

bool b3ProfileIterator::Is_Done(void)
{
	return CurrentChild == NULL;
}

void b3ProfileIterator::Enter_Child(int index)
{
	CurrentChild = CurrentParent->Get_Child();
	while ((CurrentChild != NULL) && (index != 0))
	{
		index--;
		CurrentChild = CurrentChild->Get_Sibling();
	}

	if (CurrentChild != NULL)
	{
		CurrentParent = CurrentChild;
		CurrentChild = CurrentParent->Get_Child();
	}
}

void b3ProfileIterator::Enter_Parent(void)
{
	if (CurrentParent->Get_Parent() != NULL)
	{
		CurrentParent = CurrentParent->Get_Parent();
	}
	CurrentChild = CurrentParent->Get_Child();
}

/***************************************************************************************************
**
** b3ProfileManager
**
***************************************************************************************************/

b3ProfileNode b3ProfileManager::Root("Root", NULL);
b3ProfileNode* b3ProfileManager::CurrentNode = &b3ProfileManager::Root;
int b3ProfileManager::FrameCounter = 0;
unsigned long int b3ProfileManager::ResetTime = 0;

/***********************************************************************************************
 * b3ProfileManager::Start_Profile -- Begin a named profile                                    *
 *                                                                                             *
 * Steps one level deeper into the tree, if a child already exists with the specified name     *
 * then it accumulates the profiling; otherwise a new child node is added to the profile tree. *
 *                                                                                             *
 * INPUT:                                                                                      *
 * name - name of this profiling record                                                        *
 *                                                                                             *
 * WARNINGS:                                                                                   *
 * The string used is assumed to be a static string; pointer compares are used throughout      *
 * the profiling code for efficiency.                                                          *
 *=============================================================================================*/
void b3ProfileManager::Start_Profile(const char* name)
{
	if (name != CurrentNode->Get_Name())
	{
		CurrentNode = CurrentNode->Get_Sub_Node(name);
	}

	CurrentNode->Call();
}

/***********************************************************************************************
 * b3ProfileManager::Stop_Profile -- Stop timing and record the results.                       *
 *=============================================================================================*/
void b3ProfileManager::Stop_Profile(void)
{
	// Return will indicate whether we should back up to our parent (we may
	// be profiling a recursive function)
	if (CurrentNode->Return())
	{
		CurrentNode = CurrentNode->Get_Parent();
	}
}

/***********************************************************************************************
 * b3ProfileManager::Reset -- Reset the contents of the profiling system                       *
 *                                                                                             *
 *    This resets everything except for the tree structure.  All of the timing data is reset.  *
 *=============================================================================================*/
void b3ProfileManager::Reset(void)
{
	b3s_profileClock.reset();
	Root.Reset();
	Root.Call();
	FrameCounter = 0;
	b3Profile_Get_Ticks(&ResetTime);
}

/***********************************************************************************************
 * b3ProfileManager::Increment_Frame_Counter -- Increment the frame counter                    *
 *=============================================================================================*/
void b3ProfileManager::Increment_Frame_Counter(void)
{
	FrameCounter++;
}

/***********************************************************************************************
 * b3ProfileManager::Get_Time_Since_Reset -- returns the elapsed time since last reset         *
 *=============================================================================================*/
float b3ProfileManager::Get_Time_Since_Reset(void)
{
	unsigned long int time;
	b3Profile_Get_Ticks(&time);
	time -= ResetTime;
	return (float)time / b3Profile_Get_Tick_Rate();
}

#include <stdio.h>

void b3ProfileManager::dumpRecursive(b3ProfileIterator* profileIterator, int spacing)
{
	profileIterator->First();
	if (profileIterator->Is_Done())
		return;

	float accumulated_time = 0, parent_time = profileIterator->Is_Root() ? b3ProfileManager::Get_Time_Since_Reset() : profileIterator->Get_Current_Parent_Total_Time();
	int i;
	int frames_since_reset = b3ProfileManager::Get_Frame_Count_Since_Reset();
	for (i = 0; i < spacing; i++) b3Printf(".");
	b3Printf("----------------------------------\n");
	for (i = 0; i < spacing; i++) b3Printf(".");
	b3Printf("Profiling: %s (total running time: %.3f ms) ---\n", profileIterator->Get_Current_Parent_Name(), parent_time);
	float totalTime = 0.f;

	int numChildren = 0;

	for (i = 0; !profileIterator->Is_Done(); i++, profileIterator->Next())
	{
		numChildren++;
		float current_total_time = profileIterator->Get_Current_Total_Time();
		accumulated_time += current_total_time;
		float fraction = parent_time > B3_EPSILON ? (current_total_time / parent_time) * 100 : 0.f;
		{
			int i;
			for (i = 0; i < spacing; i++) b3Printf(".");
		}
		b3Printf("%d -- %s (%.2f %%) :: %.3f ms / frame (%d calls)\n", i, profileIterator->Get_Current_Name(), fraction, (current_total_time / (double)frames_since_reset), profileIterator->Get_Current_Total_Calls());
		totalTime += current_total_time;
		//recurse into children
	}

	if (parent_time < accumulated_time)
	{
		b3Printf("what's wrong\n");
	}
	for (i = 0; i < spacing; i++) b3Printf(".");
	b3Printf("%s (%.3f %%) :: %.3f ms\n", "Unaccounted:", parent_time > B3_EPSILON ? ((parent_time - accumulated_time) / parent_time) * 100 : 0.f, parent_time - accumulated_time);

	for (i = 0; i < numChildren; i++)
	{
		profileIterator->Enter_Child(i);
		dumpRecursive(profileIterator, spacing + 3);
		profileIterator->Enter_Parent();
	}
}

void b3ProfileManager::dumpAll()
{
	b3ProfileIterator* profileIterator = 0;
	profileIterator = b3ProfileManager::Get_Iterator();

	dumpRecursive(profileIterator, 0);

	b3ProfileManager::Release_Iterator(profileIterator);
}

void b3ProfileManager::dumpRecursive(FILE* f, b3ProfileIterator* profileIterator, int spacing)
{
	profileIterator->First();
	if (profileIterator->Is_Done())
		return;

	float accumulated_time = 0, parent_time = profileIterator->Is_Root() ? b3ProfileManager::Get_Time_Since_Reset() : profileIterator->Get_Current_Parent_Total_Time();
	int i;
	int frames_since_reset = b3ProfileManager::Get_Frame_Count_Since_Reset();
	for (i = 0; i < spacing; i++) fprintf(f, ".");
	fprintf(f, "----------------------------------\n");
	for (i = 0; i < spacing; i++) fprintf(f, ".");
	fprintf(f, "Profiling: %s (total running time: %.3f ms) ---\n", profileIterator->Get_Current_Parent_Name(), parent_time);
	float totalTime = 0.f;

	int numChildren = 0;

	for (i = 0; !profileIterator->Is_Done(); i++, profileIterator->Next())
	{
		numChildren++;
		float current_total_time = profileIterator->Get_Current_Total_Time();
		accumulated_time += current_total_time;
		float fraction = parent_time > B3_EPSILON ? (current_total_time / parent_time) * 100 : 0.f;
		{
			int i;
			for (i = 0; i < spacing; i++) fprintf(f, ".");
		}
		fprintf(f, "%d -- %s (%.2f %%) :: %.3f ms / frame (%d calls)\n", i, profileIterator->Get_Current_Name(), fraction, (current_total_time / (double)frames_since_reset), profileIterator->Get_Current_Total_Calls());
		totalTime += current_total_time;
		//recurse into children
	}

	if (parent_time < accumulated_time)
	{
		fprintf(f, "what's wrong\n");
	}
	for (i = 0; i < spacing; i++)
		fprintf(f, ".");
	fprintf(f, "%s (%.3f %%) :: %.3f ms\n", "Unaccounted:", parent_time > B3_EPSILON ? ((parent_time - accumulated_time) / parent_time) * 100 : 0.f, parent_time - accumulated_time);

	for (i = 0; i < numChildren; i++)
	{
		profileIterator->Enter_Child(i);
		dumpRecursive(f, profileIterator, spacing + 3);
		profileIterator->Enter_Parent();
	}
}

void b3ProfileManager::dumpAll(FILE* f)
{
	b3ProfileIterator* profileIterator = 0;
	profileIterator = b3ProfileManager::Get_Iterator();

	dumpRecursive(f, profileIterator, 0);

	b3ProfileManager::Release_Iterator(profileIterator);
}

#endif  //B3_NO_PROFILE
