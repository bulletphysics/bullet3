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

/***************************************************************************************************
**
** Real-Time Hierarchical Profiling for Game Programming Gems 3
**
** by Greg Hjelstrom & Byon Garrabrant
**
***************************************************************************************************/

// Credits: The Clock class was inspired by the Timer classes in 
// Ogre (www.ogre3d.org).



#ifndef B3_QUICK_PROF_H
#define B3_QUICK_PROF_H

//To disable built-in profiling, please comment out next line
//#define B3_NO_PROFILE 1
#ifndef B3_NO_PROFILE
#include <stdio.h>//@todo remove this, backwards compatibility
#include "Bullet3Common/b3Scalar.h"
#include "Bullet3Common/b3AlignedAllocator.h"
#include <new>




#include "b3Clock.h"




///A node in the Profile Hierarchy Tree
class	b3ProfileNode {

public:
	b3ProfileNode( const char * name, b3ProfileNode * parent );
	~b3ProfileNode( void );

	b3ProfileNode * Get_Sub_Node( const char * name );

	b3ProfileNode * Get_Parent( void )		{ return Parent; }
	b3ProfileNode * Get_Sibling( void )		{ return Sibling; }
	b3ProfileNode * Get_Child( void )			{ return Child; }

	void				CleanupMemory();
	void				Reset( void );
	void				Call( void );
	bool				Return( void );

	const char *	Get_Name( void )				{ return Name; }
	int				Get_Total_Calls( void )		{ return TotalCalls; }
	float				Get_Total_Time( void )		{ return TotalTime; }
	void*			GetUserPointer() const {return m_userPtr;}
	void			SetUserPointer(void* ptr) { m_userPtr = ptr;}
protected:

	const char *	Name;
	int				TotalCalls;
	float				TotalTime;
	unsigned long int			StartTime;
	int				RecursionCounter;

	b3ProfileNode *	Parent;
	b3ProfileNode *	Child;
	b3ProfileNode *	Sibling;
	void*	m_userPtr;
};

///An iterator to navigate through the tree
class b3ProfileIterator
{
public:
	// Access all the children of the current parent
	void				First(void);
	void				Next(void);
	bool				Is_Done(void);
	bool                Is_Root(void) { return (CurrentParent->Get_Parent() == 0); }

	void				Enter_Child( int index );		// Make the given child the new parent
	void				Enter_Largest_Child( void );	// Make the largest child the new parent
	void				Enter_Parent( void );			// Make the current parent's parent the new parent

	// Access the current child
	const char *	Get_Current_Name( void )			{ return CurrentChild->Get_Name(); }
	int				Get_Current_Total_Calls( void )	{ return CurrentChild->Get_Total_Calls(); }
	float				Get_Current_Total_Time( void )	{ return CurrentChild->Get_Total_Time(); }

	void*	Get_Current_UserPointer( void )			{ return CurrentChild->GetUserPointer(); }
	void	Set_Current_UserPointer(void* ptr) {CurrentChild->SetUserPointer(ptr);}
	// Access the current parent
	const char *	Get_Current_Parent_Name( void )			{ return CurrentParent->Get_Name(); }
	int				Get_Current_Parent_Total_Calls( void )	{ return CurrentParent->Get_Total_Calls(); }
	float				Get_Current_Parent_Total_Time( void )	{ return CurrentParent->Get_Total_Time(); }

	

protected:

	b3ProfileNode *	CurrentParent;
	b3ProfileNode *	CurrentChild;
	

	b3ProfileIterator( b3ProfileNode * start );
	friend	class		b3ProfileManager;
};


///The Manager for the Profile system
class	b3ProfileManager {
public:
	static	void						Start_Profile( const char * name );
	static	void						Stop_Profile( void );

	static	void						CleanupMemory(void)
	{
		Root.CleanupMemory();
	}

	static	void						Reset( void );
	static	void						Increment_Frame_Counter( void );
	static	int						Get_Frame_Count_Since_Reset( void )		{ return FrameCounter; }
	static	float						Get_Time_Since_Reset( void );

	static	b3ProfileIterator *	Get_Iterator( void )	
	{ 
		
		return new b3ProfileIterator( &Root ); 
	}
	static	void						Release_Iterator( b3ProfileIterator * iterator ) { delete ( iterator); }

	static void	dumpRecursive(b3ProfileIterator* profileIterator, int spacing);
	static void	dumpAll();

	static void	dumpRecursive(FILE* f, b3ProfileIterator* profileIterator, int spacing);
	static void	dumpAll(FILE* f);

private:
	static	b3ProfileNode			Root;
	static	b3ProfileNode *			CurrentNode;
	static	int						FrameCounter;
	static	unsigned long int					ResetTime;
};





#else



#endif //#ifndef B3_NO_PROFILE



#endif //B3_QUICK_PROF_H


