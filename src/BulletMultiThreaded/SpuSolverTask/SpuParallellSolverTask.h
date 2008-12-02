/*
Bullet Continuous Collision Detection and Physics Library - Parallel solver
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
*/

#ifndef SPU_PARALLELSOLVERTASK_H
#define SPU_PARALLELSOLVERTASK_H

#include "../PlatformDefinitions.h"
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btMatrix3x3.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "../SpuSync.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "LinearMath/btAlignedAllocator.h"
#include "BulletDynamics/ConstraintSolver/btSolverBody.h"
#include "BulletDynamics/ConstraintSolver/btSolverConstraint.h"

ATTRIBUTE_ALIGNED16(struct) ManifoldCellHolder
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	uint32_t					m_hashCellIndex;		
	class btPersistentManifold*	m_manifold;
};

ATTRIBUTE_ALIGNED16(struct) ConstraintCellHolder
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	uint32_t					m_hashCellIndex;		
	uint32_t					m_constraintType;
	class btTypedConstraint*	m_constraint;
};

enum
{
	SPU_HASH_NUMCELLS = 128,
	SPU_HASH_WORDWIDTH = sizeof(uint32_t)*8,
	SPU_HASH_NUMCELLDWORDS = ((SPU_HASH_NUMCELLS + SPU_HASH_WORDWIDTH - 1) / SPU_HASH_WORDWIDTH),
	SPU_HASH_NUMUNUSEDBITS = (SPU_HASH_NUMCELLDWORDS * SPU_HASH_WORDWIDTH) - SPU_HASH_NUMCELLS, 
	SPU_HASH_PHYSSIZE = 4, //TODO: MAKE CONFIGURABLE

	SPU_MAX_BODIES_PER_CELL = 1024,

	SPU_MAX_SPUS = 6
};

enum
{
	CMD_SOLVER_SETUP_BODIES = 1,
	CMD_SOLVER_MANIFOLD_SETUP,
	CMD_SOLVER_CONSTRAINT_SETUP,
	CMD_SOLVER_SOLVE_ITERATE,
	CMD_SOLVER_COPYBACK_BODIES,
	CMD_SOLVER_MANIFOLD_WARMSTART_WRITEBACK
};

struct SpuSolverHashCell
{
	uint16_t						m_numLocalBodies;
	uint16_t						m_solverBodyOffsetListOffset;

	uint16_t						m_numManifolds;
	uint16_t						m_manifoldListOffset;

	uint16_t						m_numContacts;
	uint16_t						m_internalConstraintListOffset;

	uint16_t						m_numConstraints;
	uint16_t						m_constraintListOffset;
};

// Shared data structures
struct SpuSolverHash
{
	// Dependency matrix
	ATTRIBUTE_ALIGNED16(uint32_t m_dependencyMatrix[SPU_HASH_NUMCELLS][SPU_HASH_NUMCELLDWORDS]);
	ATTRIBUTE_ALIGNED16(uint32_t m_currentMask[SPU_MAX_SPUS+1][SPU_HASH_NUMCELLDWORDS]);

	// The hash itself
	ATTRIBUTE_ALIGNED16(SpuSolverHashCell m_Hash[SPU_HASH_NUMCELLS]);

	// Hash meta-data	
};

inline unsigned int spuHash(unsigned int k)  { return k*2654435769u; }
inline unsigned int spuGetHashCellIndex(int x, int y, int z)
{
	//int n = 0x8da6b343 * x + 0xd8163841 * y + 0xcb1ab31f * z;

	int n = x ^ spuHash(y ^ spuHash (z));

	return ((unsigned int)n) & (SPU_HASH_NUMCELLS-1);
}










ATTRIBUTE_ALIGNED16(struct) SpuSolverDataDesc
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	SpuSolverHash*					m_solverHash;
	btSolverBody*					m_solverBodyList;
	btSolverConstraint*				m_solverInternalConstraintList;
	btSolverConstraint*				m_solverConstraintList;
	uint32_t*						m_solverBodyOffsetList;
};


ATTRIBUTE_ALIGNED16(struct) SpuSolverTaskDesc
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	uint32_t						m_solverCommand;
	uint32_t						m_taskId;
	SpuSolverDataDesc				m_solverData;

	// command specific data
	union
	{
		// Body setup
		struct 
		{
			uint32_t				m_startBody;
			uint32_t				m_numBodies;

			class btRigidBody**		m_rbList;
		} m_bodySetup, m_bodyCopyback;

		struct 
		{
			uint32_t				m_startCell;
			uint32_t				m_numCells;

			uint32_t				m_numBodies;
			uint32_t				m_numManifolds;

			ManifoldCellHolder*		m_manifoldHolders;
			ConstraintCellHolder*	m_constraintHolders;
			btContactSolverInfoData	m_solverInfo;
		} m_manifoldSetup;

		struct  
		{
			btSpinlock::SpinVariable*	m_spinLockVar;
		} m_iterate;
	}								m_commandData;
};

void	processSolverTask(void* userPtr, void* lsMemory);
void*	createSolverLocalStoreMemory();

// Helper
inline bool constraintTypeSupported(btTypedConstraintType type)
{
	return type == POINT2POINT_CONSTRAINT_TYPE ||
		type == HINGE_CONSTRAINT_TYPE ||
		type == CONETWIST_CONSTRAINT_TYPE ||
		type == D6_CONSTRAINT_TYPE;
}

#endif
