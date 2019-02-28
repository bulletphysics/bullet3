
#ifndef BLOCK_SOLVER_EXAMPLE_H
#define BLOCK_SOLVER_EXAMPLE_H

enum BlockSolverOptions
{
	BLOCK_SOLVER_SI=1<<0,
	BLOCK_SOLVER_MLCP_PGS = 1 << 1,
	BLOCK_SOLVER_MLCP_DANTZIG = 1 << 2,
	BLOCK_SOLVER_BLOCK = 1 << 3,

	BLOCK_SOLVER_SCENE_MB_STACK= 1 << 5,
	BLOCK_SOLVER_SCENE_CHAIN = 1<< 6,
	
};

class CommonExampleInterface* BlockSolverExampleCreateFunc(struct CommonExampleOptions& options);

#endif  //BLOCK_SOLVER_EXAMPLE_H
