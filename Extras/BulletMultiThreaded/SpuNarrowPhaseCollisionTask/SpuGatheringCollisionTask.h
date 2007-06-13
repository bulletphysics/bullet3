
#ifndef SPU_GATHERING_COLLISION_TASK_H
#define SPU_GATHERING_COLLISION_TASK_H

struct SpuGatherAndProcessPairsTaskDesc;

void	processCollisionTask(void* userPtr, void* lsMemory);

void*	createCollisionLocalStoreMemory();

#endif //SPU_GATHERING_COLLISION_TASK_H