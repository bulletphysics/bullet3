extern "C"
{
void cudaInit(int argc, char **argv);

void allocateArray(void **devPtr, int size);
void freeArray(void *devPtr);

void threadSync();

void copyArrayFromDevice(void* host, const void* device, unsigned int vbo, int size);
void copyArrayToDevice(void* device, const void* host, int offset, int size);
void registerGLBufferObject(unsigned int vbo);
void unregisterGLBufferObject(unsigned int vbo);

void setParameters(SimParams *hostParams);

void 
integrateSystem(uint vboOldPos, uint vboNewPos, 
                float* oldVel, float* newVel, 
                float deltaTime,
                int numBodies);

void 
updateGrid(uint    vboPos, 
           uint*   gridCounters,
           uint*   gridCells,
           uint    numBodies,
           uint    numCells);

void 
calcHash(uint    vboPos, 
         uint*   particleHash,
         int     numBodies);

void 
reorderDataAndFindCellStart(uint*  particleHash,
							uint   vboOldPos,
							float* oldVel,
							float* sortedPos,
							float* sortedVel,
							uint*  cellStart,
							uint   numBodies,
							uint   numCells);

void 
findCellStart(	uint*  particleHash,
				uint*  cellStart,
				uint   numBodies,
				uint   numCells);
							
void
collide(uint   vboOldPos, uint vboNewPos,
        float* sortedPos, float* sortedVel,
        float* oldVel, float* newVel,
        uint*  gridCounters,
        uint*  gridCells,
        uint*  particleHash,
        uint*  cellStart,
        uint   numBodies,
        uint   numCells,
        uint   maxParticlesPerCell);
        
void
btCudaFindOverlappingPairs(	float*	pAABB,
							uint*	pParticleHash,
							uint*	pCellStart,
							uint*	pPairBuff,
							uint*	pPairBuffStartCurr,
							uint	numParticles);
							
void
btCudaComputePairCacheChanges(uint* pPairBuff, uint* pPairBuffStartCurr, uint* pPairScan, uint numParticles);
							
        
void btCudaSqueezeOverlappingPairBuff(uint* pPairBuff, uint* pPairBuffStartCurr, uint* pPairScan, uint* pPairOut, uint numParticles);
        

}
