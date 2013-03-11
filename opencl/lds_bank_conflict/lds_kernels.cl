
#define TILE_DIM  32
#define BLOCK_ROWS  8


/*// simple copy kernel (CUDA)
// Used as reference case representing best effective bandwidth.
__global__ void copy(float *odata, const float *idata)
{
  int x = blockIdx.x * TILE_DIM + threadIdx.x;
  int y = blockIdx.y * TILE_DIM + threadIdx.y;
  int width = gridDim.x * TILE_DIM;

  for (int j = 0; j < TILE_DIM; j+= BLOCK_ROWS)
	odata[(y+j)*width + x] = idata[(y+j)*width + x];
}
*/
// simple copy kernel (OpenCL)
__kernel void copyKernel(__global float* odata, __global const float* idata)
{
  int x = get_group_id(0) * get_num_groups(0) + get_local_id(0);
  int y = get_group_id(1) * get_num_groups(1) + get_local_id(1);
  int width = get_num_groups(0) * get_local_size(0);
  for (int j = 0; j < get_num_groups(1); j+= get_local_size(1))
  {
	odata[(y+j)*width + x] = idata[(y+j)*width + x];
  }
}

/*
// copy kernel using shared memory (CUDA)
// Also used as reference case, demonstrating effect of using shared memory.
__global__ void copySharedMem(float *odata, const float *idata)
{
  __shared__ float tile[TILE_DIM * TILE_DIM];
  
  int x = blockIdx.x * TILE_DIM + threadIdx.x;
  int y = blockIdx.y * TILE_DIM + threadIdx.y;
  int width = gridDim.x * TILE_DIM;

  for (int j = 0; j < TILE_DIM; j += BLOCK_ROWS)
	 tile[(threadIdx.y+j)*TILE_DIM + threadIdx.x] = idata[(y+j)*width + x];

  __syncthreads();

  for (int j = 0; j < TILE_DIM; j += BLOCK_ROWS)
	 odata[(y+j)*width + x] = tile[(threadIdx.y+j)*TILE_DIM + threadIdx.x];          
}
*/

// copy kernel using shared memory (OpenCL)
// Also used as reference case, demonstrating effect of using shared memory.
__kernel void copySharedMemKernel(__global float *odata, __global const float *idata)
{
  __local float tile[TILE_DIM * TILE_DIM];
  
  int x = get_group_id(0) * get_num_groups(0) + get_local_id(0);
  int y = get_group_id(1) * get_num_groups(1) + get_local_id(1);
  int width = get_num_groups(0) * get_local_size(0);

  for (int j = 0; j < TILE_DIM; j += BLOCK_ROWS)
	 tile[(get_local_id(1)+j)*TILE_DIM + get_local_id(0)] = idata[(y+j)*width + x];

  barrier(CLK_LOCAL_MEM_FENCE);

  for (int j = 0; j < TILE_DIM; j += BLOCK_ROWS)
	 odata[(y+j)*width + x] = tile[(get_local_id(1)+j)*TILE_DIM + get_local_id(0)];
}

/*
// naive transpose (CUDA)
// Simplest transpose; doesn't use shared memory.
// Global memory reads are coalesced but writes are not.
__global__ void transposeNaive(float *odata, const float *idata)
{
  int x = blockIdx.x * TILE_DIM + threadIdx.x;
  int y = blockIdx.y * TILE_DIM + threadIdx.y;
  int width = gridDim.x * TILE_DIM;

  for (int j = 0; j < TILE_DIM; j+= BLOCK_ROWS)
	odata[x*width + (y+j)] = idata[(y+j)*width + x];
}
*/

// naive transpose (OpenCL)
// Simplest transpose; doesn't use shared memory.
// Global memory reads are coalesced but writes are not.
__kernel void transposeNaiveKernel(__global float *odata, __global const float *idata)
{
  int x = get_group_id(0) * get_num_groups(0) + get_local_id(0);
  int y = get_group_id(1) * get_num_groups(1) + get_local_id(1);
  int width = get_num_groups(0) * get_local_size(0);

  for (int j = 0; j < TILE_DIM; j+= BLOCK_ROWS)
	odata[x*width + (y+j)] = idata[(y+j)*width + x];
}

/*
// coalesced transpose (CUDA)
// Uses shared memory to achieve coalesing in both reads and writes
// Tile width == #banks causes shared memory bank conflicts.
__global__ void transposeCoalesced(float *odata, const float *idata)
{
  __shared__ float tile[TILE_DIM][TILE_DIM];
	
  int x = blockIdx.x * TILE_DIM + threadIdx.x;
  int y = blockIdx.y * TILE_DIM + threadIdx.y;
  int width = gridDim.x * TILE_DIM;

  for (int j = 0; j < TILE_DIM; j += BLOCK_ROWS)
	 tile[threadIdx.y+j][threadIdx.x] = idata[(y+j)*width + x];

  __syncthreads();

  x = blockIdx.y * TILE_DIM + threadIdx.x;  // transpose block offset
  y = blockIdx.x * TILE_DIM + threadIdx.y;

  for (int j = 0; j < TILE_DIM; j += BLOCK_ROWS)
	 odata[(y+j)*width + x] = tile[threadIdx.x][threadIdx.y + j];
}
*/

// coalesced transpose (OpenCL)
// Uses shared memory to achieve coalesing in both reads and writes
// Tile width == #banks causes shared memory bank conflicts.
__kernel void transposeCoalescedKernel(__global float *odata, __global const float *idata)
{
  __local float tile[TILE_DIM][TILE_DIM];
	
  int x = get_group_id(0) * get_num_groups(0) + get_local_id(0);
  int y = get_group_id(1) * get_num_groups(1) + get_local_id(1);
  int width = get_num_groups(0) * get_local_size(0);
    
  for (int j = 0; j < TILE_DIM; j += BLOCK_ROWS)
	 tile[get_local_id(1)+j][get_local_id(0)] = idata[(y+j)*width + x];

  barrier(CLK_LOCAL_MEM_FENCE);

  x = get_group_id(1) * TILE_DIM + get_local_id(0);
  y = get_group_id(0) * TILE_DIM + get_local_id(1);
  
  for (int j = 0; j < TILE_DIM; j += BLOCK_ROWS)
	 odata[(y+j)*width + x] = tile[get_local_id(0)][get_local_id(1) + j];
}


// No bank-conflict transpose (OpenCL)
// Same as transposeCoalesced except the first tile dimension is padded 
// to avoid shared memory bank conflicts.
__kernel void transposeNoBankConflictsKernel(__global float *odata, __global const float *idata)
{
  __local float tile[TILE_DIM][TILE_DIM+1];
	
  int x = get_group_id(0) * get_num_groups(0) + get_local_id(0);
  int y = get_group_id(1) * get_num_groups(1) + get_local_id(1);
  int width = get_num_groups(0) * get_local_size(0);
    
  for (int j = 0; j < TILE_DIM; j += BLOCK_ROWS)
	 tile[get_local_id(1)+j][get_local_id(0)] = idata[(y+j)*width + x];

  barrier(CLK_LOCAL_MEM_FENCE);

  x = get_group_id(1) * TILE_DIM + get_local_id(0);
  y = get_group_id(0) * TILE_DIM + get_local_id(1);
  
  for (int j = 0; j < TILE_DIM; j += BLOCK_ROWS)
	 odata[(y+j)*width + x] = tile[get_local_id(0)][get_local_id(1) + j];
}



