



//OpenCL 1.1 has atomic_inc build-in (no extension needed)
//see http://www.khronos.org/registry/cl/sdk/1.1/docs/man/xhtml/atomic_inc.html
__kernel void  globalAtomicKernelOpenCL1_1( volatile __global int* counter)
{
	atomic_inc(counter);
}

//OpenCL 1.1 atomic device counters extension, usually faster on current AMD hardware
//http://www.khronos.org/registry/cl/extensions/ext/cl_ext_atomic_counters_32.txt
#pragma OPENCL EXTENSION cl_ext_atomic_counters_32 : enable
__kernel void  counterAtomicKernelExt( counter32_t counter)
{
	atomic_inc(counter);
}


//OpenCL 1.0 optional extension, using atom_inc
//see http://www.khronos.org/registry/cl/sdk/1.0/docs/man/xhtml/cl_khr_global_int32_base_atomics.html
#pragma OPENCL EXTENSION cl_khr_global_int32_base_atomics : enable //atomic_inc
__kernel void  globalAtomicKernelExt( __global int* counter)
{
	atom_inc(counter);
}


__kernel void  globalAtomicKernelCounters32Broken( __global int* counter)
{
	(*counter)++;
}

