
@echo off


premake4 --file=stringifyKernel.lua --kernelfile="../opencl/vector_add/VectorAddKernels.cl" --headerfile="../opencl/vector_add/VectorAddKernels.h" --stringname="vectorAddCL" stringify

premake4 --file=stringifyKernel.lua --kernelfile="../opencl/parallel_primitives/kernels/RadixSort32Kernels.cl" 	--headerfile="../opencl/parallel_primitives/kernels/RadixSort32KernelsCL.h" --stringname="radixSort32KernelsCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../opencl/parallel_primitives/kernels/BoundSearchKernels.cl" 	--headerfile="../opencl/parallel_primitives/kernels/BoundSearchKernelsCL.h" --stringname="boundSearchKernelsCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../opencl/parallel_primitives/kernels/PrefixScanKernels.cl" 	--headerfile="../opencl/parallel_primitives/kernels/PrefixScanKernelsCL.h" --stringname="prefixScanKernelsCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../opencl/parallel_primitives/kernels/FillKernels.cl" 				--headerfile="../opencl/parallel_primitives/kernels/FillKernelsCL.h" --stringname="fillKernelsCL" stringify


pause