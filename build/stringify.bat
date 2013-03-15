
@echo off


premake4 --file=stringifyKernel.lua --kernelfile="../opencl/vector_add/VectorAddKernels.cl" --headerfile="../opencl/vector_add/VectorAddKernels.h" --stringname="vectorAddCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../opencl/parallel_primitives/kernels/RadixSort32Kernels.cl" 	--headerfile="../opencl/parallel_primitives/kernels/RadixSort32KernelsCL.h" --stringname="radixSort32KernelsCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../opencl/parallel_primitives/kernels/BoundSearchKernels.cl" 	--headerfile="../opencl/parallel_primitives/kernels/BoundSearchKernelsCL.h" --stringname="boundSearchKernelsCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../opencl/parallel_primitives/kernels/PrefixScanKernels.cl" 	--headerfile="../opencl/parallel_primitives/kernels/PrefixScanKernelsCL.h" --stringname="prefixScanKernelsCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../opencl/parallel_primitives/kernels/FillKernels.cl" 				--headerfile="../opencl/parallel_primitives/kernels/FillKernelsCL.h" --stringname="fillKernelsCL" stringify

premake4 --file=stringifyKernel.lua --kernelfile="../opencl/gpu_broadphase/kernels/sap.cl" --headerfile="../opencl/gpu_broadphase/kernels/sapKernels.h" --stringname="sapCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../opencl/gpu_broadphase/kernels/sapFast.cl" --headerfile="../opencl/gpu_broadphase/kernels/sapFastKernels.h" --stringname="sapFastCL" stringify

premake4 --file=stringifyKernel.lua --kernelfile="../opencl/gpu_sat/kernels/sat.cl" --headerfile="../opencl/gpu_sat/kernels/satKernels.h" --stringname="satKernelsCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../opencl/gpu_sat/kernels/satClipHullContacts.cl" --headerfile="../opencl/gpu_sat/kernels/satClipHullContacts.h" --stringname="satClipKernelsCL" stringify

premake4 --file=stringifyKernel.lua --kernelfile="../opencl/gpu_rigidbody/kernels/integrateKernel.cl" --headerfile="../opencl/gpu_rigidbody/kernels/integrateKernel.h" --stringname="integrateKernelCL" stringify



pause