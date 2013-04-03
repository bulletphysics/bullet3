
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
premake4 --file=stringifyKernel.lua --kernelfile="../opencl/gpu_sat/kernels/primitiveContacts.cl" --headerfile="../opencl/gpu_sat/kernels/primitiveContacts.h" --stringname="primitiveContactsKernelsCL" stringify

premake4 --file=stringifyKernel.lua --kernelfile="../opencl/gpu_sat/kernels/bvhTraversal.cl" --headerfile="../opencl/gpu_sat/kernels/bvhTraversal.h" --stringname="bvhTraversalKernelCL" stringify


premake4 --file=stringifyKernel.lua --kernelfile="../opencl/gpu_rigidbody/kernels/integrateKernel.cl" --headerfile="../opencl/gpu_rigidbody/kernels/integrateKernel.h" --stringname="integrateKernelCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../opencl/gpu_rigidbody/kernels/updateAabbsKernel.cl" --headerfile="../opencl/gpu_rigidbody/kernels/updateAabbsKernel.h" --stringname="updateAabbsKernelCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../opencl/gpu_rigidbody/kernels/solverSetup.cl" --headerfile="../opencl/gpu_rigidbody/kernels/solverSetup.h" --stringname="solverSetupCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../opencl/gpu_rigidbody/kernels/solverSetup2.cl" --headerfile="../opencl/gpu_rigidbody/kernels/solverSetup2.h" --stringname="solverSetup2CL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../opencl/gpu_rigidbody/kernels/batchingKernels.cl" --headerfile="../opencl/gpu_rigidbody/kernels/batchingKernels.h" --stringname="batchingKernelsCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../opencl/gpu_rigidbody/kernels/batchingKernelsNew.cl" --headerfile="../opencl/gpu_rigidbody/kernels/batchingKernelsNew.h" --stringname="batchingKernelsNewCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../opencl/gpu_rigidbody/kernels/solverUtils.cl" --headerfile="../opencl/gpu_rigidbody/kernels/solverUtils.h" --stringname="solverUtilsCL" stringify


premake4 --file=stringifyKernel.lua --kernelfile="../opencl/gpu_rigidbody/kernels/solveContact.cl" --headerfile="../opencl/gpu_rigidbody/kernels/solveContact.h" --stringname="solveContactCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../opencl/gpu_rigidbody/kernels/solveFriction.cl" --headerfile="../opencl/gpu_rigidbody/kernels/solveFriction.h" --stringname="solveFrictionCL" stringify




pause