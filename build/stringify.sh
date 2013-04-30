#!/bin/sh

./premake4_osx --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/RadixSort32Kernels.cl" 	--headerfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/RadixSort32KernelsCL.h" --stringname="radixSort32KernelsCL" stringify
./premake4_osx --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/BoundSearchKernels.cl" 	--headerfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/BoundSearchKernelsCL.h" --stringname="boundSearchKernelsCL" stringify
./premake4_osx --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/PrefixScanKernels.cl" 	--headerfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/PrefixScanKernelsCL.h" --stringname="prefixScanKernelsCL" stringify
./premake4_osx --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/FillKernels.cl" 				--headerfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/FillKernelsCL.h" --stringname="fillKernelsCL" stringify

./premake4_osx --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/BroadphaseCollision/kernels/sap.cl" --headerfile="../src/Bullet3OpenCL/BroadphaseCollision/kernels/sapKernels.h" --stringname="sapCL" stringify
./premake4_osx --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/BroadphaseCollision/kernels/sapFast.cl" --headerfile="../src/Bullet3OpenCL/BroadphaseCollision/kernels/sapFastKernels.h" --stringname="sapFastCL" stringify

./premake4_osx --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/sat.cl" --headerfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/satKernels.h" --stringname="satKernelsCL" stringify
./premake4_osx --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/satClipHullContacts.cl" --headerfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/satClipHullContacts.h" --stringname="satClipKernelsCL" stringify
./premake4_osx --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/primitiveContacts.cl" --headerfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/primitiveContacts.h" --stringname="primitiveContactsKernelsCL" stringify
./premake4_osx --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/bvhTraversal.cl" --headerfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/bvhTraversal.h" --stringname="bvhTraversalKernelCL" stringify


./premake4_osx --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody//kernels/integrateKernel.cl" --headerfile="../src/Bullet3OpenCL/RigidBody//kernels/integrateKernel.h" --stringname="integrateKernelCL" stringify
./premake4_osx --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody//kernels/updateAabbsKernel.cl" --headerfile="../src/Bullet3OpenCL/RigidBody//kernels/updateAabbsKernel.h" --stringname="updateAabbsKernelCL" stringify
./premake4_osx --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody//kernels/solverSetup.cl" --headerfile="../src/Bullet3OpenCL/RigidBody//kernels/solverSetup.h" --stringname="solverSetupCL" stringify
./premake4_osx --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody//kernels/solverSetup2.cl" --headerfile="../src/Bullet3OpenCL/RigidBody//kernels/solverSetup2.h" --stringname="solverSetup2CL" stringify
./premake4_osx --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody//kernels/batchingKernels.cl" --headerfile="../src/Bullet3OpenCL/RigidBody//kernels/batchingKernels.h" --stringname="batchingKernelsCL" stringify
./premake4_osx --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody//kernels/batchingKernelsNew.cl" --headerfile="../src/Bullet3OpenCL/RigidBody//kernels/batchingKernelsNew.h" --stringname="batchingKernelsNewCL" stringify
./premake4_osx --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody//kernels/solverUtils.cl" --headerfile="../src/Bullet3OpenCL/RigidBody//kernels/solverUtils.h" --stringname="solverUtilsCL" stringify
./premake4_osx --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody//kernels/solveContact.cl" --headerfile="../src/Bullet3OpenCL/RigidBody//kernels/solveContact.h" --stringname="solveContactCL" stringify
./premake4_osx --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody//kernels/solveFriction.cl" --headerfile="../src/Bullet3OpenCL/RigidBody//kernels/solveFriction.h" --stringname="solveFrictionCL" stringify

