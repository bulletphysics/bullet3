
rem @echo off


premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/RadixSort32Kernels.cl" 	--headerfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/RadixSort32KernelsCL.h" --stringname="radixSort32KernelsCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/BoundSearchKernels.cl" 	--headerfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/BoundSearchKernelsCL.h" --stringname="boundSearchKernelsCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/PrefixScanKernels.cl" 	--headerfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/PrefixScanKernelsCL.h" --stringname="prefixScanKernelsCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/PrefixScanFloat4Kernels.cl" 	--headerfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/PrefixScanKernelsFloat4CL.h" --stringname="prefixScanKernelsFloat4CL" stringify

premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/FillKernels.cl" 				--headerfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/FillKernelsCL.h" --stringname="fillKernelsCL" stringify

premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/BroadphaseCollision/kernels/sap.cl" --headerfile="../src/Bullet3OpenCL/BroadphaseCollision/kernels/sapKernels.h" --stringname="sapCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/BroadphaseCollision/kernels/sapFast.cl" --headerfile="../src/Bullet3OpenCL/BroadphaseCollision/kernels/sapFastKernels.h" --stringname="sapFastCL" stringify

premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/BroadphaseCollision/kernels/gridBroadphase.cl" --headerfile="../src/Bullet3OpenCL/BroadphaseCollision/kernels/gridBroadphaseKernels.h" --stringname="gridBroadphaseCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/BroadphaseCollision/kernels/parallelLinearBvh.cl" --headerfile="../src/Bullet3OpenCL/BroadphaseCollision/kernels/parallelLinearBvhKernels.h" --stringname="parallelLinearBvhCL" stringify



premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/sat.cl" --headerfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/satKernels.h" --stringname="satKernelsCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/satConcave.cl" --headerfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/satConcaveKernels.h" --stringname="satConcaveKernelsCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/mpr.cl" --headerfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/mprKernels.h" --stringname="mprKernelsCL" stringify



premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/satClipHullContacts.cl" --headerfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/satClipHullContacts.h" --stringname="satClipKernelsCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/primitiveContacts.cl" --headerfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/primitiveContacts.h" --stringname="primitiveContactsKernelsCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/bvhTraversal.cl" --headerfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/bvhTraversal.h" --stringname="bvhTraversalKernelCL" stringify


premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody/kernels/integrateKernel.cl" --headerfile="../src/Bullet3OpenCL/RigidBody/kernels/integrateKernel.h" --stringname="integrateKernelCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody/kernels/updateAabbsKernel.cl" --headerfile="../src/Bullet3OpenCL/RigidBody/kernels/updateAabbsKernel.h" --stringname="updateAabbsKernelCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody/kernels/solverSetup.cl" --headerfile="../src/Bullet3OpenCL/RigidBody/kernels/solverSetup.h" --stringname="solverSetupCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody/kernels/solverSetup2.cl" --headerfile="../src/Bullet3OpenCL/RigidBody/kernels/solverSetup2.h" --stringname="solverSetup2CL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody/kernels/batchingKernels.cl" --headerfile="../src/Bullet3OpenCL/RigidBody/kernels/batchingKernels.h" --stringname="batchingKernelsCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody/kernels/batchingKernelsNew.cl" --headerfile="../src/Bullet3OpenCL/RigidBody/kernels/batchingKernelsNew.h" --stringname="batchingKernelsNewCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody/kernels/solverUtils.cl" --headerfile="../src/Bullet3OpenCL/RigidBody/kernels/solverUtils.h" --stringname="solverUtilsCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody/kernels/solveContact.cl" --headerfile="../src/Bullet3OpenCL/RigidBody/kernels/solveContact.h" --stringname="solveContactCL" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody/kernels/solveFriction.cl" --headerfile="../src/Bullet3OpenCL/RigidBody/kernels/solveFriction.h" --stringname="solveFrictionCL" stringify

premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody/kernels/jointSolver.cl" --headerfile="../src/Bullet3OpenCL/RigidBody/kernels/jointSolver.h" --stringname="solveConstraintRowsCL" stringify



premake4 --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/Raycast/kernels/rayCastKernels.cl" --headerfile="../src/Bullet3OpenCL/Raycast/kernels/rayCastKernels.h" --stringname="rayCastKernelCL" stringify

premake4 --file=stringifyKernel.lua --kernelfile="../btgui/OpenGLWindow/Shaders/instancingVS.glsl" --headerfile="../btgui/OpenGLWindow/Shaders/instancingVS.h" --stringname="instancingVertexShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../btgui/OpenGLWindow/Shaders/instancingPS.glsl" --headerfile="../btgui/OpenGLWindow/Shaders/instancingPS.h" --stringname="instancingFragmentShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../btgui/OpenGLWindow/Shaders/pointSpriteVS.glsl" --headerfile="../btgui/OpenGLWindow/Shaders/pointSpriteVS.h" --stringname="pointSpriteVertexShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../btgui/OpenGLWindow/Shaders/pointSpritePS.glsl" --headerfile="../btgui/OpenGLWindow/Shaders/pointSpritePS.h" --stringname="pointSpriteFragmentShader" stringify


premake4 --file=stringifyKernel.lua --kernelfile="../btgui/OpenGLWindow/Shaders/createShadowMapInstancingPS.glsl" --headerfile="../btgui/OpenGLWindow/Shaders/createShadowMapInstancingPS.h" --stringname="createShadowMapInstancingFragmentShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../btgui/OpenGLWindow/Shaders/createShadowMapInstancingVS.glsl" --headerfile="../btgui/OpenGLWindow/Shaders/createShadowMapInstancingVS.h" --stringname="createShadowMapInstancingVertexShader" stringify

premake4 --file=stringifyKernel.lua --kernelfile="../btgui/OpenGLWindow/Shaders/useShadowMapInstancingPS.glsl" --headerfile="../btgui/OpenGLWindow/Shaders/useShadowMapInstancingPS.h" --stringname="useShadowMapInstancingFragmentShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../btgui/OpenGLWindow/Shaders/useShadowMapInstancingVS.glsl" --headerfile="../btgui/OpenGLWindow/Shaders/useShadowMapInstancingVS.h" --stringname="useShadowMapInstancingVertexShader" stringify




premake4 --file=stringifyKernel.lua --kernelfile="../Demos3/GpuDemos/broadphase/pairsKernel.cl" --headerfile="../Demos3/GpuDemos/broadphase/pairsKernel.h" --stringname="pairsKernelsCL" stringify



pause