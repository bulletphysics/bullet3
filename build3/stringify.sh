#!/bin/sh
unamestr=`uname`

if [ $unamestr = 'Linux' ]; then
        echo "Using Linux"
        mypremake="./premake4_linux64"
else
        echo "Assuming Mac OSX"
        mypremake="./premake4_osx"
fi

#rem @echo off


eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/RadixSort32Kernels.cl" 	--headerfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/RadixSort32KernelsCL.h" --stringname="radixSort32KernelsCL" stringify'
eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/BoundSearchKernels.cl" 	--headerfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/BoundSearchKernelsCL.h" --stringname="boundSearchKernelsCL" stringify'
eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/PrefixScanKernels.cl" 	--headerfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/PrefixScanKernelsCL.h" --stringname="prefixScanKernelsCL" stringify'
eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/PrefixScanFloat4Kernels.cl" 	--headerfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/PrefixScanKernelsFloat4CL.h" --stringname="prefixScanKernelsFloat4CL" stringify'

eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/FillKernels.cl" 				--headerfile="../src/Bullet3OpenCL/ParallelPrimitives/kernels/FillKernelsCL.h" --stringname="fillKernelsCL" stringify'

eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/BroadphaseCollision/kernels/sap.cl" --headerfile="../src/Bullet3OpenCL/BroadphaseCollision/kernels/sapKernels.h" --stringname="sapCL" stringify'

eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/BroadphaseCollision/kernels/gridBroadphase.cl" --headerfile="../src/Bullet3OpenCL/BroadphaseCollision/kernels/gridBroadphaseKernels.h" --stringname="gridBroadphaseCL" stringify'
eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/BroadphaseCollision/kernels/parallelLinearBvh.cl" --headerfile="../src/Bullet3OpenCL/BroadphaseCollision/kernels/parallelLinearBvhKernels.h" --stringname="parallelLinearBvhCL" stringify'

eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/sat.cl" --headerfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/satKernels.h" --stringname="satKernelsCL" stringify'
eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/satConcave.cl" --headerfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/satConcaveKernels.h" --stringname="satConcaveKernelsCL" stringify'
eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/mpr.cl" --headerfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/mprKernels.h" --stringname="mprKernelsCL" stringify'



eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/satClipHullContacts.cl" --headerfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/satClipHullContacts.h" --stringname="satClipKernelsCL" stringify'
eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/primitiveContacts.cl" --headerfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/primitiveContacts.h" --stringname="primitiveContactsKernelsCL" stringify'
eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/bvhTraversal.cl" --headerfile="../src/Bullet3OpenCL/NarrowphaseCollision/kernels/bvhTraversal.h" --stringname="bvhTraversalKernelCL" stringify'


eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody/kernels/integrateKernel.cl" --headerfile="../src/Bullet3OpenCL/RigidBody/kernels/integrateKernel.h" --stringname="integrateKernelCL" stringify'
eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody/kernels/updateAabbsKernel.cl" --headerfile="../src/Bullet3OpenCL/RigidBody/kernels/updateAabbsKernel.h" --stringname="updateAabbsKernelCL" stringify'
eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody/kernels/solverSetup.cl" --headerfile="../src/Bullet3OpenCL/RigidBody/kernels/solverSetup.h" --stringname="solverSetupCL" stringify'
eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody/kernels/solverSetup2.cl" --headerfile="../src/Bullet3OpenCL/RigidBody/kernels/solverSetup2.h" --stringname="solverSetup2CL" stringify'
eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody/kernels/batchingKernels.cl" --headerfile="../src/Bullet3OpenCL/RigidBody/kernels/batchingKernels.h" --stringname="batchingKernelsCL" stringify'
eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody/kernels/batchingKernelsNew.cl" --headerfile="../src/Bullet3OpenCL/RigidBody/kernels/batchingKernelsNew.h" --stringname="batchingKernelsNewCL" stringify'
eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody/kernels/solverUtils.cl" --headerfile="../src/Bullet3OpenCL/RigidBody/kernels/solverUtils.h" --stringname="solverUtilsCL" stringify'
eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody/kernels/solveContact.cl" --headerfile="../src/Bullet3OpenCL/RigidBody/kernels/solveContact.h" --stringname="solveContactCL" stringify'
eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody/kernels/solveFriction.cl" --headerfile="../src/Bullet3OpenCL/RigidBody/kernels/solveFriction.h" --stringname="solveFrictionCL" stringify'

eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/RigidBody/kernels/jointSolver.cl" --headerfile="../src/Bullet3OpenCL/RigidBody/kernels/jointSolver.h" --stringname="solveConstraintRowsCL" stringify'



eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../src/Bullet3OpenCL/Raycast/kernels/rayCastKernels.cl" --headerfile="../src/Bullet3OpenCL/Raycast/kernels/rayCastKernels.h" --stringname="rayCastKernelCL" stringify'

eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../examples/OpenGLWindow/Shaders/instancingVS.glsl" --headerfile="../examples/OpenGLWindow/Shaders/instancingVS.h" --stringname="instancingVertexShader" stringify'
eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../examples/OpenGLWindow/Shaders/instancingPS.glsl" --headerfile="../examples/OpenGLWindow/Shaders/instancingPS.h" --stringname="instancingFragmentShader" stringify'
eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../examples/OpenGLWindow/Shaders/pointSpriteVS.glsl" --headerfile="../examples/OpenGLWindow/Shaders/pointSpriteVS.h" --stringname="pointSpriteVertexShader" stringify'
eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../examples/OpenGLWindow/Shaders/pointSpritePS.glsl" --headerfile="../examples/OpenGLWindow/Shaders/pointSpritePS.h" --stringname="pointSpriteFragmentShader" stringify'


eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../examples/OpenGLWindow/Shaders/createShadowMapInstancingPS.glsl" --headerfile="../examples/OpenGLWindow/Shaders/createShadowMapInstancingPS.h" --stringname="createShadowMapInstancingFragmentShader" stringify'
eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../examples/OpenGLWindow/Shaders/createShadowMapInstancingVS.glsl" --headerfile="../examples/OpenGLWindow/Shaders/createShadowMapInstancingVS.h" --stringname="createShadowMapInstancingVertexShader" stringify'

eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../examples/OpenGLWindow/Shaders/useShadowMapInstancingPS.glsl" --headerfile="../examples/OpenGLWindow/Shaders/useShadowMapInstancingPS.h" --stringname="useShadowMapInstancingFragmentShader" stringify'
eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../examples/OpenGLWindow/Shaders/useShadowMapInstancingVS.glsl" --headerfile="../examples/OpenGLWindow/Shaders/useShadowMapInstancingVS.h" --stringname="useShadowMapInstancingVertexShader" stringify'
eval '$mypremake  --file=stringifyKernel.lua --kernelfile="../examples/OpenCL/broadphase/pairsKernel.cl" --headerfile="../examples/OpenCL/broadphase/pairsKernel.h" --stringname="pairsKernelsCL" stringify'



