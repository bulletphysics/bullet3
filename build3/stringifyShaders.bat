
rem @echo off


premake4 --file=stringifyKernel.lua --kernelfile="../examples/OpenGLWindow/Shaders/instancingVS.glsl" --headerfile="../examples/OpenGLWindow/Shaders/instancingVS.h" --stringname="instancingVertexShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../examples/OpenGLWindow/Shaders/instancingPS.glsl" --headerfile="../examples/OpenGLWindow/Shaders/instancingPS.h" --stringname="instancingFragmentShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../examples/OpenGLWindow/Shaders/pointSpriteVS.glsl" --headerfile="../examples/OpenGLWindow/Shaders/pointSpriteVS.h" --stringname="pointSpriteVertexShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../examples/OpenGLWindow/Shaders/pointSpritePS.glsl" --headerfile="../examples/OpenGLWindow/Shaders/pointSpritePS.h" --stringname="pointSpriteFragmentShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../examples/OpenGLWindow/Shaders/createShadowMapInstancingPS.glsl" --headerfile="../examples/OpenGLWindow/Shaders/createShadowMapInstancingPS.h" --stringname="createShadowMapInstancingFragmentShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../examples/OpenGLWindow/Shaders/createShadowMapInstancingVS.glsl" --headerfile="../examples/OpenGLWindow/Shaders/createShadowMapInstancingVS.h" --stringname="createShadowMapInstancingVertexShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../examples/OpenGLWindow/Shaders/useShadowMapInstancingPS.glsl" --headerfile="../examples/OpenGLWindow/Shaders/useShadowMapInstancingPS.h" --stringname="useShadowMapInstancingFragmentShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../examples/OpenGLWindow/Shaders/useShadowMapInstancingVS.glsl" --headerfile="../examples/OpenGLWindow/Shaders/useShadowMapInstancingVS.h" --stringname="useShadowMapInstancingVertexShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../examples/OpenGLWindow/Shaders/projectiveTextureInstancingPS.glsl" --headerfile="../examples/OpenGLWindow/Shaders/projectiveTextureInstancingPS.h" --stringname="projectiveTextureInstancingFragmentShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../examples/OpenGLWindow/Shaders/projectiveTextureInstancingVS.glsl" --headerfile="../examples/OpenGLWindow/Shaders/projectiveTextureInstancingVS.h" --stringname="projectiveTextureInstancingVertexShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../examples/OpenGLWindow/Shaders/linesVS.glsl" --headerfile="../examples/OpenGLWindow/Shaders/linesVS.h" --stringname="linesVertexShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../examples/OpenGLWindow/Shaders/linesPS.glsl" --headerfile="../examples/OpenGLWindow/Shaders/linesPS.h" --stringname="linesFragmentShader" stringify




pause