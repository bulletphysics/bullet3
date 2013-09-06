
rem @echo off


premake4 --file=stringifyKernel.lua --kernelfile="../btgui/OpenGLWindow/Shaders/instancingVS.glsl" --headerfile="../btgui/OpenGLWindow/Shaders/instancingVS.h" --stringname="instancingVertexShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../btgui/OpenGLWindow/Shaders/instancingPS.glsl" --headerfile="../btgui/OpenGLWindow/Shaders/instancingPS.h" --stringname="instancingFragmentShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../btgui/OpenGLWindow/Shaders/pointSpriteVS.glsl" --headerfile="../btgui/OpenGLWindow/Shaders/pointSpriteVS.h" --stringname="pointSpriteVertexShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../btgui/OpenGLWindow/Shaders/pointSpritePS.glsl" --headerfile="../btgui/OpenGLWindow/Shaders/pointSpritePS.h" --stringname="pointSpriteFragmentShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../btgui/OpenGLWindow/Shaders/createShadowMapInstancingPS.glsl" --headerfile="../btgui/OpenGLWindow/Shaders/createShadowMapInstancingPS.h" --stringname="createShadowMapInstancingFragmentShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../btgui/OpenGLWindow/Shaders/createShadowMapInstancingVS.glsl" --headerfile="../btgui/OpenGLWindow/Shaders/createShadowMapInstancingVS.h" --stringname="createShadowMapInstancingVertexShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../btgui/OpenGLWindow/Shaders/useShadowMapInstancingPS.glsl" --headerfile="../btgui/OpenGLWindow/Shaders/useShadowMapInstancingPS.h" --stringname="useShadowMapInstancingFragmentShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../btgui/OpenGLWindow/Shaders/useShadowMapInstancingVS.glsl" --headerfile="../btgui/OpenGLWindow/Shaders/useShadowMapInstancingVS.h" --stringname="useShadowMapInstancingVertexShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../btgui/OpenGLWindow/Shaders/linesVS.glsl" --headerfile="../btgui/OpenGLWindow/Shaders/linesVS.h" --stringname="linesVertexShader" stringify
premake4 --file=stringifyKernel.lua --kernelfile="../btgui/OpenGLWindow/Shaders/linesPS.glsl" --headerfile="../btgui/OpenGLWindow/Shaders/linesPS.h" --stringname="linesFragmentShader" stringify




pause