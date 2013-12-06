//  ---------------------------------------------------------------------------
//
//  @file       LoadOGLCore.cpp
//  @author     Philippe Decaudin - http://www.antisphere.com
//  @license    This file is part of the AntTweakBar library.
//              For conditions of distribution and use, see License.txt
//
//  ---------------------------------------------------------------------------


#include "TwPrecomp.h"
#include "LoadOGLCore.h"

//  ---------------------------------------------------------------------------

#define ANT_NB_OGL_CORE_FUNC_MAX 512

struct COGLCoreFuncRec
{
    const char *        m_Name;
    GLCore::PFNOpenGL * m_FuncPtr;
    COGLCoreFuncRec() : m_Name(NULL), m_FuncPtr(NULL) {}
};
COGLCoreFuncRec g_OGLCoreFuncRec[ANT_NB_OGL_CORE_FUNC_MAX];
int g_NbOGLCoreFunc = 0;
#if defined(ANT_WINDOWS)
HMODULE g_OGLCoreModule = NULL;
#endif

//  ---------------------------------------------------------------------------

// GL 1.0
ANT_GL_CORE_IMPL(glCullFace)
ANT_GL_CORE_IMPL(glFrontFace)
ANT_GL_CORE_IMPL(glHint)
ANT_GL_CORE_IMPL(glLineWidth)
ANT_GL_CORE_IMPL(glPointSize)
ANT_GL_CORE_IMPL(glPolygonMode)
ANT_GL_CORE_IMPL(glScissor)
ANT_GL_CORE_IMPL(glTexParameterf)
ANT_GL_CORE_IMPL(glTexParameterfv)
ANT_GL_CORE_IMPL(glTexParameteri)
ANT_GL_CORE_IMPL(glTexParameteriv)
ANT_GL_CORE_IMPL(glTexImage1D)
ANT_GL_CORE_IMPL(glTexImage2D)
ANT_GL_CORE_IMPL(glDrawBuffer)
ANT_GL_CORE_IMPL(glClear)
ANT_GL_CORE_IMPL(glClearColor)
ANT_GL_CORE_IMPL(glClearStencil)
ANT_GL_CORE_IMPL(glClearDepth)
ANT_GL_CORE_IMPL(glStencilMask)
ANT_GL_CORE_IMPL(glColorMask)
ANT_GL_CORE_IMPL(glDepthMask)
ANT_GL_CORE_IMPL(glDisable)
ANT_GL_CORE_IMPL(glEnable)
ANT_GL_CORE_IMPL(glFinish)
ANT_GL_CORE_IMPL(glFlush)
ANT_GL_CORE_IMPL(glBlendFunc)
ANT_GL_CORE_IMPL(glLogicOp)
ANT_GL_CORE_IMPL(glStencilFunc)
ANT_GL_CORE_IMPL(glStencilOp)
ANT_GL_CORE_IMPL(glDepthFunc)
ANT_GL_CORE_IMPL(glPixelStoref)
ANT_GL_CORE_IMPL(glPixelStorei)
ANT_GL_CORE_IMPL(glReadBuffer)
ANT_GL_CORE_IMPL(glReadPixels)
ANT_GL_CORE_IMPL(glGetBooleanv)
ANT_GL_CORE_IMPL(glGetDoublev)
ANT_GL_CORE_IMPL(glGetError)
ANT_GL_CORE_IMPL(glGetFloatv)
ANT_GL_CORE_IMPL(glGetIntegerv)
ANT_GL_CORE_IMPL(glGetString)
ANT_GL_CORE_IMPL(glGetTexImage)
ANT_GL_CORE_IMPL(glGetTexParameterfv)
ANT_GL_CORE_IMPL(glGetTexParameteriv)
ANT_GL_CORE_IMPL(glGetTexLevelParameterfv)
ANT_GL_CORE_IMPL(glGetTexLevelParameteriv)
ANT_GL_CORE_IMPL(glIsEnabled)
ANT_GL_CORE_IMPL(glDepthRange)
ANT_GL_CORE_IMPL(glViewport)
// GL 1.1
ANT_GL_CORE_IMPL(glDrawArrays)
ANT_GL_CORE_IMPL(glDrawElements)
ANT_GL_CORE_IMPL(glGetPointerv)
ANT_GL_CORE_IMPL(glPolygonOffset)
ANT_GL_CORE_IMPL(glCopyTexImage1D)
ANT_GL_CORE_IMPL(glCopyTexImage2D)
ANT_GL_CORE_IMPL(glCopyTexSubImage1D)
ANT_GL_CORE_IMPL(glCopyTexSubImage2D)
ANT_GL_CORE_IMPL(glTexSubImage1D)
ANT_GL_CORE_IMPL(glTexSubImage2D)
ANT_GL_CORE_IMPL(glBindTexture)
ANT_GL_CORE_IMPL(glDeleteTextures)
ANT_GL_CORE_IMPL(glGenTextures)
ANT_GL_CORE_IMPL(glIsTexture)
// GL 1.2
ANT_GL_CORE_IMPL(glBlendColor)
ANT_GL_CORE_IMPL(glBlendEquation)
ANT_GL_CORE_IMPL(glDrawRangeElements)
ANT_GL_CORE_IMPL(glTexImage3D)
ANT_GL_CORE_IMPL(glTexSubImage3D)
ANT_GL_CORE_IMPL(glCopyTexSubImage3D)
// GL 1.3
ANT_GL_CORE_IMPL(glActiveTexture)
ANT_GL_CORE_IMPL(glSampleCoverage)
ANT_GL_CORE_IMPL(glCompressedTexImage3D)
ANT_GL_CORE_IMPL(glCompressedTexImage2D)
ANT_GL_CORE_IMPL(glCompressedTexImage1D)
ANT_GL_CORE_IMPL(glCompressedTexSubImage3D)
ANT_GL_CORE_IMPL(glCompressedTexSubImage2D)
ANT_GL_CORE_IMPL(glCompressedTexSubImage1D)
ANT_GL_CORE_IMPL(glGetCompressedTexImage)
// GL 1.4
ANT_GL_CORE_IMPL(glBlendFuncSeparate)
ANT_GL_CORE_IMPL(glMultiDrawArrays)
ANT_GL_CORE_IMPL(glMultiDrawElements)
ANT_GL_CORE_IMPL(glPointParameterf)
ANT_GL_CORE_IMPL(glPointParameterfv)
ANT_GL_CORE_IMPL(glPointParameteri)
ANT_GL_CORE_IMPL(glPointParameteriv)
// GL 1.5
ANT_GL_CORE_IMPL(glGenQueries)
ANT_GL_CORE_IMPL(glDeleteQueries)
ANT_GL_CORE_IMPL(glIsQuery)
ANT_GL_CORE_IMPL(glBeginQuery)
ANT_GL_CORE_IMPL(glEndQuery)
ANT_GL_CORE_IMPL(glGetQueryiv)
ANT_GL_CORE_IMPL(glGetQueryObjectiv)
ANT_GL_CORE_IMPL(glGetQueryObjectuiv)
ANT_GL_CORE_IMPL(glBindBuffer)
ANT_GL_CORE_IMPL(glDeleteBuffers)
ANT_GL_CORE_IMPL(glGenBuffers)
ANT_GL_CORE_IMPL(glIsBuffer)
ANT_GL_CORE_IMPL(glBufferData)
ANT_GL_CORE_IMPL(glBufferSubData)
ANT_GL_CORE_IMPL(glGetBufferSubData)
ANT_GL_CORE_IMPL(glMapBuffer)
ANT_GL_CORE_IMPL(glUnmapBuffer)
ANT_GL_CORE_IMPL(glGetBufferParameteriv)
ANT_GL_CORE_IMPL(glGetBufferPointerv)
// GL 2.0
ANT_GL_CORE_IMPL(glBlendEquationSeparate)
ANT_GL_CORE_IMPL(glDrawBuffers)
ANT_GL_CORE_IMPL(glStencilOpSeparate)
ANT_GL_CORE_IMPL(glStencilFuncSeparate)
ANT_GL_CORE_IMPL(glStencilMaskSeparate)
ANT_GL_CORE_IMPL(glAttachShader)
ANT_GL_CORE_IMPL(glBindAttribLocation)
ANT_GL_CORE_IMPL(glCompileShader)
ANT_GL_CORE_IMPL(glCreateProgram)
ANT_GL_CORE_IMPL(glCreateShader)
ANT_GL_CORE_IMPL(glDeleteProgram)
ANT_GL_CORE_IMPL(glDeleteShader)
ANT_GL_CORE_IMPL(glDetachShader)
ANT_GL_CORE_IMPL(glDisableVertexAttribArray)
ANT_GL_CORE_IMPL(glEnableVertexAttribArray)
ANT_GL_CORE_IMPL(glGetActiveAttrib)
ANT_GL_CORE_IMPL(glGetActiveUniform)
ANT_GL_CORE_IMPL(glGetAttachedShaders)
ANT_GL_CORE_IMPL(glGetAttribLocation)
ANT_GL_CORE_IMPL(glGetProgramiv)
ANT_GL_CORE_IMPL(glGetProgramInfoLog)
ANT_GL_CORE_IMPL(glGetShaderiv)
ANT_GL_CORE_IMPL(glGetShaderInfoLog)
ANT_GL_CORE_IMPL(glGetShaderSource)
ANT_GL_CORE_IMPL(glGetUniformLocation)
ANT_GL_CORE_IMPL(glGetUniformfv)
ANT_GL_CORE_IMPL(glGetUniformiv)
ANT_GL_CORE_IMPL(glGetVertexAttribdv)
ANT_GL_CORE_IMPL(glGetVertexAttribfv)
ANT_GL_CORE_IMPL(glGetVertexAttribiv)
ANT_GL_CORE_IMPL(glGetVertexAttribPointerv)
ANT_GL_CORE_IMPL(glIsProgram)
ANT_GL_CORE_IMPL(glIsShader)
ANT_GL_CORE_IMPL(glLinkProgram)
ANT_GL_CORE_IMPL(glShaderSource)
ANT_GL_CORE_IMPL(glUseProgram)
ANT_GL_CORE_IMPL(glUniform1f)
ANT_GL_CORE_IMPL(glUniform2f)
ANT_GL_CORE_IMPL(glUniform3f)
ANT_GL_CORE_IMPL(glUniform4f)
ANT_GL_CORE_IMPL(glUniform1i)
ANT_GL_CORE_IMPL(glUniform2i)
ANT_GL_CORE_IMPL(glUniform3i)
ANT_GL_CORE_IMPL(glUniform4i)
ANT_GL_CORE_IMPL(glUniform1fv)
ANT_GL_CORE_IMPL(glUniform2fv)
ANT_GL_CORE_IMPL(glUniform3fv)
ANT_GL_CORE_IMPL(glUniform4fv)
ANT_GL_CORE_IMPL(glUniform1iv)
ANT_GL_CORE_IMPL(glUniform2iv)
ANT_GL_CORE_IMPL(glUniform3iv)
ANT_GL_CORE_IMPL(glUniform4iv)
ANT_GL_CORE_IMPL(glUniformMatrix2fv)
ANT_GL_CORE_IMPL(glUniformMatrix3fv)
ANT_GL_CORE_IMPL(glUniformMatrix4fv)
ANT_GL_CORE_IMPL(glValidateProgram)
ANT_GL_CORE_IMPL(glVertexAttrib1d)
ANT_GL_CORE_IMPL(glVertexAttrib1dv)
ANT_GL_CORE_IMPL(glVertexAttrib1f)
ANT_GL_CORE_IMPL(glVertexAttrib1fv)
ANT_GL_CORE_IMPL(glVertexAttrib1s)
ANT_GL_CORE_IMPL(glVertexAttrib1sv)
ANT_GL_CORE_IMPL(glVertexAttrib2d)
ANT_GL_CORE_IMPL(glVertexAttrib2dv)
ANT_GL_CORE_IMPL(glVertexAttrib2f)
ANT_GL_CORE_IMPL(glVertexAttrib2fv)
ANT_GL_CORE_IMPL(glVertexAttrib2s)
ANT_GL_CORE_IMPL(glVertexAttrib2sv)
ANT_GL_CORE_IMPL(glVertexAttrib3d)
ANT_GL_CORE_IMPL(glVertexAttrib3dv)
ANT_GL_CORE_IMPL(glVertexAttrib3f)
ANT_GL_CORE_IMPL(glVertexAttrib3fv)
ANT_GL_CORE_IMPL(glVertexAttrib3s)
ANT_GL_CORE_IMPL(glVertexAttrib3sv)
ANT_GL_CORE_IMPL(glVertexAttrib4Nbv)
ANT_GL_CORE_IMPL(glVertexAttrib4Niv)
ANT_GL_CORE_IMPL(glVertexAttrib4Nsv)
ANT_GL_CORE_IMPL(glVertexAttrib4Nub)
ANT_GL_CORE_IMPL(glVertexAttrib4Nubv)
ANT_GL_CORE_IMPL(glVertexAttrib4Nuiv)
ANT_GL_CORE_IMPL(glVertexAttrib4Nusv)
ANT_GL_CORE_IMPL(glVertexAttrib4bv)
ANT_GL_CORE_IMPL(glVertexAttrib4d)
ANT_GL_CORE_IMPL(glVertexAttrib4dv)
ANT_GL_CORE_IMPL(glVertexAttrib4f)
ANT_GL_CORE_IMPL(glVertexAttrib4fv)
ANT_GL_CORE_IMPL(glVertexAttrib4iv)
ANT_GL_CORE_IMPL(glVertexAttrib4s)
ANT_GL_CORE_IMPL(glVertexAttrib4sv)
ANT_GL_CORE_IMPL(glVertexAttrib4ubv)
ANT_GL_CORE_IMPL(glVertexAttrib4uiv)
ANT_GL_CORE_IMPL(glVertexAttrib4usv)
ANT_GL_CORE_IMPL(glVertexAttribPointer)
/*
// GL 2.1
ANT_GL_CORE_IMPL(glUniformMatrix2x3fv)
ANT_GL_CORE_IMPL(glUniformMatrix3x2fv)
ANT_GL_CORE_IMPL(glUniformMatrix2x4fv)
ANT_GL_CORE_IMPL(glUniformMatrix4x2fv)
ANT_GL_CORE_IMPL(glUniformMatrix3x4fv)
ANT_GL_CORE_IMPL(glUniformMatrix4x3fv)
// GL 3.0
ANT_GL_CORE_IMPL(glColorMaski)
ANT_GL_CORE_IMPL(glGetBooleani_v)
ANT_GL_CORE_IMPL(glGetIntegeri_v)
ANT_GL_CORE_IMPL(glEnablei)
ANT_GL_CORE_IMPL(glDisablei)
ANT_GL_CORE_IMPL(glIsEnabledi)
ANT_GL_CORE_IMPL(glBeginTransformFeedback)
ANT_GL_CORE_IMPL(glEndTransformFeedback)
ANT_GL_CORE_IMPL(glBindBufferRange)
ANT_GL_CORE_IMPL(glBindBufferBase)
ANT_GL_CORE_IMPL(glTransformFeedbackVaryings)
ANT_GL_CORE_IMPL(glGetTransformFeedbackVarying)
ANT_GL_CORE_IMPL(glClampColor)
ANT_GL_CORE_IMPL(glBeginConditionalRender)
ANT_GL_CORE_IMPL(glEndConditionalRender)
ANT_GL_CORE_IMPL(glVertexAttribIPointer)
ANT_GL_CORE_IMPL(glGetVertexAttribIiv)
ANT_GL_CORE_IMPL(glGetVertexAttribIuiv)
ANT_GL_CORE_IMPL(glVertexAttribI1i)
ANT_GL_CORE_IMPL(glVertexAttribI2i)
ANT_GL_CORE_IMPL(glVertexAttribI3i)
ANT_GL_CORE_IMPL(glVertexAttribI4i)
ANT_GL_CORE_IMPL(glVertexAttribI1ui)
ANT_GL_CORE_IMPL(glVertexAttribI2ui)
ANT_GL_CORE_IMPL(glVertexAttribI3ui)
ANT_GL_CORE_IMPL(glVertexAttribI4ui)
ANT_GL_CORE_IMPL(glVertexAttribI1iv)
ANT_GL_CORE_IMPL(glVertexAttribI2iv)
ANT_GL_CORE_IMPL(glVertexAttribI3iv)
ANT_GL_CORE_IMPL(glVertexAttribI4iv)
ANT_GL_CORE_IMPL(glVertexAttribI1uiv)
ANT_GL_CORE_IMPL(glVertexAttribI2uiv)
ANT_GL_CORE_IMPL(glVertexAttribI3uiv)
ANT_GL_CORE_IMPL(glVertexAttribI4uiv)
ANT_GL_CORE_IMPL(glVertexAttribI4bv)
ANT_GL_CORE_IMPL(glVertexAttribI4sv)
ANT_GL_CORE_IMPL(glVertexAttribI4ubv)
ANT_GL_CORE_IMPL(glVertexAttribI4usv)
ANT_GL_CORE_IMPL(glGetUniformuiv)
ANT_GL_CORE_IMPL(glBindFragDataLocation)
ANT_GL_CORE_IMPL(glGetFragDataLocation)
ANT_GL_CORE_IMPL(glUniform1ui)
ANT_GL_CORE_IMPL(glUniform2ui)
ANT_GL_CORE_IMPL(glUniform3ui)
ANT_GL_CORE_IMPL(glUniform4ui)
ANT_GL_CORE_IMPL(glUniform1uiv)
ANT_GL_CORE_IMPL(glUniform2uiv)
ANT_GL_CORE_IMPL(glUniform3uiv)
ANT_GL_CORE_IMPL(glUniform4uiv)
ANT_GL_CORE_IMPL(glTexParameterIiv)
ANT_GL_CORE_IMPL(glTexParameterIuiv)
ANT_GL_CORE_IMPL(glGetTexParameterIiv)
ANT_GL_CORE_IMPL(glGetTexParameterIuiv)
ANT_GL_CORE_IMPL(glClearBufferiv)
ANT_GL_CORE_IMPL(glClearBufferuiv)
ANT_GL_CORE_IMPL(glClearBufferfv)
ANT_GL_CORE_IMPL(glClearBufferfi)
ANT_GL_CORE_IMPL(glGetStringi)
// GL 3.1
ANT_GL_CORE_IMPL(glDrawArraysInstanced)
ANT_GL_CORE_IMPL(glDrawElementsInstanced)
ANT_GL_CORE_IMPL(glTexBuffer)
ANT_GL_CORE_IMPL(glPrimitiveRestartIndex)
// GL 3.2
//ANT_GL_CORE_IMPL(glGetInteger64i_v)
//ANT_GL_CORE_IMPL(glGetBufferParameteri64v)
ANT_GL_CORE_IMPL(glFramebufferTexture)
*/

// GL_ARB_vertex_array_object
#if defined(ANT_WINDOWS)
    ANT_GL_CORE_IMPL(glBindVertexArray)
    ANT_GL_CORE_IMPL(glDeleteVertexArrays)
    ANT_GL_CORE_IMPL(glGenVertexArrays)
    ANT_GL_CORE_IMPL(glIsVertexArray)
#else
    // these extensions are loaded explicitely by LoadOpenGLCore
    // because they may not be avialable on non-OpenGL 3.2 environments
    namespace GLCore 
    { 
        PFNglBindVertexArray _glBindVertexArray = NULL; 
        PFNglDeleteVertexArrays _glDeleteVertexArrays = NULL; 
        PFNglGenVertexArrays _glGenVertexArrays = NULL; 
        PFNglIsVertexArray _glIsVertexArray = NULL; 
    }
#endif

#if defined(ANT_WINDOWS)
    ANT_GL_CORE_IMPL(wglGetProcAddress)
#endif

namespace GLCore { PFNGLGetProcAddress _glGetProcAddress = NULL; }

//  ---------------------------------------------------------------------------

#if defined(ANT_WINDOWS)

    //  ---------------------------------------------------------------------------
    
    int LoadOpenGLCore()
    {
        if( g_OGLCoreModule!=NULL )
        {
            return 1; // "OpenGL library already loaded"
        }
    
        g_OGLCoreModule = LoadLibrary("OPENGL32.DLL");
        if( g_OGLCoreModule )
        {
            // Info(VERB_LOW, "Load %d OpenGL Core functions", g_NbOGLCoreFunc);
    
            int Res = 1;

            // Use wglGetProcAddress to retreive Core functions
            _glGetProcAddress = reinterpret_cast<GLCore::PFNGLGetProcAddress>(GetProcAddress(g_OGLCoreModule, "wglGetProcAddress"));
            if( _glGetProcAddress!=NULL )
                for(int i=0; i<g_NbOGLCoreFunc; ++i)
                {
                    assert(g_OGLCoreFuncRec[i].m_FuncPtr!=NULL);
                    assert(*(g_OGLCoreFuncRec[i].m_FuncPtr)==NULL);
                    assert(g_OGLCoreFuncRec[i].m_Name!=NULL);
                    assert(strlen(g_OGLCoreFuncRec[i].m_Name)>0);
                    // Try to get the function pointer with wglGetProcAddress
                    *(g_OGLCoreFuncRec[i].m_FuncPtr) = reinterpret_cast<GLCore::PFNOpenGL>(_glGetProcAddress(g_OGLCoreFuncRec[i].m_Name));
                    if( *(g_OGLCoreFuncRec[i].m_FuncPtr)==NULL ) 
                    {
                        // Try to get the function pointer with GetProcAddress
                        *(g_OGLCoreFuncRec[i].m_FuncPtr) = reinterpret_cast<GLCore::PFNOpenGL>(GetProcAddress(g_OGLCoreModule, g_OGLCoreFuncRec[i].m_Name));
                        if( *(g_OGLCoreFuncRec[i].m_FuncPtr)==NULL )
                        {
                        #ifdef _DEBUG
                            fprintf(stderr, "AntTweakBar: Cannot load function %s\n", g_OGLCoreFuncRec[i].m_Name);
                        #endif
                            Res = 0; // Error("cannot find OpenGL Core function");
                        }
                    }
        
                }

            return Res;
        }
        else
        {
            // InternDisplayLastErrorWIN("Cannot load opengl32 DLL", false);
            return 0;   // cannot load DLL
        }
    }
    
    //  ---------------------------------------------------------------------------
    
    int UnloadOpenGLCore()
    {
        if( g_OGLCoreModule==NULL )
        {
            return 1; // "OpenGL library not loaded"
        }
    
        // Info(VERB_LOW, "Unload %d OpenGL Core functions", g_NbOGLCoreFunc);
        for(int i=0; i<g_NbOGLCoreFunc; ++i)
        {
            assert(g_OGLCoreFuncRec[i].m_FuncPtr!=NULL);
            assert(*(g_OGLCoreFuncRec[i].m_FuncPtr)!=NULL);
            assert(g_OGLCoreFuncRec[i].m_Name!=NULL);
            assert(strlen(g_OGLCoreFuncRec[i].m_Name)>0);
            *(g_OGLCoreFuncRec[i].m_FuncPtr) = NULL;
        }
        if( FreeLibrary(g_OGLCoreModule) )
        {
            // Info(VERB_LOW, "OpenGL library unloaded");
            g_OGLCoreModule = NULL;
            return 1;
        }
        else
        {
            // InternDisplayLastErrorWIN("Cannot unload opengl32 DLL", false);
            return 0; // cannot unload opengl32.dll
        }
    }
    
    //  ---------------------------------------------------------------------------
    
    namespace GLCore
    {
    
        PFNOpenGL Record(const char *_FuncName, PFNOpenGL *_FuncPtr)
        {
            if( g_NbOGLCoreFunc>=ANT_NB_OGL_CORE_FUNC_MAX )
            {
                fprintf(stderr, "Too many OpenGL Core functions declared. Change ANT_NB_OGL_CORE_FUNC_MAX.");
                exit(-1);
            }
    
            g_OGLCoreFuncRec[g_NbOGLCoreFunc].m_Name = _FuncName;
            g_OGLCoreFuncRec[g_NbOGLCoreFunc].m_FuncPtr = _FuncPtr;
            ++g_NbOGLCoreFunc;
    
            return NULL;
        }
    
    } // namespace GL
    
    //  ---------------------------------------------------------------------------

#endif // defined(ANT_WINDOWS)

//  ---------------------------------------------------------------------------

#if defined(ANT_UNIX)
    
    int LoadOpenGLCore()
    {
        _glGetProcAddress = reinterpret_cast<GLCore::PFNGLGetProcAddress>(glXGetProcAddressARB);

        _glBindVertexArray = reinterpret_cast<PFNglBindVertexArray>(_glGetProcAddress("glBindVertexArray"));
        _glDeleteVertexArrays = reinterpret_cast<PFNglDeleteVertexArrays>(_glGetProcAddress("glDeleteVertexArrays"));
        _glGenVertexArrays = reinterpret_cast<PFNglGenVertexArrays>(_glGetProcAddress("glGenVertexArrays"));
        _glIsVertexArray = reinterpret_cast<PFNglIsVertexArray>(_glGetProcAddress("glIsVertexArray"));

        if( _glBindVertexArray==NULL || _glDeleteVertexArrays==NULL || _glGenVertexArrays==NULL || _glIsVertexArray==NULL )
        {
            fprintf(stderr, "AntTweakBar: OpenGL Core Profile functions cannot be loaded.\n");
            return 0;
        }
        else
            return 1;
    }
    
    int UnloadOpenGLCore()
    {
        return 1;
    }
    
#elif defined(ANT_OSX)

    #include <dlfcn.h>

    static void *gl_dyld = NULL;
    void *NSGLCoreGetProcAddressNew(const GLubyte *name) 
    {
        void *proc=NULL;
        if (gl_dyld == NULL) 
        {
            gl_dyld = dlopen("OpenGL",RTLD_LAZY);
        }
        if (gl_dyld) 
        {
            NSString *sym = [[NSString alloc] initWithFormat: @"_%s",name];
            proc = dlsym(gl_dyld,[sym UTF8String]);
            [sym release];
        }
        return proc;
    }

    int LoadOpenGLCore() 
    {
        _glGetProcAddress = reinterpret_cast<GLCore::PFNGLGetProcAddress>(NSGLCoreGetProcAddressNew);

        _glBindVertexArray = reinterpret_cast<PFNglBindVertexArray>(_glGetProcAddress("glBindVertexArray"));
        _glDeleteVertexArrays = reinterpret_cast<PFNglDeleteVertexArrays>(_glGetProcAddress("glDeleteVertexArrays"));
        _glGenVertexArrays = reinterpret_cast<PFNglGenVertexArrays>(_glGetProcAddress("glGenVertexArrays"));
        _glIsVertexArray = reinterpret_cast<PFNglIsVertexArray>(_glGetProcAddress("glIsVertexArray"));

        if( _glBindVertexArray==NULL || _glDeleteVertexArrays==NULL || _glGenVertexArrays==NULL || _glIsVertexArray==NULL )
        {
            fprintf(stderr, "AntTweakBar: OpenGL Core Profile functions cannot be loaded.\n");
            return 0;
        }
        else
            return 1;
    }

    int UnloadOpenGLCore() 
    {
       if (gl_dyld) 
       {
           dlclose(gl_dyld);
           gl_dyld = NULL;
       }
       return 1;
   }    
   
#endif // defined(ANT_UNIX)

//  ---------------------------------------------------------------------------
