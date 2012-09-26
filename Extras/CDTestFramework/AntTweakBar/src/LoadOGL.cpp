//  ---------------------------------------------------------------------------
//
//  @file       LoadOGL.cpp
//  @author     Philippe Decaudin - http://www.antisphere.com
//  @license    This file is part of the AntTweakBar library.
//              For conditions of distribution and use, see License.txt
//
//  ---------------------------------------------------------------------------


#include "TwPrecomp.h"
#include "LoadOGL.h"


//  ---------------------------------------------------------------------------

#define ANT_NB_OGL_FUNC_MAX 1024

struct COGLFuncRec
{
    const char *    m_Name;
    GL::PFNOpenGL * m_FuncPtr;
    COGLFuncRec() : m_Name(NULL), m_FuncPtr(NULL) {}
};
COGLFuncRec g_OGLFuncRec[ANT_NB_OGL_FUNC_MAX];
int g_NbOGLFunc = 0;
#if defined(ANT_WINDOWS)
HMODULE g_OGLModule = NULL;
#endif

//  ---------------------------------------------------------------------------

ANT_GL_IMPL(glAccum)
ANT_GL_IMPL(glAlphaFunc)
ANT_GL_IMPL(glAreTexturesResident)
ANT_GL_IMPL(glArrayElement)
ANT_GL_IMPL(glBegin)
ANT_GL_IMPL(glBindTexture)
ANT_GL_IMPL(glBitmap)
ANT_GL_IMPL(glBlendFunc)
ANT_GL_IMPL(glCallList)
ANT_GL_IMPL(glCallLists)
ANT_GL_IMPL(glClear)
ANT_GL_IMPL(glClearAccum)
ANT_GL_IMPL(glClearColor)
ANT_GL_IMPL(glClearDepth)
ANT_GL_IMPL(glClearIndex)
ANT_GL_IMPL(glClearStencil)
ANT_GL_IMPL(glClipPlane)
ANT_GL_IMPL(glColor3b)
ANT_GL_IMPL(glColor3bv)
ANT_GL_IMPL(glColor3d)
ANT_GL_IMPL(glColor3dv)
ANT_GL_IMPL(glColor3f)
ANT_GL_IMPL(glColor3fv)
ANT_GL_IMPL(glColor3i)
ANT_GL_IMPL(glColor3iv)
ANT_GL_IMPL(glColor3s)
ANT_GL_IMPL(glColor3sv)
ANT_GL_IMPL(glColor3ub)
ANT_GL_IMPL(glColor3ubv)
ANT_GL_IMPL(glColor3ui)
ANT_GL_IMPL(glColor3uiv)
ANT_GL_IMPL(glColor3us)
ANT_GL_IMPL(glColor3usv)
ANT_GL_IMPL(glColor4b)
ANT_GL_IMPL(glColor4bv)
ANT_GL_IMPL(glColor4d)
ANT_GL_IMPL(glColor4dv)
ANT_GL_IMPL(glColor4f)
ANT_GL_IMPL(glColor4fv)
ANT_GL_IMPL(glColor4i)
ANT_GL_IMPL(glColor4iv)
ANT_GL_IMPL(glColor4s)
ANT_GL_IMPL(glColor4sv)
ANT_GL_IMPL(glColor4ub)
ANT_GL_IMPL(glColor4ubv)
ANT_GL_IMPL(glColor4ui)
ANT_GL_IMPL(glColor4uiv)
ANT_GL_IMPL(glColor4us)
ANT_GL_IMPL(glColor4usv)
ANT_GL_IMPL(glColorMask)
ANT_GL_IMPL(glColorMaterial)
ANT_GL_IMPL(glColorPointer)
ANT_GL_IMPL(glCopyPixels)
ANT_GL_IMPL(glCopyTexImage1D)
ANT_GL_IMPL(glCopyTexImage2D)
ANT_GL_IMPL(glCopyTexSubImage1D)
ANT_GL_IMPL(glCopyTexSubImage2D)
ANT_GL_IMPL(glCullFace)
ANT_GL_IMPL(glDeleteLists)
ANT_GL_IMPL(glDeleteTextures)
ANT_GL_IMPL(glDepthFunc)
ANT_GL_IMPL(glDepthMask)
ANT_GL_IMPL(glDepthRange)
ANT_GL_IMPL(glDisable)
ANT_GL_IMPL(glDisableClientState)
ANT_GL_IMPL(glDrawArrays)
ANT_GL_IMPL(glDrawBuffer)
ANT_GL_IMPL(glDrawElements)
ANT_GL_IMPL(glDrawPixels)
ANT_GL_IMPL(glEdgeFlag)
ANT_GL_IMPL(glEdgeFlagPointer)
ANT_GL_IMPL(glEdgeFlagv)
ANT_GL_IMPL(glEnable)
ANT_GL_IMPL(glEnableClientState)
ANT_GL_IMPL(glEnd)
ANT_GL_IMPL(glEndList)
ANT_GL_IMPL(glEvalCoord1d)
ANT_GL_IMPL(glEvalCoord1dv)
ANT_GL_IMPL(glEvalCoord1f)
ANT_GL_IMPL(glEvalCoord1fv)
ANT_GL_IMPL(glEvalCoord2d)
ANT_GL_IMPL(glEvalCoord2dv)
ANT_GL_IMPL(glEvalCoord2f)
ANT_GL_IMPL(glEvalCoord2fv)
ANT_GL_IMPL(glEvalMesh1)
ANT_GL_IMPL(glEvalMesh2)
ANT_GL_IMPL(glEvalPoint1)
ANT_GL_IMPL(glEvalPoint2)
ANT_GL_IMPL(glFeedbackBuffer)
ANT_GL_IMPL(glFinish)
ANT_GL_IMPL(glFlush)
ANT_GL_IMPL(glFogf)
ANT_GL_IMPL(glFogfv)
ANT_GL_IMPL(glFogi)
ANT_GL_IMPL(glFogiv)
ANT_GL_IMPL(glFrontFace)
ANT_GL_IMPL(glFrustum)
ANT_GL_IMPL(glGenLists)
ANT_GL_IMPL(glGenTextures)
ANT_GL_IMPL(glGetBooleanv)
ANT_GL_IMPL(glGetClipPlane)
ANT_GL_IMPL(glGetDoublev)
ANT_GL_IMPL(glGetError)
ANT_GL_IMPL(glGetFloatv)
ANT_GL_IMPL(glGetIntegerv)
ANT_GL_IMPL(glGetLightfv)
ANT_GL_IMPL(glGetLightiv)
ANT_GL_IMPL(glGetMapdv)
ANT_GL_IMPL(glGetMapfv)
ANT_GL_IMPL(glGetMapiv)
ANT_GL_IMPL(glGetMaterialfv)
ANT_GL_IMPL(glGetMaterialiv)
ANT_GL_IMPL(glGetPixelMapfv)
ANT_GL_IMPL(glGetPixelMapuiv)
ANT_GL_IMPL(glGetPixelMapusv)
ANT_GL_IMPL(glGetPointerv)
ANT_GL_IMPL(glGetPolygonStipple)
ANT_GL_IMPL(glGetString)
ANT_GL_IMPL(glGetTexEnvfv)
ANT_GL_IMPL(glGetTexEnviv)
ANT_GL_IMPL(glGetTexGendv)
ANT_GL_IMPL(glGetTexGenfv)
ANT_GL_IMPL(glGetTexGeniv)
ANT_GL_IMPL(glGetTexImage)
ANT_GL_IMPL(glGetTexLevelParameterfv)
ANT_GL_IMPL(glGetTexLevelParameteriv)
ANT_GL_IMPL(glGetTexParameterfv)
ANT_GL_IMPL(glGetTexParameteriv)
ANT_GL_IMPL(glHint)
ANT_GL_IMPL(glIndexMask)
ANT_GL_IMPL(glIndexPointer)
ANT_GL_IMPL(glIndexd)
ANT_GL_IMPL(glIndexdv)
ANT_GL_IMPL(glIndexf)
ANT_GL_IMPL(glIndexfv)
ANT_GL_IMPL(glIndexi)
ANT_GL_IMPL(glIndexiv)
ANT_GL_IMPL(glIndexs)
ANT_GL_IMPL(glIndexsv)
ANT_GL_IMPL(glIndexub)
ANT_GL_IMPL(glIndexubv)
ANT_GL_IMPL(glInitNames)
ANT_GL_IMPL(glInterleavedArrays)
ANT_GL_IMPL(glIsEnabled)
ANT_GL_IMPL(glIsList)
ANT_GL_IMPL(glIsTexture)
ANT_GL_IMPL(glLightModelf)
ANT_GL_IMPL(glLightModelfv)
ANT_GL_IMPL(glLightModeli)
ANT_GL_IMPL(glLightModeliv)
ANT_GL_IMPL(glLightf)
ANT_GL_IMPL(glLightfv)
ANT_GL_IMPL(glLighti)
ANT_GL_IMPL(glLightiv)
ANT_GL_IMPL(glLineStipple)
ANT_GL_IMPL(glLineWidth)
ANT_GL_IMPL(glListBase)
ANT_GL_IMPL(glLoadIdentity)
ANT_GL_IMPL(glLoadMatrixd)
ANT_GL_IMPL(glLoadMatrixf)
ANT_GL_IMPL(glLoadName)
ANT_GL_IMPL(glLogicOp)
ANT_GL_IMPL(glMap1d)
ANT_GL_IMPL(glMap1f)
ANT_GL_IMPL(glMap2d)
ANT_GL_IMPL(glMap2f)
ANT_GL_IMPL(glMapGrid1d)
ANT_GL_IMPL(glMapGrid1f)
ANT_GL_IMPL(glMapGrid2d)
ANT_GL_IMPL(glMapGrid2f)
ANT_GL_IMPL(glMaterialf)
ANT_GL_IMPL(glMaterialfv)
ANT_GL_IMPL(glMateriali)
ANT_GL_IMPL(glMaterialiv)
ANT_GL_IMPL(glMatrixMode)
ANT_GL_IMPL(glMultMatrixd)
ANT_GL_IMPL(glMultMatrixf)
ANT_GL_IMPL(glNewList)
ANT_GL_IMPL(glNormal3b)
ANT_GL_IMPL(glNormal3bv)
ANT_GL_IMPL(glNormal3d)
ANT_GL_IMPL(glNormal3dv)
ANT_GL_IMPL(glNormal3f)
ANT_GL_IMPL(glNormal3fv)
ANT_GL_IMPL(glNormal3i)
ANT_GL_IMPL(glNormal3iv)
ANT_GL_IMPL(glNormal3s)
ANT_GL_IMPL(glNormal3sv)
ANT_GL_IMPL(glNormalPointer)
ANT_GL_IMPL(glOrtho)
ANT_GL_IMPL(glPassThrough)
ANT_GL_IMPL(glPixelMapfv)
ANT_GL_IMPL(glPixelMapuiv)
ANT_GL_IMPL(glPixelMapusv)
ANT_GL_IMPL(glPixelStoref)
ANT_GL_IMPL(glPixelStorei)
ANT_GL_IMPL(glPixelTransferf)
ANT_GL_IMPL(glPixelTransferi)
ANT_GL_IMPL(glPixelZoom)
ANT_GL_IMPL(glPointSize)
ANT_GL_IMPL(glPolygonMode)
ANT_GL_IMPL(glPolygonOffset)
ANT_GL_IMPL(glPolygonStipple)
ANT_GL_IMPL(glPopAttrib)
ANT_GL_IMPL(glPopClientAttrib)
ANT_GL_IMPL(glPopMatrix)
ANT_GL_IMPL(glPopName)
ANT_GL_IMPL(glPrioritizeTextures)
ANT_GL_IMPL(glPushAttrib)
ANT_GL_IMPL(glPushClientAttrib)
ANT_GL_IMPL(glPushMatrix)
ANT_GL_IMPL(glPushName)
ANT_GL_IMPL(glRasterPos2d)
ANT_GL_IMPL(glRasterPos2dv)
ANT_GL_IMPL(glRasterPos2f)
ANT_GL_IMPL(glRasterPos2fv)
ANT_GL_IMPL(glRasterPos2i)
ANT_GL_IMPL(glRasterPos2iv)
ANT_GL_IMPL(glRasterPos2s)
ANT_GL_IMPL(glRasterPos2sv)
ANT_GL_IMPL(glRasterPos3d)
ANT_GL_IMPL(glRasterPos3dv)
ANT_GL_IMPL(glRasterPos3f)
ANT_GL_IMPL(glRasterPos3fv)
ANT_GL_IMPL(glRasterPos3i)
ANT_GL_IMPL(glRasterPos3iv)
ANT_GL_IMPL(glRasterPos3s)
ANT_GL_IMPL(glRasterPos3sv)
ANT_GL_IMPL(glRasterPos4d)
ANT_GL_IMPL(glRasterPos4dv)
ANT_GL_IMPL(glRasterPos4f)
ANT_GL_IMPL(glRasterPos4fv)
ANT_GL_IMPL(glRasterPos4i)
ANT_GL_IMPL(glRasterPos4iv)
ANT_GL_IMPL(glRasterPos4s)
ANT_GL_IMPL(glRasterPos4sv)
ANT_GL_IMPL(glReadBuffer)
ANT_GL_IMPL(glReadPixels)
ANT_GL_IMPL(glRectd)
ANT_GL_IMPL(glRectdv)
ANT_GL_IMPL(glRectf)
ANT_GL_IMPL(glRectfv)
ANT_GL_IMPL(glRecti)
ANT_GL_IMPL(glRectiv)
ANT_GL_IMPL(glRects)
ANT_GL_IMPL(glRectsv)
ANT_GL_IMPL(glRenderMode)
ANT_GL_IMPL(glRotated)
ANT_GL_IMPL(glRotatef)
ANT_GL_IMPL(glScaled)
ANT_GL_IMPL(glScalef)
ANT_GL_IMPL(glScissor)
ANT_GL_IMPL(glSelectBuffer)
ANT_GL_IMPL(glShadeModel)
ANT_GL_IMPL(glStencilFunc)
ANT_GL_IMPL(glStencilMask)
ANT_GL_IMPL(glStencilOp)
ANT_GL_IMPL(glTexCoord1d)
ANT_GL_IMPL(glTexCoord1dv)
ANT_GL_IMPL(glTexCoord1f)
ANT_GL_IMPL(glTexCoord1fv)
ANT_GL_IMPL(glTexCoord1i)
ANT_GL_IMPL(glTexCoord1iv)
ANT_GL_IMPL(glTexCoord1s)
ANT_GL_IMPL(glTexCoord1sv)
ANT_GL_IMPL(glTexCoord2d)
ANT_GL_IMPL(glTexCoord2dv)
ANT_GL_IMPL(glTexCoord2f)
ANT_GL_IMPL(glTexCoord2fv)
ANT_GL_IMPL(glTexCoord2i)
ANT_GL_IMPL(glTexCoord2iv)
ANT_GL_IMPL(glTexCoord2s)
ANT_GL_IMPL(glTexCoord2sv)
ANT_GL_IMPL(glTexCoord3d)
ANT_GL_IMPL(glTexCoord3dv)
ANT_GL_IMPL(glTexCoord3f)
ANT_GL_IMPL(glTexCoord3fv)
ANT_GL_IMPL(glTexCoord3i)
ANT_GL_IMPL(glTexCoord3iv)
ANT_GL_IMPL(glTexCoord3s)
ANT_GL_IMPL(glTexCoord3sv)
ANT_GL_IMPL(glTexCoord4d)
ANT_GL_IMPL(glTexCoord4dv)
ANT_GL_IMPL(glTexCoord4f)
ANT_GL_IMPL(glTexCoord4fv)
ANT_GL_IMPL(glTexCoord4i)
ANT_GL_IMPL(glTexCoord4iv)
ANT_GL_IMPL(glTexCoord4s)
ANT_GL_IMPL(glTexCoord4sv)
ANT_GL_IMPL(glTexCoordPointer)
ANT_GL_IMPL(glTexEnvf)
ANT_GL_IMPL(glTexEnvfv)
ANT_GL_IMPL(glTexEnvi)
ANT_GL_IMPL(glTexEnviv)
ANT_GL_IMPL(glTexGend)
ANT_GL_IMPL(glTexGendv)
ANT_GL_IMPL(glTexGenf)
ANT_GL_IMPL(glTexGenfv)
ANT_GL_IMPL(glTexGeni)
ANT_GL_IMPL(glTexGeniv)
ANT_GL_IMPL(glTexImage1D)
ANT_GL_IMPL(glTexImage2D)
ANT_GL_IMPL(glTexParameterf)
ANT_GL_IMPL(glTexParameterfv)
ANT_GL_IMPL(glTexParameteri)
ANT_GL_IMPL(glTexParameteriv)
ANT_GL_IMPL(glTexSubImage1D)
ANT_GL_IMPL(glTexSubImage2D)
ANT_GL_IMPL(glTranslated)
ANT_GL_IMPL(glTranslatef)
ANT_GL_IMPL(glVertex2d)
ANT_GL_IMPL(glVertex2dv)
ANT_GL_IMPL(glVertex2f)
ANT_GL_IMPL(glVertex2fv)
ANT_GL_IMPL(glVertex2i)
ANT_GL_IMPL(glVertex2iv)
ANT_GL_IMPL(glVertex2s)
ANT_GL_IMPL(glVertex2sv)
ANT_GL_IMPL(glVertex3d)
ANT_GL_IMPL(glVertex3dv)
ANT_GL_IMPL(glVertex3f)
ANT_GL_IMPL(glVertex3fv)
ANT_GL_IMPL(glVertex3i)
ANT_GL_IMPL(glVertex3iv)
ANT_GL_IMPL(glVertex3s)
ANT_GL_IMPL(glVertex3sv)
ANT_GL_IMPL(glVertex4d)
ANT_GL_IMPL(glVertex4dv)
ANT_GL_IMPL(glVertex4f)
ANT_GL_IMPL(glVertex4fv)
ANT_GL_IMPL(glVertex4i)
ANT_GL_IMPL(glVertex4iv)
ANT_GL_IMPL(glVertex4s)
ANT_GL_IMPL(glVertex4sv)
ANT_GL_IMPL(glVertexPointer)
ANT_GL_IMPL(glViewport)
#if defined(ANT_WINDOWS)
ANT_GL_IMPL(wglGetProcAddress)
#endif

namespace GL { PFNGLGetProcAddress _glGetProcAddress = NULL; }

//  ---------------------------------------------------------------------------

#if defined(ANT_WINDOWS)

    //  ---------------------------------------------------------------------------
    
    int LoadOpenGL()
    {
        if( g_OGLModule!=NULL )
        {
            return 1; // "OpenGL library already loaded"
        }
    
        g_OGLModule = LoadLibrary("OPENGL32.DLL");
        if( g_OGLModule )
        {
            // Info(VERB_LOW, "Load %d OpenGL functions", g_NbOGLFunc);
    
            int Res = 1;
            for(int i=0; i<g_NbOGLFunc; ++i)
            {
                assert(g_OGLFuncRec[i].m_FuncPtr!=NULL);
                assert(*(g_OGLFuncRec[i].m_FuncPtr)==NULL);
                assert(g_OGLFuncRec[i].m_Name!=NULL);
                assert(strlen(g_OGLFuncRec[i].m_Name)>0);
                *(g_OGLFuncRec[i].m_FuncPtr) = reinterpret_cast<GL::PFNOpenGL>(GetProcAddress(g_OGLModule, g_OGLFuncRec[i].m_Name));
                if( *(g_OGLFuncRec[i].m_FuncPtr)==NULL )
                    Res = 0; // Error("cannot find OpenGL function");
    
            }

            _glGetProcAddress = reinterpret_cast<GL::PFNGLGetProcAddress>(_wglGetProcAddress);
            if( _glGetProcAddress==NULL )
                Res = 0;

            return Res;
        }
        else
        {
            // InternDisplayLastErrorWIN("Cannot load opengl32 DLL", false);
            return 0;   // cannot load DLL
        }
    }
    
    //  ---------------------------------------------------------------------------
    
    int UnloadOpenGL()
    {
        if( g_OGLModule==NULL )
        {
            return 1; // "OpenGL library not loaded"
        }
    
        // Info(VERB_LOW, "Unload %d OpenGL functions", g_NbOGLFunc);
        for(int i=0; i<g_NbOGLFunc; ++i)
        {
            assert(g_OGLFuncRec[i].m_FuncPtr!=NULL);
            assert(*(g_OGLFuncRec[i].m_FuncPtr)!=NULL);
            assert(g_OGLFuncRec[i].m_Name!=NULL);
            assert(strlen(g_OGLFuncRec[i].m_Name)>0);
            *(g_OGLFuncRec[i].m_FuncPtr) = NULL;
        }
        if( FreeLibrary(g_OGLModule) )
        {
            // Info(VERB_LOW, "OpenGL library unloaded");
            g_OGLModule = NULL;
            return 1;
        }
        else
        {
            // InternDisplayLastErrorWIN("Cannot unload opengl32 DLL", false);
            return 0; // cannot unload opengl32.dll
        }
    }
    
    //  ---------------------------------------------------------------------------
    
    namespace GL
    {
    
        PFNOpenGL Record(const char *_FuncName, PFNOpenGL *_FuncPtr)
        {
            if( g_NbOGLFunc>=ANT_NB_OGL_FUNC_MAX )
            {
                fprintf(stderr, "Too many OpenGL functions declared. Change ANT_NB_OGL_FUNC_MAX.");
                exit(-1);
            }
    
            g_OGLFuncRec[g_NbOGLFunc].m_Name = _FuncName;
            g_OGLFuncRec[g_NbOGLFunc].m_FuncPtr = _FuncPtr;
            ++g_NbOGLFunc;
    
            return NULL;
        }
    
    } // namespace GL
    
    //  ---------------------------------------------------------------------------

#endif // defined(ANT_WINDOWS)

//  ---------------------------------------------------------------------------

#if defined(ANT_UNIX)
    
    int LoadOpenGL()
    {
        _glGetProcAddress = reinterpret_cast<GL::PFNGLGetProcAddress>(glXGetProcAddressARB);

        return 1; // "OpenGL library is statically linked"
    }
    
    int UnloadOpenGL()
    {
        return 1; // "OpenGL library is statically linked"
    }
    
#elif defined(ANT_OSX)

    #include <dlfcn.h>

    static void *gl_dyld = NULL;
    void *NSGLGetProcAddressNew(const GLubyte *name) 
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

    int LoadOpenGL() 
    {
        _glGetProcAddress = reinterpret_cast<GL::PFNGLGetProcAddress>(NSGLGetProcAddressNew);
        return 1;
    }

    int UnloadOpenGL() 
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
