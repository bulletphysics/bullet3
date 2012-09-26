//  ---------------------------------------------------------------------------
//
//  @file       LoadOGLCore.h
//  @brief      OpenGL Core Profile declarations for dynamic loading
//  @author     Philippe Decaudin - http://www.antisphere.com
//  @license    This file is part of the AntTweakBar library.
//              For conditions of distribution and use, see License.txt
//
//  note:       Private header
//
//  ---------------------------------------------------------------------------


#if !defined ANT_LOAD_OGL_CORE_INCLUDED
#define ANT_LOAD_OGL_CORE_INCLUDED


#define ANT_GL_CORE_DECL_NO_FORWARD(_Ret, _Fct, _Params) \
    extern "C" { typedef _Ret (APIENTRY* PFN##_Fct)_Params; } \
    namespace GLCore { extern PFN##_Fct _##_Fct; } \
    using GLCore::_##_Fct;

#if defined(ANT_WINDOWS)
#   define ANT_GL_CORE_DECL(_Ret, _Fct, _Params) \
        ANT_GL_CORE_DECL_NO_FORWARD(_Ret, _Fct, _Params)
#   define ANT_GL_CORE_IMPL(_Fct) \
        namespace GLCore { PFN##_Fct _##_Fct = (PFN##_Fct)Record(#_Fct, (PFNOpenGL*)(&_##_Fct)); }
#elif defined(ANT_UNIX) || defined(ANT_OSX)
#   if !defined(APIENTRY)
#       define APIENTRY
#   endif
#   define ANT_GL_CORE_DECL(_Ret, _Fct, _Params) \
        ANT_GL_CORE_DECL_NO_FORWARD(_Ret, _Fct, _Params) \
        extern "C" { _Ret APIENTRY _Fct _Params; }
#   define ANT_GL_CORE_IMPL(_Fct) \
        namespace GLCore { PFN##_Fct _##_Fct = _Fct; }
#endif


int LoadOpenGLCore();
int UnloadOpenGLCore();

namespace GLCore
{
    extern "C" { typedef void (APIENTRY* PFNOpenGL)(); }
    PFNOpenGL Record(const char *_FuncName, PFNOpenGL *_FuncPtr);

    extern "C" { typedef PFNOpenGL (APIENTRY *PFNGLGetProcAddress)(const char *); }
    extern PFNGLGetProcAddress _glGetProcAddress;
}
using GLCore::_glGetProcAddress;


// GL 1.0
ANT_GL_CORE_DECL(void, glCullFace, (GLenum mode))
ANT_GL_CORE_DECL(void, glFrontFace, (GLenum mode))
ANT_GL_CORE_DECL(void, glHint, (GLenum target, GLenum mode))
ANT_GL_CORE_DECL(void, glLineWidth, (GLfloat width))
ANT_GL_CORE_DECL(void, glPointSize, (GLfloat size))
ANT_GL_CORE_DECL(void, glPolygonMode, (GLenum face, GLenum mode))
ANT_GL_CORE_DECL(void, glScissor, (GLint x, GLint y, GLsizei width, GLsizei height))
ANT_GL_CORE_DECL(void, glTexParameterf, (GLenum target, GLenum pname, GLfloat param))
ANT_GL_CORE_DECL(void, glTexParameterfv, (GLenum target, GLenum pname, const GLfloat *params))
ANT_GL_CORE_DECL(void, glTexParameteri, (GLenum target, GLenum pname, GLint param))
ANT_GL_CORE_DECL(void, glTexParameteriv, (GLenum target, GLenum pname, const GLint *params))
#if defined(ANT_OSX) && (MAC_OS_X_VERSION_MAX_ALLOWED < 1070)
// Mac OSX < 10.7 redefines these OpenGL calls: glTexImage1D, glTexImage2D
ANT_GL_CORE_DECL(void, glTexImage1D, (GLenum target, GLint level, GLenum internalformat, GLsizei width, GLint border, GLenum format, GLenum type, const GLvoid *pixels))
ANT_GL_CORE_DECL(void, glTexImage2D, (GLenum target, GLint level, GLenum internalformat, GLsizei width, GLsizei height, GLint border, GLenum format, GLenum type, const GLvoid *pixels))
#else
ANT_GL_CORE_DECL(void, glTexImage1D, (GLenum target, GLint level, GLint internalformat, GLsizei width, GLint border, GLenum format, GLenum type, const GLvoid *pixels))
ANT_GL_CORE_DECL(void, glTexImage2D, (GLenum target, GLint level, GLint internalformat, GLsizei width, GLsizei height, GLint border, GLenum format, GLenum type, const GLvoid *pixels))
#endif
ANT_GL_CORE_DECL(void, glDrawBuffer, (GLenum mode))
ANT_GL_CORE_DECL(void, glClear, (GLbitfield mask))
ANT_GL_CORE_DECL(void, glClearColor, (GLclampf red, GLclampf green, GLclampf blue, GLclampf alpha))
ANT_GL_CORE_DECL(void, glClearStencil, (GLint s))
ANT_GL_CORE_DECL(void, glClearDepth, (GLclampd depth))
ANT_GL_CORE_DECL(void, glStencilMask, (GLuint mask))
ANT_GL_CORE_DECL(void, glColorMask, (GLboolean red, GLboolean green, GLboolean blue, GLboolean alpha))
ANT_GL_CORE_DECL(void, glDepthMask, (GLboolean flag))
ANT_GL_CORE_DECL(void, glDisable, (GLenum cap))
ANT_GL_CORE_DECL(void, glEnable, (GLenum cap))
ANT_GL_CORE_DECL(void, glFinish, (void))
ANT_GL_CORE_DECL(void, glFlush, (void))
ANT_GL_CORE_DECL(void, glBlendFunc, (GLenum sfactor, GLenum dfactor))
ANT_GL_CORE_DECL(void, glLogicOp, (GLenum opcode))
ANT_GL_CORE_DECL(void, glStencilFunc, (GLenum func, GLint ref, GLuint mask))
ANT_GL_CORE_DECL(void, glStencilOp, (GLenum fail, GLenum zfail, GLenum zpass))
ANT_GL_CORE_DECL(void, glDepthFunc, (GLenum func))
ANT_GL_CORE_DECL(void, glPixelStoref, (GLenum pname, GLfloat param))
ANT_GL_CORE_DECL(void, glPixelStorei, (GLenum pname, GLint param))
ANT_GL_CORE_DECL(void, glReadBuffer, (GLenum mode))
ANT_GL_CORE_DECL(void, glReadPixels, (GLint x, GLint y, GLsizei width, GLsizei height, GLenum format, GLenum type, GLvoid *pixels))
ANT_GL_CORE_DECL(void, glGetBooleanv, (GLenum pname, GLboolean *params))
ANT_GL_CORE_DECL(void, glGetDoublev, (GLenum pname, GLdouble *params))
ANT_GL_CORE_DECL(GLenum, glGetError, (void))
ANT_GL_CORE_DECL(void, glGetFloatv, (GLenum pname, GLfloat *params))
ANT_GL_CORE_DECL(void, glGetIntegerv, (GLenum pname, GLint *params))
ANT_GL_CORE_DECL(const GLubyte *, glGetString, (GLenum name))
ANT_GL_CORE_DECL(void, glGetTexImage, (GLenum target, GLint level, GLenum format, GLenum type, GLvoid *pixels))
ANT_GL_CORE_DECL(void, glGetTexParameterfv, (GLenum target, GLenum pname, GLfloat *params))
ANT_GL_CORE_DECL(void, glGetTexParameteriv, (GLenum target, GLenum pname, GLint *params))
ANT_GL_CORE_DECL(void, glGetTexLevelParameterfv, (GLenum target, GLint level, GLenum pname, GLfloat *params))
ANT_GL_CORE_DECL(void, glGetTexLevelParameteriv, (GLenum target, GLint level, GLenum pname, GLint *params))
ANT_GL_CORE_DECL(GLboolean, glIsEnabled, (GLenum cap))
ANT_GL_CORE_DECL(void, glDepthRange, (GLclampd near, GLclampd far))
ANT_GL_CORE_DECL(void, glViewport, (GLint x, GLint y, GLsizei width, GLsizei height))
// GL 1.1
ANT_GL_CORE_DECL(void, glDrawArrays, (GLenum mode, GLint first, GLsizei count))
ANT_GL_CORE_DECL(void, glDrawElements, (GLenum mode, GLsizei count, GLenum type, const GLvoid *indices))
ANT_GL_CORE_DECL(void, glGetPointerv, (GLenum pname, GLvoid* *params))
ANT_GL_CORE_DECL(void, glPolygonOffset, (GLfloat factor, GLfloat units))
ANT_GL_CORE_DECL(void, glCopyTexImage1D, (GLenum target, GLint level, GLenum internalformat, GLint x, GLint y, GLsizei width, GLint border))
ANT_GL_CORE_DECL(void, glCopyTexImage2D, (GLenum target, GLint level, GLenum internalformat, GLint x, GLint y, GLsizei width, GLsizei height, GLint border))
ANT_GL_CORE_DECL(void, glCopyTexSubImage1D, (GLenum target, GLint level, GLint xoffset, GLint x, GLint y, GLsizei width))
ANT_GL_CORE_DECL(void, glCopyTexSubImage2D, (GLenum target, GLint level, GLint xoffset, GLint yoffset, GLint x, GLint y, GLsizei width, GLsizei height))
ANT_GL_CORE_DECL(void, glTexSubImage1D, (GLenum target, GLint level, GLint xoffset, GLsizei width, GLenum format, GLenum type, const GLvoid *pixels))
ANT_GL_CORE_DECL(void, glTexSubImage2D, (GLenum target, GLint level, GLint xoffset, GLint yoffset, GLsizei width, GLsizei height, GLenum format, GLenum type, const GLvoid *pixels))
ANT_GL_CORE_DECL(void, glBindTexture, (GLenum target, GLuint texture))
ANT_GL_CORE_DECL(void, glDeleteTextures, (GLsizei n, const GLuint *textures))
ANT_GL_CORE_DECL(void, glGenTextures, (GLsizei n, GLuint *textures))
ANT_GL_CORE_DECL(GLboolean, glIsTexture, (GLuint texture))
// GL 1.2
ANT_GL_CORE_DECL(void, glBlendColor, (GLclampf red, GLclampf green, GLclampf blue, GLclampf alpha))
ANT_GL_CORE_DECL(void, glBlendEquation, (GLenum mode))
ANT_GL_CORE_DECL(void, glDrawRangeElements, (GLenum mode, GLuint start, GLuint end, GLsizei count, GLenum type, const GLvoid *indices))
#if defined(ANT_OSX) && (MAC_OS_X_VERSION_MAX_ALLOWED < 1070)
// Mac OSX < 10.7 redefines this OpenGL call: glTexImage3D
ANT_GL_CORE_DECL(void, glTexImage3D, (GLenum target, GLint level, GLenum internalformat, GLsizei width, GLsizei height, GLsizei depth, GLint border, GLenum format, GLenum type, const GLvoid *pixels))
#else
ANT_GL_CORE_DECL(void, glTexImage3D, (GLenum target, GLint level, GLint internalformat, GLsizei width, GLsizei height, GLsizei depth, GLint border, GLenum format, GLenum type, const GLvoid *pixels))
#endif
ANT_GL_CORE_DECL(void, glTexSubImage3D, (GLenum target, GLint level, GLint xoffset, GLint yoffset, GLint zoffset, GLsizei width, GLsizei height, GLsizei depth, GLenum format, GLenum type, const GLvoid *pixels))
ANT_GL_CORE_DECL(void, glCopyTexSubImage3D, (GLenum target, GLint level, GLint xoffset, GLint yoffset, GLint zoffset, GLint x, GLint y, GLsizei width, GLsizei height))
// GL 1.3
ANT_GL_CORE_DECL(void, glActiveTexture, (GLenum texture))
ANT_GL_CORE_DECL(void, glSampleCoverage, (GLclampf value, GLboolean invert))
ANT_GL_CORE_DECL(void, glCompressedTexImage3D, (GLenum target, GLint level, GLenum internalformat, GLsizei width, GLsizei height, GLsizei depth, GLint border, GLsizei imageSize, const GLvoid *data))
ANT_GL_CORE_DECL(void, glCompressedTexImage2D, (GLenum target, GLint level, GLenum internalformat, GLsizei width, GLsizei height, GLint border, GLsizei imageSize, const GLvoid *data))
ANT_GL_CORE_DECL(void, glCompressedTexImage1D, (GLenum target, GLint level, GLenum internalformat, GLsizei width, GLint border, GLsizei imageSize, const GLvoid *data))
ANT_GL_CORE_DECL(void, glCompressedTexSubImage3D, (GLenum target, GLint level, GLint xoffset, GLint yoffset, GLint zoffset, GLsizei width, GLsizei height, GLsizei depth, GLenum format, GLsizei imageSize, const GLvoid *data))
ANT_GL_CORE_DECL(void, glCompressedTexSubImage2D, (GLenum target, GLint level, GLint xoffset, GLint yoffset, GLsizei width, GLsizei height, GLenum format, GLsizei imageSize, const GLvoid *data))
ANT_GL_CORE_DECL(void, glCompressedTexSubImage1D, (GLenum target, GLint level, GLint xoffset, GLsizei width, GLenum format, GLsizei imageSize, const GLvoid *data))
ANT_GL_CORE_DECL(void, glGetCompressedTexImage, (GLenum target, GLint level, GLvoid *img))
// GL 1.4
ANT_GL_CORE_DECL(void, glBlendFuncSeparate, (GLenum sfactorRGB, GLenum dfactorRGB, GLenum sfactorAlpha, GLenum dfactorAlpha))
ANT_GL_CORE_DECL(void, glMultiDrawArrays, (GLenum mode, const GLint *first, const GLsizei *count, GLsizei primcount))
ANT_GL_CORE_DECL(void, glMultiDrawElements, (GLenum mode, const GLsizei *count, GLenum type, const GLvoid* *indices, GLsizei primcount))
ANT_GL_CORE_DECL(void, glPointParameterf, (GLenum pname, GLfloat param))
ANT_GL_CORE_DECL(void, glPointParameterfv, (GLenum pname, const GLfloat *params))
ANT_GL_CORE_DECL(void, glPointParameteri, (GLenum pname, GLint param))
ANT_GL_CORE_DECL(void, glPointParameteriv, (GLenum pname, const GLint *params))
// GL 1.5
#ifndef ANT_OSX
    typedef ptrdiff_t GLintptr;
    typedef ptrdiff_t GLsizeiptr;
#endif
ANT_GL_CORE_DECL(void, glGenQueries, (GLsizei n, GLuint *ids))
ANT_GL_CORE_DECL(void, glDeleteQueries, (GLsizei n, const GLuint *ids))
ANT_GL_CORE_DECL(GLboolean, glIsQuery, (GLuint id))
ANT_GL_CORE_DECL(void, glBeginQuery, (GLenum target, GLuint id))
ANT_GL_CORE_DECL(void, glEndQuery, (GLenum target))
ANT_GL_CORE_DECL(void, glGetQueryiv, (GLenum target, GLenum pname, GLint *params))
ANT_GL_CORE_DECL(void, glGetQueryObjectiv, (GLuint id, GLenum pname, GLint *params))
ANT_GL_CORE_DECL(void, glGetQueryObjectuiv, (GLuint id, GLenum pname, GLuint *params))
ANT_GL_CORE_DECL(void, glBindBuffer, (GLenum target, GLuint buffer))
ANT_GL_CORE_DECL(void, glDeleteBuffers, (GLsizei n, const GLuint *buffers))
ANT_GL_CORE_DECL(void, glGenBuffers, (GLsizei n, GLuint *buffers))
ANT_GL_CORE_DECL(GLboolean, glIsBuffer, (GLuint buffer))
ANT_GL_CORE_DECL(void, glBufferData, (GLenum target, GLsizeiptr size, const GLvoid *data, GLenum usage))
ANT_GL_CORE_DECL(void, glBufferSubData, (GLenum target, GLintptr offset, GLsizeiptr size, const GLvoid *data))
ANT_GL_CORE_DECL(void, glGetBufferSubData, (GLenum target, GLintptr offset, GLsizeiptr size, GLvoid *data))
ANT_GL_CORE_DECL(GLvoid*, glMapBuffer, (GLenum target, GLenum access))
ANT_GL_CORE_DECL(GLboolean, glUnmapBuffer, (GLenum target))
ANT_GL_CORE_DECL(void, glGetBufferParameteriv, (GLenum target, GLenum pname, GLint *params))
ANT_GL_CORE_DECL(void, glGetBufferPointerv, (GLenum target, GLenum pname, GLvoid* *params))
// GL 2.0
typedef char GLchar;
ANT_GL_CORE_DECL(void, glBlendEquationSeparate, (GLenum modeRGB, GLenum modeAlpha))
ANT_GL_CORE_DECL(void, glDrawBuffers, (GLsizei n, const GLenum *bufs))
ANT_GL_CORE_DECL(void, glStencilOpSeparate, (GLenum face, GLenum sfail, GLenum dpfail, GLenum dppass))
ANT_GL_CORE_DECL(void, glStencilFuncSeparate, (GLenum face, GLenum func, GLint ref, GLuint mask))
ANT_GL_CORE_DECL(void, glStencilMaskSeparate, (GLenum face, GLuint mask))
ANT_GL_CORE_DECL(void, glAttachShader, (GLuint program, GLuint shader))
ANT_GL_CORE_DECL(void, glBindAttribLocation, (GLuint program, GLuint index, const GLchar *name))
ANT_GL_CORE_DECL(void, glCompileShader, (GLuint shader))
ANT_GL_CORE_DECL(GLuint, glCreateProgram, (void))
ANT_GL_CORE_DECL(GLuint, glCreateShader, (GLenum type))
ANT_GL_CORE_DECL(void, glDeleteProgram, (GLuint program))
ANT_GL_CORE_DECL(void, glDeleteShader, (GLuint shader))
ANT_GL_CORE_DECL(void, glDetachShader, (GLuint program, GLuint shader))
ANT_GL_CORE_DECL(void, glDisableVertexAttribArray, (GLuint index))
ANT_GL_CORE_DECL(void, glEnableVertexAttribArray, (GLuint index))
ANT_GL_CORE_DECL(void, glGetActiveAttrib, (GLuint program, GLuint index, GLsizei bufSize, GLsizei *length, GLint *size, GLenum *type, GLchar *name))
ANT_GL_CORE_DECL(void, glGetActiveUniform, (GLuint program, GLuint index, GLsizei bufSize, GLsizei *length, GLint *size, GLenum *type, GLchar *name))
ANT_GL_CORE_DECL(void, glGetAttachedShaders, (GLuint program, GLsizei maxCount, GLsizei *count, GLuint *obj))
ANT_GL_CORE_DECL(GLint, glGetAttribLocation, (GLuint program, const GLchar *name))
ANT_GL_CORE_DECL(void, glGetProgramiv, (GLuint program, GLenum pname, GLint *params))
ANT_GL_CORE_DECL(void, glGetProgramInfoLog, (GLuint program, GLsizei bufSize, GLsizei *length, GLchar *infoLog))
ANT_GL_CORE_DECL(void, glGetShaderiv, (GLuint shader, GLenum pname, GLint *params))
ANT_GL_CORE_DECL(void, glGetShaderInfoLog, (GLuint shader, GLsizei bufSize, GLsizei *length, GLchar *infoLog))
ANT_GL_CORE_DECL(void, glGetShaderSource, (GLuint shader, GLsizei bufSize, GLsizei *length, GLchar *source))
ANT_GL_CORE_DECL(GLint, glGetUniformLocation, (GLuint program, const GLchar *name))
ANT_GL_CORE_DECL(void, glGetUniformfv, (GLuint program, GLint location, GLfloat *params))
ANT_GL_CORE_DECL(void, glGetUniformiv, (GLuint program, GLint location, GLint *params))
ANT_GL_CORE_DECL(void, glGetVertexAttribdv, (GLuint index, GLenum pname, GLdouble *params))
ANT_GL_CORE_DECL(void, glGetVertexAttribfv, (GLuint index, GLenum pname, GLfloat *params))
ANT_GL_CORE_DECL(void, glGetVertexAttribiv, (GLuint index, GLenum pname, GLint *params))
ANT_GL_CORE_DECL(void, glGetVertexAttribPointerv, (GLuint index, GLenum pname, GLvoid* *pointer))
ANT_GL_CORE_DECL(GLboolean, glIsProgram, (GLuint program))
ANT_GL_CORE_DECL(GLboolean, glIsShader, (GLuint shader))
ANT_GL_CORE_DECL(void, glLinkProgram, (GLuint program))
ANT_GL_CORE_DECL(void, glShaderSource, (GLuint shader, GLsizei count, const GLchar* *string, const GLint *length))
ANT_GL_CORE_DECL(void, glUseProgram, (GLuint program))
ANT_GL_CORE_DECL(void, glUniform1f, (GLint location, GLfloat v0))
ANT_GL_CORE_DECL(void, glUniform2f, (GLint location, GLfloat v0, GLfloat v1))
ANT_GL_CORE_DECL(void, glUniform3f, (GLint location, GLfloat v0, GLfloat v1, GLfloat v2))
ANT_GL_CORE_DECL(void, glUniform4f, (GLint location, GLfloat v0, GLfloat v1, GLfloat v2, GLfloat v3))
ANT_GL_CORE_DECL(void, glUniform1i, (GLint location, GLint v0))
ANT_GL_CORE_DECL(void, glUniform2i, (GLint location, GLint v0, GLint v1))
ANT_GL_CORE_DECL(void, glUniform3i, (GLint location, GLint v0, GLint v1, GLint v2))
ANT_GL_CORE_DECL(void, glUniform4i, (GLint location, GLint v0, GLint v1, GLint v2, GLint v3))
ANT_GL_CORE_DECL(void, glUniform1fv, (GLint location, GLsizei count, const GLfloat *value))
ANT_GL_CORE_DECL(void, glUniform2fv, (GLint location, GLsizei count, const GLfloat *value))
ANT_GL_CORE_DECL(void, glUniform3fv, (GLint location, GLsizei count, const GLfloat *value))
ANT_GL_CORE_DECL(void, glUniform4fv, (GLint location, GLsizei count, const GLfloat *value))
ANT_GL_CORE_DECL(void, glUniform1iv, (GLint location, GLsizei count, const GLint *value))
ANT_GL_CORE_DECL(void, glUniform2iv, (GLint location, GLsizei count, const GLint *value))
ANT_GL_CORE_DECL(void, glUniform3iv, (GLint location, GLsizei count, const GLint *value))
ANT_GL_CORE_DECL(void, glUniform4iv, (GLint location, GLsizei count, const GLint *value))
ANT_GL_CORE_DECL(void, glUniformMatrix2fv, (GLint location, GLsizei count, GLboolean transpose, const GLfloat *value))
ANT_GL_CORE_DECL(void, glUniformMatrix3fv, (GLint location, GLsizei count, GLboolean transpose, const GLfloat *value))
ANT_GL_CORE_DECL(void, glUniformMatrix4fv, (GLint location, GLsizei count, GLboolean transpose, const GLfloat *value))
ANT_GL_CORE_DECL(void, glValidateProgram, (GLuint program))
ANT_GL_CORE_DECL(void, glVertexAttrib1d, (GLuint index, GLdouble x))
ANT_GL_CORE_DECL(void, glVertexAttrib1dv, (GLuint index, const GLdouble *v))
ANT_GL_CORE_DECL(void, glVertexAttrib1f, (GLuint index, GLfloat x))
ANT_GL_CORE_DECL(void, glVertexAttrib1fv, (GLuint index, const GLfloat *v))
ANT_GL_CORE_DECL(void, glVertexAttrib1s, (GLuint index, GLshort x))
ANT_GL_CORE_DECL(void, glVertexAttrib1sv, (GLuint index, const GLshort *v))
ANT_GL_CORE_DECL(void, glVertexAttrib2d, (GLuint index, GLdouble x, GLdouble y))
ANT_GL_CORE_DECL(void, glVertexAttrib2dv, (GLuint index, const GLdouble *v))
ANT_GL_CORE_DECL(void, glVertexAttrib2f, (GLuint index, GLfloat x, GLfloat y))
ANT_GL_CORE_DECL(void, glVertexAttrib2fv, (GLuint index, const GLfloat *v))
ANT_GL_CORE_DECL(void, glVertexAttrib2s, (GLuint index, GLshort x, GLshort y))
ANT_GL_CORE_DECL(void, glVertexAttrib2sv, (GLuint index, const GLshort *v))
ANT_GL_CORE_DECL(void, glVertexAttrib3d, (GLuint index, GLdouble x, GLdouble y, GLdouble z))
ANT_GL_CORE_DECL(void, glVertexAttrib3dv, (GLuint index, const GLdouble *v))
ANT_GL_CORE_DECL(void, glVertexAttrib3f, (GLuint index, GLfloat x, GLfloat y, GLfloat z))
ANT_GL_CORE_DECL(void, glVertexAttrib3fv, (GLuint index, const GLfloat *v))
ANT_GL_CORE_DECL(void, glVertexAttrib3s, (GLuint index, GLshort x, GLshort y, GLshort z))
ANT_GL_CORE_DECL(void, glVertexAttrib3sv, (GLuint index, const GLshort *v))
ANT_GL_CORE_DECL(void, glVertexAttrib4Nbv, (GLuint index, const GLbyte *v))
ANT_GL_CORE_DECL(void, glVertexAttrib4Niv, (GLuint index, const GLint *v))
ANT_GL_CORE_DECL(void, glVertexAttrib4Nsv, (GLuint index, const GLshort *v))
ANT_GL_CORE_DECL(void, glVertexAttrib4Nub, (GLuint index, GLubyte x, GLubyte y, GLubyte z, GLubyte w))
ANT_GL_CORE_DECL(void, glVertexAttrib4Nubv, (GLuint index, const GLubyte *v))
ANT_GL_CORE_DECL(void, glVertexAttrib4Nuiv, (GLuint index, const GLuint *v))
ANT_GL_CORE_DECL(void, glVertexAttrib4Nusv, (GLuint index, const GLushort *v))
ANT_GL_CORE_DECL(void, glVertexAttrib4bv, (GLuint index, const GLbyte *v))
ANT_GL_CORE_DECL(void, glVertexAttrib4d, (GLuint index, GLdouble x, GLdouble y, GLdouble z, GLdouble w))
ANT_GL_CORE_DECL(void, glVertexAttrib4dv, (GLuint index, const GLdouble *v))
ANT_GL_CORE_DECL(void, glVertexAttrib4f, (GLuint index, GLfloat x, GLfloat y, GLfloat z, GLfloat w))
ANT_GL_CORE_DECL(void, glVertexAttrib4fv, (GLuint index, const GLfloat *v))
ANT_GL_CORE_DECL(void, glVertexAttrib4iv, (GLuint index, const GLint *v))
ANT_GL_CORE_DECL(void, glVertexAttrib4s, (GLuint index, GLshort x, GLshort y, GLshort z, GLshort w))
ANT_GL_CORE_DECL(void, glVertexAttrib4sv, (GLuint index, const GLshort *v))
ANT_GL_CORE_DECL(void, glVertexAttrib4ubv, (GLuint index, const GLubyte *v))
ANT_GL_CORE_DECL(void, glVertexAttrib4uiv, (GLuint index, const GLuint *v))
ANT_GL_CORE_DECL(void, glVertexAttrib4usv, (GLuint index, const GLushort *v))
ANT_GL_CORE_DECL(void, glVertexAttribPointer, (GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const GLvoid *pointer))
/*
// GL 2.1
ANT_GL_CORE_DECL(void, glUniformMatrix2x3fv, (GLint location, GLsizei count, GLboolean transpose, const GLfloat *value))
ANT_GL_CORE_DECL(void, glUniformMatrix3x2fv, (GLint location, GLsizei count, GLboolean transpose, const GLfloat *value))
ANT_GL_CORE_DECL(void, glUniformMatrix2x4fv, (GLint location, GLsizei count, GLboolean transpose, const GLfloat *value))
ANT_GL_CORE_DECL(void, glUniformMatrix4x2fv, (GLint location, GLsizei count, GLboolean transpose, const GLfloat *value))
ANT_GL_CORE_DECL(void, glUniformMatrix3x4fv, (GLint location, GLsizei count, GLboolean transpose, const GLfloat *value))
ANT_GL_CORE_DECL(void, glUniformMatrix4x3fv, (GLint location, GLsizei count, GLboolean transpose, const GLfloat *value))
// GL 3.0
ANT_GL_CORE_DECL(void, glColorMaski, (GLuint index, GLboolean r, GLboolean g, GLboolean b, GLboolean a))
ANT_GL_CORE_DECL(void, glGetBooleani_v, (GLenum target, GLuint index, GLboolean *data))
ANT_GL_CORE_DECL(void, glGetIntegeri_v, (GLenum target, GLuint index, GLint *data))
ANT_GL_CORE_DECL(void, glEnablei, (GLenum target, GLuint index))
ANT_GL_CORE_DECL(void, glDisablei, (GLenum target, GLuint index))
ANT_GL_CORE_DECL(GLboolean, glIsEnabledi, (GLenum target, GLuint index))
ANT_GL_CORE_DECL(void, glBeginTransformFeedback, (GLenum primitiveMode))
ANT_GL_CORE_DECL(void, glEndTransformFeedback, (void))
ANT_GL_CORE_DECL(void, glBindBufferRange, (GLenum target, GLuint index, GLuint buffer, GLintptr offset, GLsizeiptr size))
ANT_GL_CORE_DECL(void, glBindBufferBase, (GLenum target, GLuint index, GLuint buffer))
ANT_GL_CORE_DECL(void, glTransformFeedbackVaryings, (GLuint program, GLsizei count, const GLchar* *varyings, GLenum bufferMode))
ANT_GL_CORE_DECL(void, glGetTransformFeedbackVarying, (GLuint program, GLuint index, GLsizei bufSize, GLsizei *length, GLsizei *size, GLenum *type, GLchar *name))
ANT_GL_CORE_DECL(void, glClampColor, (GLenum target, GLenum clamp))
ANT_GL_CORE_DECL(void, glBeginConditionalRender, (GLuint id, GLenum mode))
ANT_GL_CORE_DECL(void, glEndConditionalRender, (void))
ANT_GL_CORE_DECL(void, glVertexAttribIPointer, (GLuint index, GLint size, GLenum type, GLsizei stride, const GLvoid *pointer))
ANT_GL_CORE_DECL(void, glGetVertexAttribIiv, (GLuint index, GLenum pname, GLint *params))
ANT_GL_CORE_DECL(void, glGetVertexAttribIuiv, (GLuint index, GLenum pname, GLuint *params))
ANT_GL_CORE_DECL(void, glVertexAttribI1i, (GLuint index, GLint x))
ANT_GL_CORE_DECL(void, glVertexAttribI2i, (GLuint index, GLint x, GLint y))
ANT_GL_CORE_DECL(void, glVertexAttribI3i, (GLuint index, GLint x, GLint y, GLint z))
ANT_GL_CORE_DECL(void, glVertexAttribI4i, (GLuint index, GLint x, GLint y, GLint z, GLint w))
ANT_GL_CORE_DECL(void, glVertexAttribI1ui, (GLuint index, GLuint x))
ANT_GL_CORE_DECL(void, glVertexAttribI2ui, (GLuint index, GLuint x, GLuint y))
ANT_GL_CORE_DECL(void, glVertexAttribI3ui, (GLuint index, GLuint x, GLuint y, GLuint z))
ANT_GL_CORE_DECL(void, glVertexAttribI4ui, (GLuint index, GLuint x, GLuint y, GLuint z, GLuint w))
ANT_GL_CORE_DECL(void, glVertexAttribI1iv, (GLuint index, const GLint *v))
ANT_GL_CORE_DECL(void, glVertexAttribI2iv, (GLuint index, const GLint *v))
ANT_GL_CORE_DECL(void, glVertexAttribI3iv, (GLuint index, const GLint *v))
ANT_GL_CORE_DECL(void, glVertexAttribI4iv, (GLuint index, const GLint *v))
ANT_GL_CORE_DECL(void, glVertexAttribI1uiv, (GLuint index, const GLuint *v))
ANT_GL_CORE_DECL(void, glVertexAttribI2uiv, (GLuint index, const GLuint *v))
ANT_GL_CORE_DECL(void, glVertexAttribI3uiv, (GLuint index, const GLuint *v))
ANT_GL_CORE_DECL(void, glVertexAttribI4uiv, (GLuint index, const GLuint *v))
ANT_GL_CORE_DECL(void, glVertexAttribI4bv, (GLuint index, const GLbyte *v))
ANT_GL_CORE_DECL(void, glVertexAttribI4sv, (GLuint index, const GLshort *v))
ANT_GL_CORE_DECL(void, glVertexAttribI4ubv, (GLuint index, const GLubyte *v))
ANT_GL_CORE_DECL(void, glVertexAttribI4usv, (GLuint index, const GLushort *v))
ANT_GL_CORE_DECL(void, glGetUniformuiv, (GLuint program, GLint location, GLuint *params))
ANT_GL_CORE_DECL(void, glBindFragDataLocation, (GLuint program, GLuint color, const GLchar *name))
ANT_GL_CORE_DECL(GLint, glGetFragDataLocation, (GLuint program, const GLchar *name))
ANT_GL_CORE_DECL(void, glUniform1ui, (GLint location, GLuint v0))
ANT_GL_CORE_DECL(void, glUniform2ui, (GLint location, GLuint v0, GLuint v1))
ANT_GL_CORE_DECL(void, glUniform3ui, (GLint location, GLuint v0, GLuint v1, GLuint v2))
ANT_GL_CORE_DECL(void, glUniform4ui, (GLint location, GLuint v0, GLuint v1, GLuint v2, GLuint v3))
ANT_GL_CORE_DECL(void, glUniform1uiv, (GLint location, GLsizei count, const GLuint *value))
ANT_GL_CORE_DECL(void, glUniform2uiv, (GLint location, GLsizei count, const GLuint *value))
ANT_GL_CORE_DECL(void, glUniform3uiv, (GLint location, GLsizei count, const GLuint *value))
ANT_GL_CORE_DECL(void, glUniform4uiv, (GLint location, GLsizei count, const GLuint *value))
ANT_GL_CORE_DECL(void, glTexParameterIiv, (GLenum target, GLenum pname, const GLint *params))
ANT_GL_CORE_DECL(void, glTexParameterIuiv, (GLenum target, GLenum pname, const GLuint *params))
ANT_GL_CORE_DECL(void, glGetTexParameterIiv, (GLenum target, GLenum pname, GLint *params))
ANT_GL_CORE_DECL(void, glGetTexParameterIuiv, (GLenum target, GLenum pname, GLuint *params))
ANT_GL_CORE_DECL(void, glClearBufferiv, (GLenum buffer, GLint drawbuffer, const GLint *value))
ANT_GL_CORE_DECL(void, glClearBufferuiv, (GLenum buffer, GLint drawbuffer, const GLuint *value))
ANT_GL_CORE_DECL(void, glClearBufferfv, (GLenum buffer, GLint drawbuffer, const GLfloat *value))
ANT_GL_CORE_DECL(void, glClearBufferfi, (GLenum buffer, GLint drawbuffer, GLfloat depth, GLint stencil))
ANT_GL_CORE_DECL(const GLubyte *, glGetStringi, (GLenum name, GLuint index))
// GL 3.1
ANT_GL_CORE_DECL(void, glDrawArraysInstanced, (GLenum mode, GLint first, GLsizei count, GLsizei primcount))
ANT_GL_CORE_DECL(void, glDrawElementsInstanced, (GLenum mode, GLsizei count, GLenum type, const GLvoid *indices, GLsizei primcount))
ANT_GL_CORE_DECL(void, glTexBuffer, (GLenum target, GLenum internalformat, GLuint buffer))
ANT_GL_CORE_DECL(void, glPrimitiveRestartIndex, (GLuint index))
// GL 3.2
//typedef int64_t GLint64;
//ANT_GL_CORE_DECL(void, glGetInteger64i_v, (GLenum target, GLuint index, GLint64 *data))
//ANT_GL_CORE_DECL(void, glGetBufferParameteri64v, (GLenum target, GLenum pname, GLint64 *params))
ANT_GL_CORE_DECL(void, glFramebufferTexture, (GLenum target, GLenum attachment, GLuint texture, GLint level))
*/
// GL_ARB_vertex_array_object
ANT_GL_CORE_DECL_NO_FORWARD(void, glBindVertexArray, (GLuint array))
ANT_GL_CORE_DECL_NO_FORWARD(void, glDeleteVertexArrays, (GLsizei n, const GLuint *arrays))
ANT_GL_CORE_DECL_NO_FORWARD(void, glGenVertexArrays, (GLsizei n, GLuint *arrays))
ANT_GL_CORE_DECL_NO_FORWARD(GLboolean, glIsVertexArray, (GLuint array))


#ifdef ANT_WINDOWS
ANT_GL_CORE_DECL(PROC, wglGetProcAddress, (LPCSTR))
#endif                                                                                                                                                                                                                                                                                                                                                

#ifndef GL_CLAMP_TO_EDGE
#   define GL_CLAMP_TO_EDGE     0x812F
#endif
#ifndef GL_COMPILE_STATUS
#   define GL_COMPILE_STATUS    0x8B81
#endif
#ifndef GL_INFO_LOG_LENGTH
#   define GL_INFO_LOG_LENGTH   0x8B84
#endif
#ifndef GL_LINK_STATUS
#   define GL_LINK_STATUS       0x8B82
#endif
#ifndef GL_ARRAY_BUFFER
#   define GL_ARRAY_BUFFER      0x8892
#endif
#ifndef GL_DYNAMIC_DRAW
#   define GL_DYNAMIC_DRAW      0x88E8
#endif
#ifndef GL_VERTEX_SHADER
#   define GL_VERTEX_SHADER     0x8B31
#endif
#ifndef GL_FRAGMENT_SHADER
#   define GL_FRAGMENT_SHADER   0x8B30
#endif
#ifndef GL_VERTEX_ARRAY_BINDING
#   define GL_VERTEX_ARRAY_BINDING  0x85B5
#endif
#ifndef GL_CURRENT_PROGRAM
#   define GL_CURRENT_PROGRAM   0x8B8D
#endif
#ifndef GL_ACTIVE_TEXTURE
#   define GL_ACTIVE_TEXTURE    0x84E0
#endif
#ifndef GL_TEXTURE0
#   define GL_TEXTURE0          0x84C0
#endif
#ifndef GL_BGRA
#   define GL_BGRA              0x80E1
#endif


#endif // !defined ANT_LOAD_OGL_CORE_INCLUDED
