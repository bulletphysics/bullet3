#==============================================================================
# Copyright (C)2003-2006 by Eric Sunshine <sunshine@sunshineco.com>
#
#    This library is free software; you can redistribute it and/or modify it
#    under the terms of the GNU Library General Public License as published by
#    the Free Software Foundation; either version 2 of the License, or (at your
#    option) any later version.
#
#    This library is distributed in the hope that it will be useful, but
#    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
#    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library General Public
#    License for more details.
#
#    You should have received a copy of the GNU Library General Public License
#    along with this library; if not, write to the Free Software Foundation,
#    Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
#
#==============================================================================
AC_PREREQ([2.56])

#------------------------------------------------------------------------------
# CS_CHECK_OPENGL
#	Check for OpenGL.
#
# IMPLEMENTATION NOTES
#
# Some Mesa installations require pthread, so pthread flags are employed if
# available.
#
# The check for opengl32 needs to precede other checks because Cygwin users
# often have Mesa installed, and Mesa's OpenGL library is compiled without the
# __stdcall flags which results in link errors, whereas Microsoft's native
# opengl32 works fine.  Conversely, some Unix implementations have Wine
# installed (Windows emulation layer) which includes an opengl32.so library.
# We need to avoid detection of this library on Unix since it would cause an
# undesirable dependence upon Wine.
#
# Many OpenGL libraries on Unix already contain GLX, so there is no separate
# GLX library, thus we first check for GLX using the discovered OpenGL library
# before attempting to locate a separate GLX-specific library.  
#
# On MacOS/X, some users have XFree86 installed which creates a link from
# /usr/include/GL to /usr/X11R6/include/GL.  We want to ignore this directory
# and instead check for Apple's OpenGL.framework, if we are not cross-building
# for Darwin.  We accomplish this by placing the OpenGL.framework test ahead of
# the other tests.
#
# At least one user (Jorrit) has a strange installation in which inclusion of
# <windows.h> fails if an int32 is not present, thus we must take this into
# account.
#------------------------------------------------------------------------------
m4_define([cs_define_int32],
    [[#if !HAVE_TYPE_INT32
    typedef long int32;
    #endif
    ]])

# CS_GL_INCLUDE(CPP-MACRO,FALLBACK,HEADER)
AC_DEFUN([CS_GL_INCLUDE],
    [[#if HAVE_WINDOWS_H
    #if !HAVE_TYPE_INT32
    typedef long int32;
    #endif
    #include <windows.h>
    #endif
    #ifndef CS_HEADER_GLOBAL
    #define CS_HEADER_GLOBAL(X,Y) CS_HEADER_GLOBAL_COMPOSE(X,Y)
    #define CS_HEADER_GLOBAL_COMPOSE(X,Y) <X/Y>
    #endif
    #ifdef $1
    #include CS_HEADER_GLOBAL($1,$3)
    #else
    #include <$2/$3>
    #endif]])

AC_DEFUN([CS_CHECK_OPENGL],
    [AC_REQUIRE([CS_CHECK_HOST])
    AC_REQUIRE([CS_CHECK_COMMON_LIBS])
    AC_REQUIRE([CS_CHECK_PTHREAD])
    AC_REQUIRE([AC_PATH_X])
    AC_REQUIRE([AC_PATH_XTRA])
    AC_CHECK_TYPE([int32], [AC_DEFINE([HAVE_TYPE_INT32], [], 
	[Whether the int32 type is available])], [])
    AC_CHECK_HEADERS([windows.h], [], [], [cs_define_int32])
    
    # Apply plaform-specific flags if necessary.
    cs_gl_plat_cflags=''
    cs_gl_plat_lflags=''
    cs_gl_plat_libs=''
    AS_IF([test -n "$cs_cv_libm_cflags$cs_cv_libm_lflags$cs_cv_libm_libs"],
	[cs_gl_plat_cflags="$cs_cv_libm_cflags $cs_gl_plat_cflags"
	cs_gl_plat_lflags="$cs_cv_libm_lflags $cs_gl_plat_lflags"
	cs_gl_plat_libs="$cs_cv_libm_libs $cs_gl_plat_libs"])
    AS_IF([test $cs_cv_sys_pthread = yes],
	[cs_gl_plat_cflags="$cs_cv_sys_pthread_cflags $cs_gl_plat_cflags"
	cs_gl_plat_lflags="$cs_cv_sys_pthread_lflags $cs_gl_plat_lflags"
	cs_gl_plat_libs="$cs_cv_sys_pthread_libs $cs_gl_plat_libs"])
    AS_IF([test "$no_x" != yes],
	[cs_gl_plat_cflags="$X_CFLAGS $cs_gl_plat_cflags"
	cs_gl_plat_lflags="$cs_gl_plat_lflags"
	cs_gl_plat_libs="
	    $X_PRE_LIBS $X_LIBS -lX11 -lXext $X_EXTRA_LIBS $cs_gl_plat_libs"])

    # Mesa requested?
    AC_ARG_WITH([mesa], [AC_HELP_STRING([--with-mesa],
	    [use Mesa OpenGL library if available (default YES)])],
	    [], [with_mesa=yes])
    
    AS_IF([test $with_mesa != no],
	[cs_mesa_gl=CS_CREATE_TUPLE([],[],[-lMesaGL])])
    
    # MacOS/X or Darwin?
    AS_IF([test "x$cs_host_macosx" = "xyes"],
	[cs_osx_gl=CS_CREATE_TUPLE([-DCS_OPENGL_PATH=OpenGL],[],[-framework OpenGL])])

    # Windows?
    AS_IF([test $cs_host_family = windows],
	[cs_win32_gl=CS_CREATE_TUPLE([],[],[-lopengl32])])
    
    # Check for OpenGL.
    CS_CHECK_BUILD([for OpenGL], [cs_cv_libgl],
	[AC_LANG_PROGRAM([CS_GL_INCLUDE([CS_OPENGL_PATH],[GL],[gl.h])],[glEnd()])],
	[$cs_win32_gl \
	$cs_osx_gl \
	CS_CREATE_TUPLE([],[],[-lGL]) \
	CS_CREATE_TUPLE([],[],[-lgl]) \
	$cs_mesa_gl], [],
	[CS_EMIT_BUILD_RESULT([cs_cv_libgl], [GL])], [], [],
	[$cs_gl_plat_cflags], [$cs_gl_plat_lflags], [$cs_gl_plat_libs])])


#------------------------------------------------------------------------------
# CS_CHECK_GLU
#	Check for GLU.
#------------------------------------------------------------------------------
AC_DEFUN([CS_CHECK_GLU],
    [AC_REQUIRE([CS_CHECK_OPENGL])
    AS_IF([test $cs_cv_libgl = yes],
        [AS_IF([test $with_mesa != no],
	    [cs_mesa_glu=CS_CREATE_TUPLE([],[],[-lMesaGLU])])
	
	# MacOS/X or Darwin?
	AS_IF([test "x$cs_host_macosx" = "xyes"],
	    [cs_osx_glu=CS_CREATE_TUPLE([-DCS_GLU_PATH=OpenGL],[],[-framework OpenGL])])
	
	# Windows?
	AS_IF([test $cs_host_family = windows],
	    [cs_win32_glu=CS_CREATE_TUPLE([],[],[-lglu32])])
    
	# Check for GLU.
	CS_CHECK_BUILD([for GLU], [cs_cv_libglu],
	    [AC_LANG_PROGRAM(
		[CS_GL_INCLUDE([CS_GLU_PATH],[GL],[glu.h])], [gluNewQuadric()])],
	    [$cs_osx_glu \
	    CS_CREATE_TUPLE() \
	    $cs_win32_glu \
	    CS_CREATE_TUPLE([],[],[-lGLU]) \
	    CS_CREATE_TUPLE([],[],[-lglu]) \
	    $cs_mesa_glu], [],
	    [CS_EMIT_BUILD_RESULT([cs_cv_libglu], [GLU])], [], [],
	    [$cs_cv_libgl_cflags], [$cs_cv_libgl_lflags], [$cs_cv_libgl_libs])])])


#------------------------------------------------------------------------------
# CS_CHECK_GLX
#	Check for GLX.
#------------------------------------------------------------------------------
AC_DEFUN([CS_CHECK_GLX],
    [AC_REQUIRE([CS_CHECK_OPENGL])
    AS_IF([test $cs_cv_libgl = yes],
        [AS_IF([test $with_mesa != no],
            [cs_mesa_glx=CS_CREATE_TUPLE([],[],[-lMesaGLX])])
	
        # Check for GLX.
	AS_IF([test "$no_x" != yes],
	    [CS_CHECK_BUILD([for GLX], [cs_cv_libglx],
		[AC_LANG_PROGRAM([[#include <GL/glx.h>]], [glXWaitGL()])],
		[CS_CREATE_TUPLE() \
		CS_CREATE_TUPLE([],[],[-lGLX]) \
		CS_CREATE_TUPLE([],[],[-lglx]) \
		$cs_mesa_glx], [],
		[CS_EMIT_BUILD_RESULT([cs_cv_libglx], [GLX])], [], [],
		[$cs_cv_libgl_cflags], [$cs_cv_libgl_lflags], [$cs_cv_libgl_libs])])])])
    

#------------------------------------------------------------------------------
# CS_CHECK_GLXEXT([ACTION-IF-FOUND], [ACTION-IF-NOT-FOUND])
#	Check for GLX extensions.
#------------------------------------------------------------------------------
AC_DEFUN([CS_CHECK_GLXEXT],
    [AC_REQUIRE([CS_CHECK_GLX])
    AS_IF([test x$cs_cv_libglx = "xyes"],
	[# Check for GLX extensions.
	CS_CHECK_BUILD([for GLX extensions], [cs_cv_libglx_extensions],
	    [AC_LANG_PROGRAM(
		[[#define GLX_GLXEXT_PROTOTYPES
		#include <GL/glx.h>]],
		[glXGetProcAddressARB(0)])],
	    [CS_CREATE_TUPLE(
		[$cs_cv_libglx_cflags],
		[$cs_cv_libglx_lflags],
		[$cs_cv_libglx_libs])],
	    [], [$1], [$2])])])



#------------------------------------------------------------------------------
# CS_CHECK_GLUT
#	Check for GLUT.
#------------------------------------------------------------------------------
AC_DEFUN([CS_CHECK_GLUT],
    [AC_REQUIRE([CS_CHECK_GLU])
    AS_IF([test x$cs_cv_libglu = "xyes"],
        [# MacOS/X or Darwin?
	AS_IF([test "x$cs_host_macosx" = "xyes"],
	    [cs_osx_glut=CS_CREATE_TUPLE([-DCS_GLUT_PATH=GLUT],[],[-framework GLUT])])
	
	# Windows?
	AS_IF([test $cs_host_family = windows],
	    [cs_win32_glut=CS_CREATE_TUPLE([],[],[-lglut32])])
    
	# Check for GLUT.
	CS_CHECK_BUILD([for GLUT], [cs_cv_libglut],
	    [AC_LANG_PROGRAM(
		[CS_GL_INCLUDE([CS_GLUT_PATH],[GL],[glut.h])], [glutSwapBuffers()])],
	    [$cs_osx_glut \
	    CS_CREATE_TUPLE() \
	    $cs_win32_glut \
	    CS_CREATE_TUPLE([],[],[-lGLUT]) \
	    CS_CREATE_TUPLE([],[],[-lglut])], [],
	    [CS_EMIT_BUILD_RESULT([cs_cv_libglut], [GLUT])], [], [],
	    [$cs_cv_libgl_cflags $cs_cv_libglu_cflags], 
	    [$cs_cv_libgl_lflags $cs_cv_libglu_lflags], 
	    [$cs_cv_libgl_libs   $cs_cv_libglu_libs])])])

