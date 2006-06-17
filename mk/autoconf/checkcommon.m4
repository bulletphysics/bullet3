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
# CS_CHECK_COMMON_TOOLS_LINK
#	Checks for common tools related to linking.
#------------------------------------------------------------------------------
AC_DEFUN([CS_CHECK_COMMON_TOOLS_LINK],
    [
    # The default RANLIB in Jambase is wrong on some platforms, and is also
    # unsuitable during cross-compilation, so we set the value unconditionally
    # (sixth argument of CS_EMIT_BUILD_PROPERTY).
    AC_PROG_RANLIB
    CS_EMIT_BUILD_PROPERTY([RANLIB], [$RANLIB], [], [], [], [Y])
    
    CS_CHECK_TOOLS([DLLTOOL], [dlltool])
    CS_EMIT_BUILD_PROPERTY([CMD.DLLTOOL], [$DLLTOOL])
    
    CS_CHECK_TOOLS([DLLWRAP], [dllwrap])
    CS_EMIT_BUILD_PROPERTY([CMD.DLLWRAP], [$DLLWRAP])
    
    CS_CHECK_TOOLS([WINDRES], [windres])
    CS_EMIT_BUILD_PROPERTY([CMD.WINDRES], [$WINDRES])
    
    CS_CHECK_TOOLS([STRINGS], [strings])
    CS_EMIT_BUILD_PROPERTY([CMD.STRINGS], [$STRINGS])

    CS_CHECK_TOOLS([OBJCOPY], [objcopy])
    CS_EMIT_BUILD_PROPERTY([CMD.OBJCOPY], [$OBJCOPY])
    
    CS_CHECK_LIBTOOL
    CS_EMIT_BUILD_PROPERTY([LIBTOOL], [$LIBTOOL])
    CS_EMIT_BUILD_PROPERTY([APPLE_LIBTOOL], [$APPLE_LIBTOOL])
    ])


#------------------------------------------------------------------------------
# CS_CHECK_COMMON_TOOLS_BASIC
#	Checks for basic tools for building things.
#------------------------------------------------------------------------------
AC_DEFUN([CS_CHECK_COMMON_TOOLS_BASIC],
    [CS_CHECK_MKDIR
    CS_EMIT_BUILD_PROPERTY([CMD.MKDIR], [$MKDIR])
    CS_EMIT_BUILD_PROPERTY([CMD.MKDIRS], [$MKDIRS])

    CS_CHECK_PROGS([INSTALL], [install])
    CS_EMIT_BUILD_PROPERTY([INSTALL], [$INSTALL])])


#------------------------------------------------------------------------------
# CS_CHECK_COMMON_TOOLS_DOC_TEXINFO
#	Checks for tools to generate documentation from texinfo files.
#------------------------------------------------------------------------------
AC_DEFUN([CS_CHECK_COMMON_TOOLS_DOC_TEXINFO],
    [CS_CHECK_PROGS([TEXI2DVI], [texi2dvi])
    CS_EMIT_BUILD_PROPERTY([CMD.TEXI2DVI], [$TEXI2DVI])

    CS_CHECK_PROGS([TEXI2PDF], [texi2pdf])
    CS_EMIT_BUILD_PROPERTY([CMD.TEXI2PDF], [$TEXI2PDF])

    CS_CHECK_PROGS([DVIPS], [dvips])
    CS_EMIT_BUILD_PROPERTY([CMD.DVIPS], [$DVIPS])

    CS_CHECK_PROGS([DVIPDF], [dvipdf])
    CS_EMIT_BUILD_PROPERTY([CMD.DVIPDF], [$DVIPDF])

    CS_CHECK_PROGS([MAKEINFO], [makeinfo])
    CS_EMIT_BUILD_PROPERTY([CMD.MAKEINFO], [$MAKEINFO])])


#------------------------------------------------------------------------------
# CS_CHECK_COMMON_TOOLS_DOC_DOXYGEN
#	Checks for tools to generate source documentation via doxygen.
#------------------------------------------------------------------------------
AC_DEFUN([CS_CHECK_COMMON_TOOLS_DOC_DOXYGEN],
    [CS_CHECK_PROGS([DOXYGEN], [doxygen])
    CS_EMIT_BUILD_PROPERTY([CMD.DOXYGEN], [$DOXYGEN])

    CS_CHECK_TOOLS([DOT], [dot])
    CS_EMIT_BUILD_PROPERTY([CMD.DOT], [$DOT])])


#------------------------------------------------------------------------------
# CS_CHECK_COMMON_LIBS
#       Check for typical required libraries (libm, libmx, libdl, libnsl).
#------------------------------------------------------------------------------
AC_DEFUN([CS_CHECK_COMMON_LIBS],
    [AC_LANG_PUSH([C])
    AC_CHECK_LIB([m], [pow], [cs_cv_libm_libs=-lm], [cs_cv_libm_libs=])
    AC_CHECK_LIB([m], [cosf], [cs_cv_libm_libs=-lm])
    AC_CHECK_LIB([mx], [cosf])
    AC_CHECK_LIB([dl], [dlopen], [cs_cv_libdl_libs=-ldl], [cs_cv_libdl_libs=])
    AC_CHECK_LIB([nsl], [gethostbyname])
    AC_LANG_POP([C])])
