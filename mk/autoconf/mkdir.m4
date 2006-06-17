# mkdir.m4                                                     -*- Autoconf -*-
#==============================================================================
# Copyright (C)2003 by Eric Sunshine <sunshine@sunshineco.com>
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
# CS_CHECK_MKDIR
#	Determine how to create a directory and a directory tree. Sets the
#	shell variable MKDIR to the command which creates a directory, and
#	MKDIRS to the command which creates a directory tree. Invokes
#	AC_SUBST() for MKDIR and MKDIRS.
#
# IMPLEMENTATION NOTES
#	We need to know the exact commands, so that we can emit them, thus the
#	AS_MKDIR_P function is not what we want to use here since it does not
#	provide access to the commands (and might not even discover suitable
#	commands).  First try "mkdir -p", then try the older "mkdirs".
#	Finally, if the mkdir command failed to recognize -p, then it might
#	have created a directory named "-p", so clean up that bogus directory.
#------------------------------------------------------------------------------
AC_DEFUN([CS_CHECK_MKDIR],
    [AC_CACHE_CHECK([how to create a directory], [cs_cv_shell_mkdir],
	[cs_cv_shell_mkdir='mkdir'])
    AC_SUBST([MKDIR], [$cs_cv_shell_mkdir])

    AC_CACHE_CHECK([how to create a directory tree], [cs_cv_shell_mkdir_p],
	[if $cs_cv_shell_mkdir -p . 2>/dev/null; then
	    cs_cv_shell_mkdir_p='mkdir -p'
	elif mkdirs . 2>/dev/null; then
	    cs_cv_shell_mkdir_p='mkdirs'
	fi
	test -d ./-p && rmdir ./-p])
    AS_VAR_SET_IF([cs_cv_shell_mkdir_p],
	[AC_SUBST([MKDIRS], [$cs_cv_shell_mkdir_p])],
	[CS_MSG_ERROR([do not know how to create a directory tree])])])



#------------------------------------------------------------------------------
# Replacement for AS_MKDIR_P() from m4sugar/m4sh.m4 which fixes two problems
# which are present in Autoconf 2.57 and probably all earlier 2.5x versions.
# This bug, along with a patch, was submitted to the Autoconf GNATS database by
# Eric Sunshine as #227 on 17-Dec-2002.  The bogus "-p" directory bug was fixed
# for Autoconf 2.58 on 26-Sep-2003.  The "mkdirs" optimization was not accepted
# (since it is unnecessary; it's only an optimization).
#
# 1) Removes bogus "-p" directory which the stock AS_MKDIR_P() leaves laying
#    around in the working directory if the mkdir command does not recognize
#    the -p option.
# 2) Takes advantage of the older "mkdirs" program if it exists and if "mkdir
#    -p" does not work.
#------------------------------------------------------------------------------
m4_defun([_AS_MKDIR_P_PREPARE],
[if mkdir -p . 2>/dev/null; then
  as_mkdir_p='mkdir -p'
elif mkdirs . 2>/dev/null; then
  as_mkdir_p='mkdirs'
else
  as_mkdir_p=''
fi
test -d ./-p && rmdir ./-p
])# _AS_MKDIR_P_PREPARE

m4_define([AS_MKDIR_P],
[AS_REQUIRE([_$0_PREPARE])dnl
{ if test -n "$as_mkdir_p"; then
    $as_mkdir_p $1
  else
    as_dir=$1
    as_dirs=
    while test ! -d "$as_dir"; do
      as_dirs="$as_dir $as_dirs"
      as_dir=`AS_DIRNAME("$as_dir")`
    done
    test ! -n "$as_dirs" || mkdir $as_dirs
  fi || AS_ERROR([cannot create directory $1]); }
])# AS_MKDIR_P
