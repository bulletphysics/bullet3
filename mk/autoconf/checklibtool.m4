# checklibtool.m4                                              -*- Autoconf -*-
#==============================================================================
# Copyright (C)2004 by Eric Sunshine <sunshine@sunshineco.com>
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
# CS_CHECK_LIBTOOL
#	Find and identify the various implementations of libtool.  In
#	particular, this macro is aware of GNU libtool and Apple's libtool
#	(which serves a completely different purpose).  On MacOS/X, GNU libtool
#	is typically named glibtool, however a user might also use Fink to
#	install the unadorned libtool; and the Fink-installed version might
#	shadow Apple's own libtool if it appears in the PATH before the Apple
#	tool. This macro jumps through the necessary hoops to distinguish and
#	locate the various implementations. Sets the shell variable LIBTOOL to
#	the located GNU libtool (if any), and APPLE_LIBTOOL to the located
#	Apple libtool. Invokes AC_SUBST() for LIBTOOL and APPLE_LIBTOOL.
#------------------------------------------------------------------------------
AC_DEFUN([CS_CHECK_LIBTOOL],
[# GNU: Search for libtool before glibtool since Fink version is likely newer.
m4_define([cs_lt_path_gnu],
    [/sw/bin$PATH_SEPARATOR/usr/local/bin$PATH_SEPARATOR$PATH])
AS_IF([test -z "$LIBTOOL"],
    [CS_CHECK_TOOLS([LIBTOOL_TEST], [libtool glibtool gnulibtool], [],
	[cs_lt_path_gnu])
    AS_IF([test -n "$LIBTOOL_TEST"],
	[CS_PATH_PROG([LIBTOOL_PATH], [$LIBTOOL_TEST], [], [cs_lt_path_gnu])
	CS_LIBTOOL_CLASSIFY([$LIBTOOL_PATH],
	    [LIBTOOL="$LIBTOOL_PATH"],
	    [AS_IF([test -z "$APPLE_LIBTOOL"], [APPLE_LIBTOOL="$LIBTOOL_PATH"])
	    CS_CHECK_TOOLS([LIBTOOL], [glibtool gnulibtool])])])])
AC_SUBST([LIBTOOL])

# Apple: Ensure that Apple libtool will be found before GNU libtool from Fink.
m4_define([cs_lt_path_apple],[/bin$PATH_SEPARATOR/usr/bin$PATH_SEPARATOR$PATH])
AS_IF([test -z "$APPLE_LIBTOOL"],
    [CS_PATH_PROG([CS_LT_APPLE], [libtool], [], [cs_lt_path_apple])
    CS_LIBTOOL_CLASSIFY([$CS_LT_APPLE], [],
	[APPLE_LIBTOOL="$CS_LT_APPLE"])])
AC_SUBST([APPLE_LIBTOOL])])

AC_DEFUN([CS_LIBTOOL_CLASSIFY],
    [AS_IF([test -n "$1"],
	[AC_MSG_CHECKING([classification of $1])
	CS_LIBTOOL_GNU_IFELSE([$1],
	    [AC_MSG_RESULT([gnu])
	    $2],
	    [AC_MSG_RESULT([apple])
	    $3])])])

AC_DEFUN([CS_LIBTOOL_GNU_IFELSE],
    [AS_IF([AC_RUN_LOG([$1 --version 1>&2])], [$2], [$3])])
