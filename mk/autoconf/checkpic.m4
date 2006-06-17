# checkpic.m4                                                  -*- Autoconf -*-
#==============================================================================
# Copyright (C)2005 by Eric Sunshine <sunshine@sunshineco.com>
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
# CS_COMPILER_PIC([LANGUAGE], [CACHE-VAR], [ACTION-IF-FOUND],
#                 [ACTION-IF-NOT-FOUND])
#	Check if compiler can be instructed to produce
#	position-independent-code (PIC).  This feature is required by some
#	platforms when building plugin modules and shared libraries.  If
#	LANGUAGE is not provided, then `C' is assumed (other options include
#	`C++').  If CACHE-VAR is not provided, then it defaults to the name
#	"cs_cv_prog_compiler_pic".  If a PIC-enabling option (such as `-fPIC')
#	is discovered, then it is assigned to CACHE-VAR and ACTION-IF-FOUND is
#	invoked; otherwise the empty string is assigned to CACHE-VAR and
#	ACTION-IF-NOT-FOUND is invoked.
#
# IMPLEMENTATION NOTES
#
#	On some platforms (such as Windows), the -fPIC option is superfluous
#	and emits a warning "-fPIC ignored for target (all code is position
#	independent)", despite the fact that the compiler accepts the option
#	and returns a success code. We want to re-interpret the warning as a
#	failure in order to avoid unnecessary compiler diagnostics in case the
#	client inserts the result of this check into CFLAGS, for instance. We
#	do so by attempting to promote warnings to errors using the result of
#	CS_COMPILER_ERRORS(). As an extra safe-guard, we also scan the compiler
#	output for an appropriate diagnostic because some gcc warnings fail to
#	promote to error status despite use of -Werror.
#------------------------------------------------------------------------------
AC_DEFUN([CS_COMPILER_PIC],
    [CS_COMPILER_ERRORS([$1],
	[m4_default([$2_werror],[cs_cv_prog_compiler_pic_werror])])
    CS_CHECK_BUILD_FLAGS(
	[how to enable m4_default([$1],[C]) PIC generation],
	[m4_default([$2],[cs_cv_prog_compiler_pic])],
	[CS_CREATE_TUPLE([-fPIC])], [$1], [$3], [$4],
	[m4_default([$$2_werror],[$cs_cv_prog_compiler_pic_werror])], [], [],
	[fPIC])])

# Backward-compatiblity alias.
AC_DEFUN([CS_CHECK_COMPILER_PIC], [CS_COMPILER_PIC([$1],[$2],[$3],[$4])])
