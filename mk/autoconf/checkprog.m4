# checkprog.m4                                                 -*- Autoconf -*-
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
# cs_bin_paths_default
#	Comma delimited list of additional directories in which tools and
#	commands might be found.
#
# Present Cases:
#	/usr/local/bin -- Although a common location for executables, it is
#		now-and-then absent from the default PATH setting.
#	/sw/bin -- Fink, the MacOS/X manager of Unix packages, installs
#		executables here.
#------------------------------------------------------------------------------
m4_define([cs_bin_paths_default], [/usr/local/bin, /sw/bin])


#------------------------------------------------------------------------------
# CS_CHECK_PROG(VARIABLE, PROGRAM, VALUE-IF-FOUND, [VALUE-IF-NOT-FOUND],
#		[PATH], [REJECT])
#	Simple wrapper for AC_CHECK_PROG() which ensures that the search path
#	is augmented by the directories mentioned in cs_bin_paths_default.
#------------------------------------------------------------------------------
AC_DEFUN([CS_CHECK_PROG],
    [_CS_PROG_PATH_PREPARE
    AC_CHECK_PROG([$1], [$2], [$3], [$4],
	m4_ifval([$5], [_CS_PROG_CLIENT_PATH([$5])]), [$6])])


#------------------------------------------------------------------------------
# CS_CHECK_PROGS(VARIABLE, PROGRAMS, [VALUE-IF-NOT-FOUND], [PATH])
#	Simple wrapper for AC_CHECK_PROGS() which ensures that the search path
#	is augmented by the directories mentioned in cs_bin_paths_default.
#------------------------------------------------------------------------------
AC_DEFUN([CS_CHECK_PROGS],
    [_CS_PROG_PATH_PREPARE
    AC_CHECK_PROGS([$1], [$2], [$3],
	m4_ifval([$4], [_CS_PROG_CLIENT_PATH([$4])]))])


#------------------------------------------------------------------------------
# CS_CHECK_TOOL(VARIABLE, TOOL, [VALUE-IF-NOT-FOUND], [PATH])
#	Simple wrapper for AC_CHECK_TOOL() which ensures that the search path
#	is augmented by the directories mentioned in cs_bin_paths_default.
#------------------------------------------------------------------------------
AC_DEFUN([CS_CHECK_TOOL],
    [_CS_PROG_PATH_PREPARE
    AC_CHECK_TOOL([$1], [$2], [$3],
	m4_ifval([$4], [_CS_PROG_CLIENT_PATH([$4])]))])


#------------------------------------------------------------------------------
# CS_CHECK_TOOLS(VARIABLE, TOOLS, [VALUE-IF-NOT-FOUND], [PATH])
#	Simple wrapper for AC_CHECK_TOOLS() which ensures that the search path
#	is augmented by the directories mentioned in cs_bin_paths_default.
#------------------------------------------------------------------------------
AC_DEFUN([CS_CHECK_TOOLS],
    [_CS_PROG_PATH_PREPARE
    AC_CHECK_TOOLS([$1], [$2], [$3],
	m4_ifval([$4], [_CS_PROG_CLIENT_PATH([$4])]))])


#------------------------------------------------------------------------------
# CS_PATH_PROG(VARIABLE, PROGRAM, [VALUE-IF-NOT-FOUND], [PATH])
#	Simple wrapper for AC_PATH_PROG() which ensures that the search path
#	is augmented by the directories mentioned in cs_bin_paths_default.
#------------------------------------------------------------------------------
AC_DEFUN([CS_PATH_PROG],
    [_CS_PROG_PATH_PREPARE
    AC_PATH_PROG([$1], [$2], [$3],
	m4_ifval([$4], [_CS_PROG_CLIENT_PATH([$4])]))])


#------------------------------------------------------------------------------
# CS_PATH_PROGS(VARIABLE, PROGRAMS, [VALUE-IF-NOT-FOUND], [PATH])
#	Simple wrapper for AC_PATH_PROGS() which ensures that the search path
#	is augmented by the directories mentioned in cs_bin_paths_default.
#------------------------------------------------------------------------------
AC_DEFUN([CS_PATH_PROGS],
    [_CS_PROG_PATH_PREPARE
    AC_PATH_PROGS([$1], [$2], [$3],
	m4_ifval([$4], [_CS_PROG_CLIENT_PATH([$4])]))])


#------------------------------------------------------------------------------
# CS_PATH_TOOL(VARIABLE, TOOL, [VALUE-IF-NOT-FOUND], [PATH])
#	Simple wrapper for AC_PATH_TOOL() which ensures that the search path
#	is augmented by the directories mentioned in cs_bin_paths_default.
#------------------------------------------------------------------------------
AC_DEFUN([CS_PATH_TOOL],
    [_CS_PROG_PATH_PREPARE
    AC_PATH_TOOL([$1], [$2], [$3],
	m4_ifval([$4], [_CS_PROG_CLIENT_PATH([$4])]))])


#------------------------------------------------------------------------------
# _CS_PROG_PATH_PREPARE
#	Ensure that the PATH environment variable mentions the set of
#	directories listed in cs_bin_paths_default. These directories may not
#	appear by default in the typical PATH, yet they might be common
#	locations for tools and commands.
#------------------------------------------------------------------------------
AC_DEFUN([_CS_PROG_PATH_PREPARE],
    [AS_REQUIRE([_AS_PATH_SEPARATOR_PREPARE])
    AS_IF([test "$cs_prog_path_prepared" != yes],
	[cs_prog_path_prepared=yes
	PATH="$PATH[]m4_foreach([cs_bin_path], [cs_bin_paths_default],
	[$PATH_SEPARATOR[]cs_bin_path])"
	export PATH])])


#------------------------------------------------------------------------------
# _CS_PROG_CLIENT_PATH(CLIENT-PATH)
#	Given a client-supplied replacement for PATH, augment the list by
#	appending the locations mentioned in cs_bin_paths_default.
#------------------------------------------------------------------------------
AC_DEFUN([_CS_PROG_CLIENT_PATH],
    [AS_REQUIRE([_AS_PATH_SEPARATOR_PREPARE])dnl
    $1[]m4_foreach([cs_bin_path], [cs_bin_paths_default],
	[$PATH_SEPARATOR[]cs_bin_path])])
