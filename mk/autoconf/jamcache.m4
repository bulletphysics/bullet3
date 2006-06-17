# jamcache.m4                                                  -*- Autoconf -*-
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
# Text cache facility for Jam-style properties.  The cache is stored in
# the shell variable cs_jamfile_text.
#
# CS_JAMCONFIG_APPEND(TEXT)
#	Append text to the Jam text cache.  This is a cover for
#	CS_TEXT_CACHE_APPEND().
#
# CS_JAMCONFIG_PREPEND(TEXT)
#	Prepend text to the Jam text cache.  This is a cover for
#	CS_TEXT_CACHE_PREPEND().
#
# CS_JAMCONFIG_PROPERTY(KEY, VALUE, [APPEND], [UNCONDITIONAL])
#	Append a line of the form "KEY ?= VALUE" to the Jam text cache.  If the
#	APPEND argument is not the empty string, then VALUE is appended to the
#	existing value of KEY using the form "KEY += VALUE".  If the
#	UNCONDITIONAL argument is not empty, then the value of KEY is set
#	unconditionally "KEY = VALUE", rather than via "KEY ?= VALUE".  APPEND
#	takes precedence over UNCONDITIONAL.  Note that if VALUE references
#	other Jam variables, for example $(OBJS), then be sure to protect the
#	value with AS_ESCAPE().  For example:
#	CS_JAMCONFIG_PROPERTY([ALLOBJS], [AS_ESCAPE([$(OBJS) $(LIBOBJS)])])
#
# CS_JAMCONFIG_OUTPUT(FILENAME)
#	Instruct config.status to write the Jam text cache to the given
#	filename.  This is a cover for CS_TEXT_CACHE_OUTPUT().
#------------------------------------------------------------------------------
AC_DEFUN([CS_JAMCONFIG_APPEND],
    [CS_TEXT_CACHE_APPEND([cs_jamconfig_text], [$1])])
AC_DEFUN([CS_JAMCONFIG_PREPEND],
    [CS_TEXT_CACHE_PREPEND([cs_jamconfig_text], [$1])])
AC_DEFUN([CS_JAMCONFIG_PROPERTY],
    [CS_JAMCONFIG_APPEND(
	[$1 m4_ifval([$3], [+=], m4_ifval([$4], [=], [?=])) \"$2\" ;
])])
AC_DEFUN([CS_JAMCONFIG_OUTPUT],
    [CS_TEXT_CACHE_OUTPUT([cs_jamconfig_text], [$1])])
