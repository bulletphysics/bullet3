# path.m4                                                      -*- Autoconf -*-
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
# CS_PATH_NORMALIZE(STRING)
#	Normalize a pathname at run-time by transliterating Windows/DOS
#	backslashes to forward slashes.  Also collapses whitespace.
#------------------------------------------------------------------------------
AC_DEFUN([CS_PATH_NORMALIZE],
[`echo "x$1" | tr '\\\\' '/' | sed 's/^x//;s/   */ /g;s/^ //;s/ $//'`])


#------------------------------------------------------------------------------
# CS_RUN_PATH_NORMALIZE(COMMAND)
#	Normalize the pathname emitted by COMMAND by transliterating
#	Windows/DOS backslashes to forward slashes.  Also collapses whitespace.
#------------------------------------------------------------------------------
AC_DEFUN([CS_RUN_PATH_NORMALIZE],
[`AC_RUN_LOG([$1]) | tr '\\\\' '/' | sed 's/^x//;s/   */ /g;s/^ //;s/ $//'`])
