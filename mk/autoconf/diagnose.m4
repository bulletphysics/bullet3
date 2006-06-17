# diagnose.m4                                                  -*- Autoconf -*-
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
# CS_MSG_ERROR(ERROR-DESCRIPTION, [EXIT-STATUS])
#	A convenience wrapper for AC_MSG_ERROR() which invokes AC_CACHE_SAVE()
#	before aborting the script.  Saving the cache should make subsequent
#	re-invocations of the configure script faster once the user has
#	corrected the problem(s) which caused the failure.
#------------------------------------------------------------------------------
AC_DEFUN([CS_MSG_ERROR],
    [AC_CACHE_SAVE
    AC_MSG_ERROR([$1], [$2])])
