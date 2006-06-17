# textcache.m4                                                 -*- Autoconf -*-
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
# Text cache facility.  These macros provide a way to incrementally store
# arbitrary text in a shell variable, and to write the saved text to a file.
#
# CS_TEXT_CACHE_APPEND(VARIABLE, TEXT)
#	Append text to the contents of the named shell variable.  If the text
#	contains references to shell variables (such as $foo), then those
#	references will be expanded.  If expansion is not desired, then protect
#	the text with AS_ESCAPE().
#
# CS_TEXT_CACHE_PREPEND(VARIABLE, TEXT)
#	Prepend text to the contents of the named shell variable.  If the text
#	contains references to shell variables (such as $foo), then those
#	references will be expanded.  If expansion is not desired, then protect
#	the text with AS_ESCAPE().
#
# CS_TEXT_CACHE_OUTPUT(VARIABLE, FILENAME)
#	Instruct config.status to write the contents of the named shell
#	variable to the given filename.  If the file resides in a directory,
#	the directory will be created, if necessary.  If the output file
#	already exists, and if the cached text is identical to the contents of
#	the existing file, then the existing file is left alone, thus its time
#	stamp remains unmolested.  This heuristic may help to minimize rebuilds
#	when the file is listed as a dependency in a makefile.
#
# *NOTE*
#	There is a bug in Autoconf 2.57 and probably all earlier 2.5x versions
#	which results in errors if AC_CONFIG_COMMANDS is invoked for a `tag'
#	which represents a file in a directory which does not yet exist.
#	Unfortunately, even invoking AS_MKDIR_P in the `cmd' portion of
#	AC_CONFIG_COMMANDS does not solve the problem because the generated
#	configure script attempts to access information about the directory
#	before AS_MKDIR_P has a chance to create it.  This forces us to invoke
#	AS_MKDIR_P in the third argument to AC_CONFIG_COMMANDS (the
#	`init-cmds') rather than the second (the `cmds').  This is undesirable
#	because it means that the directory will be created anytime
#	config.status is invoked (even for a simple --help), rather than being
#	created only when requested to output the text cache.  This bug was
#	submitted to the Autoconf GNATS database by Eric Sunshine as #228 on
#	27-Dec-2002.  It was fixed for Autoconf 2.58 on 26-Sep-2003.  The
#	official fix makes the assumption that `tag' always represents a file
#	(as opposed to some generic target), and creates the file's directory
#	is not present.
#------------------------------------------------------------------------------
AC_DEFUN([CS_TEXT_CACHE_APPEND], [$1="${$1}$2"])
AC_DEFUN([CS_TEXT_CACHE_PREPEND], [$1="$2${$1}"])
AC_DEFUN([CS_TEXT_CACHE_OUTPUT],
    [AC_CONFIG_COMMANDS([$2],
	[echo $ECHO_N "$$1$ECHO_C" > $tmp/tcache
	AS_IF([diff $2 $tmp/tcache >/dev/null 2>&1],
	    [AC_MSG_NOTICE([$2 is unchanged])],
	    [rm -f $2
	    cp $tmp/tcache $2])
	rm -f $tmp/tcache],
	[$1='$$1'
	cs_dir=`AS_DIRNAME([$2])`
	AS_ESCAPE(AS_MKDIR_P([$cs_dir]), [$`\])])])
