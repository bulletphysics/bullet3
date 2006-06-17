# split.m4                                                     -*- Autoconf -*-
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
# CS_SPLIT(LINE, [OUTPUT-VARIABLES], [DELIMITER], [FILLER])
#	Split LINE into individual tokens.  Tokens are delimited by DELIMITER,
#	which is the space character if omitted.  OUTPUT-VARIABLES is a
#	comma-delimited list of shell variables which should receive the
#	extracted tokens.  If there are too few tokens to fill the output
#	variables, then the excess variables will be assigned the empty string.
#	If there are too few output variables, then the excess tokens will be
#	ignored.  If OUTPUT-VARIABLES is omitted, then the split tokens will be
#	assigned to the shell meta-variables $1, $2, $3, etc.  When
#	OUTPUT-VARIABLES is omitted, FILLER is assigned to meta-variables in
#	cases where DELIMITER delimits a zero-length token.  FILLER defaults
#	to "filler".  For example, if DELIMITER is "+" and OUTPUT-VARIABLES is
#	omitted, given the line "one++three", $1 will be "one", $2 will be
#	"filler", and $3 will be "three".
#------------------------------------------------------------------------------
AC_DEFUN([CS_SPLIT],
    [m4_define([cs_split_filler], m4_default([$4],[filler]))
    set cs_split_filler `echo "$1" | awk 'BEGIN { FS="m4_default([$3],[ ])" }
	{ for (i=1; i <= NF; ++i)
	    { if ($i == "") print "cs_split_filler"; else print $i } }'`
    shift
    m4_map([_CS_SPLIT], [$2])])

AC_DEFUN([_CS_SPLIT],
    [AS_IF([test $[@%:@] -eq 0], [$1=''],
	[AS_IF([test "$[1]" = cs_split_filler], [$1=''], [$1=$[1]])
	shift])])
