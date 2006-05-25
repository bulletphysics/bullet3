# qualify.m4                                                   -*- Autoconf -*-
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
# CS_SYMBOL_QUALIFIER(MESSAGE, CACHE-VAR, QUALIFIERS, [SYMBOL], [LANG],
#		      [ACTION-IF-ACCEPTED], [ACTION-IF-NOT-ACCEPTED])
#	Test if a symbol can be qualified by one of the elements of the
#	comma-separated list of QUALIFIERS.  Examples of qualifiers include
#	__attribute__((deprecated)), __declspec(dllimport), etc. MESSAGE is the
#	"checking" message. CACHE-VAR is the variable which receives the
#	qualifier which succeeded, or the the literal "no" if none were
#	accepted. SYMBOL is the symbol to which the qualifier should be
#	applied. If omitted, then SYMBOL defaults to "void f();". LANG is the
#	language of the test, typically "C" or "C++". It defaults to "C" if
#	omitted. ACTION-IF-ACCEPTED is invoked after CACHE-VAR is set if one of
#	the qualifiers is accepted, else ACTION-IF-NOT-ACCEPTED is invoked.
#------------------------------------------------------------------------------
AC_DEFUN([CS_SYMBOL_QUALIFIER],
    [AC_CACHE_CHECK([$1], [$2],
	[$2='no'
	m4_foreach([cs_symbol_qualifier], [$3],
	    [AS_IF([test "$$2" = no],
		[CS_BUILD_IFELSE(
		    [AC_LANG_PROGRAM(
			[cs_symbol_qualifier m4_default([$4],[void f()]);],
			[])],
		    [], [$5], [$2='cs_symbol_qualifier'], [$2='no'])])])])
    AS_IF([test $$2 != no], [$6], [$7])])
