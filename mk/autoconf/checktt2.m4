# checktt2.m4                                                  -*- Autoconf -*-
#==============================================================================
# Copyright (C)2004,2005 by Eric Sunshine <sunshine@sunshineco.com>
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
# CS_CHECK_TEMPLATE_TOOLKIT2([EMITTER])
#	Check if Template Toolkit 2 (http://www.tt2.org/) is available. The
#	shell variable cs_cv_perl_tt2 is set to "yes" if the package is
#	discovered, else "no". Also sets the shell variable TTREE to the name
#	path of the 'ttree' utility program and invokes AC_SUBST().  If EMITTER
#	is provided and the package was discovered, then
#	CS_EMIT_BUILD_PROPERTY() is invoked with EMITTER in order to record the
#	value of the TTREE variable in an output file. As a convenience, if
#	EMITTER is the literal value "emit" or "yes", then
#	CS_EMIT_BUILD_RESULT()'s default emitter will be used.
#------------------------------------------------------------------------------
AC_DEFUN([CS_CHECK_TEMPLATE_TOOLKIT2],
    [CS_CHECK_PROGS([PERL], [perl5 perl])
    AS_IF([test -n "$PERL"],
	[AC_CACHE_CHECK([for TemplateToolkit], [cs_cv_perl_tt2],
	    [AS_IF([AC_RUN_LOG(
		[$PERL -M'Template 2.11' -MTemplate::Plugin -e 0 1>&2])],
		[cs_cv_perl_tt2=yes],
		[cs_cv_perl_tt2=no])])
	CS_PATH_PROGS([TTREE], [ttree])
	AS_IF([test $cs_cv_perl_tt2 = yes && test -n "$TTREE"],
	    [CS_EMIT_BUILD_PROPERTY([TTREE], [$TTREE], [], [],
		CS_EMITTER_OPTIONAL([$1]))])])])
