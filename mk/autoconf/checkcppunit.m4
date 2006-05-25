# checkcppunit.m4                                              -*- Autoconf -*-
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
# CS_CHECK_CPPUNIT([EMITTER])
#	Check if CppUnit (http://cppunit.sourceforge.net/), the unit-testing
#	framework is available. The shell variable cs_cv_libcppunit is set to
#	"yes" if CppUnit is discovered, else "no".  If available, then the
#	variables cs_cv_libcppunit_cflags, cs_cv_libcppunit_lflags, and
#	cs_cv_libcppunit_libs are set. If EMITTER is provided, then
#	CS_EMIT_BUILD_RESULT() is invoked with EMITTER in order to record the
#	results in an output file. As a convenience, if EMITTER is the literal
#	value "emit" or "yes", then CS_EMIT_BUILD_RESULT()'s default emitter
#	will be used.
#------------------------------------------------------------------------------
AC_DEFUN([CS_CHECK_CPPUNIT],
    [CS_CHECK_LIB_WITH([cppunit],
	[AC_LANG_PROGRAM([[#include <cppunit/ui/text/TestRunner.h>]],
	    [CppUnit::TextUi::TestRunner r; r.run();])],
	[], [C++])
	
    AS_IF([test $cs_cv_libcppunit = yes],
	[CS_CHECK_BUILD([if cppunit is sufficiently recent],
	    [cs_cv_libcppunit_recent],
	    [AC_LANG_PROGRAM(
		[[#include <cppunit/BriefTestProgressListener.h>]], 
		[CppUnit::BriefTestProgressListener b; b.startTest(0);])],
	    [], [C++],
	    [CS_EMIT_BUILD_RESULT([cs_cv_libcppunit], [CPPUNIT],
		CS_EMITTER_OPTIONAL([$1]))], [], [],
	    [$cs_cv_libcppunit_cflags],
	    [$cs_cv_libcppunit_lflags],
	    [$cs_cv_libcppunit_libs])])])
