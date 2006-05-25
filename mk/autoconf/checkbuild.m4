# checkbuild.m4                                                -*- Autoconf -*-
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
# CS_SPLIT_TUPLE(TUPLE, OUTPUT-VARIABLES)
#	Split a build-tuple into its component parts.  A build tuple is
#	constructed by CS_CREATE_TUPLE() and is comprised of compiler flags,
#	linker flags, and library references.  OUTPUT-VARIABLES is a
#	comma-delimited list of shell variables which should receive the
#	extracted compiler flags, linker flags, and library references,
#	respectively.
#------------------------------------------------------------------------------
AC_DEFUN([CS_SPLIT_TUPLE],
    [CS_SPLIT([$1], [cs_dummy,$2], [@])
    m4_map([_CS_SPLIT_TUPLE], [$2])])

AC_DEFUN([_CS_SPLIT_TUPLE],
    [$1=`echo $$1 | sed 'y%@%:@% %'`
    ])



#------------------------------------------------------------------------------
# CS_CREATE_TUPLE([CFLAGS], [LFLAGS], [LIBS])
#	Construct a build-tuple which is comprised of compiler flags, linker
#	flags, and library references.  Build tuples are encoded so as to
#	preserve whitespace in each component.  This makes it possible for
#	macros (such as CS_BUILD_IFELSE) which employ build tuples to accept
#	whitespace-delimited lists of tuples, and for shell "for" statements to
#	iterate over tuple lists without compromising whitespace embedded
#	within individual flags or library references.
#------------------------------------------------------------------------------
AC_DEFUN([CS_CREATE_TUPLE], [`echo @$1@$2@$3 | sed 'y% %@%:@%'`])



#------------------------------------------------------------------------------
# CS_LANG_CFLAGS
#	Return the literal string CFLAGS if the current language is C.  Return
#	the literal string CXXFLAGS if the current language is C++.  Generic
#	compiler test macros which need to modify or save the compiler flags
#	can invoke this macro to get the name of the compiler flags environment
#	variable (either CFLAGS or CXXFLAGS) depending upon the current
#	language.  For example:
#		CS_LANG_CFLAGS="$CS_LANG_CFLAGS -Wall"
#	With C, this expands to:
#		CFLAGS="$CFLAGS -Wall"
#	With C++, it expands to:
#		CXXFLAGS="$CXXFLAGS -Wall"
#------------------------------------------------------------------------------
AC_DEFUN([CS_LANG_CFLAGS], [AC_LANG_CASE([C], [CFLAGS], [C++], [CXXFLAGS])])



#------------------------------------------------------------------------------
# CS_BUILD_IFELSE([PROGRAM], [FLAGS], [LANGUAGE], [ACTION-IF-BUILT],
#                 [ACTION-IF-NOT-BUILT], [OTHER-CFLAGS], [OTHER-LFLAGS],
#                 [OTHER-LIBS], [INHIBIT-OTHER-FLAGS], [ERROR-REGEX])
#	Try building a program using the supplied compiler flags, linker flags,
#	and library references.  PROGRAM is typically a program composed via
#	AC_LANG_PROGRAM().  PROGRAM may be omitted if you are interested only
#	in learning if the compiler or linker respects certain flags.  LANGUAGE
#	is typically either C or C++ and specifies which compiler to use for
#	the test.  If LANGUAGE is omitted, C is used.  FLAGS is a whitespace
#	delimited list of build tuples.  Tuples are created with
#	CS_CREATE_TUPLE() and are composed of up to three elements each.  The
#	first element represents compiler flags, the second linker flags, and
#	the third libraries used when linking the program.  Each tuple from
#	FLAGS is attempted in order.  If you want a build attempted with no
#	special flags prior to builds with specialized flags, create an empty
#	tuple with CS_CREATE_TUPLE() at the start of the FLAGS list.  If the
#	build is successful, then the shell variables cs_build_ok is set to
#	"yes", cs_build_cflags, cs_build_lflags, and cs_build_libs are set to
#	the tuple elements which resulted in the successful build, and
#	ACTION-IF-BUILT is invoked.  Upon successful build, no further tuples
#	are consulted.  If no tuple results in a successful build, then
#	cs_build_ok is set to "no" and ACTION-IF-NOT-BUILT is invoked.
#	OTHER-CFLAGS, OTHER-LFLAGS, and OTHER-LIBS specify additional compiler
#	flags, linker flags, and libraries which should be used with each tuple
#	build attempt.  Upon successful build, these additional flags are also
#	reflected in the variables cs_build_cflags, cs_build_lflags, and
#	cs_build_libs unless INHIBIT-OTHER-FLAGS is a non-empty string.  The
#	optional ERROR-REGEX places an additional constraint upon the build
#	check.  If specified, ERROR-REGEX, which is a standard `grep' regular
#	expression, is applied to output captured from the compiler and linker.
#	If ERROR-REGEX matches, then the build is deemed a failure, and
#	cs_build_ok is set to "no".  This facility is useful for broken build
#	tools which emit an error message yet still return success as a result.
#	In such cases, it should be possible to detect the failure by scanning
#	the tools' output.
#
# IMPLEMENTATION NOTES
#
#	In Autoconf 2.57 and earlier, AC_LINK_IFELSE() invokes AC_TRY_EVAL(),
#	which does not provide access to the captured output.  To work around
#	this limitation, we temporarily re-define AC_TRY_EVAL() as
#	_AC_EVAL_STDERR(), which leaves the captured output in conftest.err
#	(which we must also delete).  In Autoconf 2.58, however,
#	AC_LINK_IFELSE() instead already invokes _AC_EVAL_STDERR() on our
#	behalf, however we must be careful to apply ERROR-REGEX within the
#	invocation AC_LINK_IFELSE(), since AC_LINK_IFELSE() deletes
#	conftest.err before it returns.
#------------------------------------------------------------------------------
AC_DEFUN([CS_BUILD_IFELSE],
    [AC_LANG_PUSH(m4_default([$3],[C]))
    cs_cflags_save="$CS_LANG_CFLAGS"
    cs_lflags_save="$LDFLAGS"
    cs_libs_save="$LIBS"
    cs_build_ok=no
    m4_ifval([$10], [m4_pushdef([AC_TRY_EVAL], [_AC_EVAL_STDERR]($$[1]))])

    for cs_build_item in m4_default([$2],[CS_CREATE_TUPLE()])
    do
	CS_SPLIT_TUPLE(
	    [$cs_build_item],[cs_cflags_test,cs_lflags_test,cs_libs_test])
	CS_LANG_CFLAGS="$cs_cflags_test $6 $cs_cflags_save"
	LDFLAGS="$cs_lflags_test $7 $cs_lflags_save"
	LIBS="$cs_libs_test $8 $cs_libs_save"
	AC_LINK_IFELSE(m4_default([$1], [AC_LANG_PROGRAM([],[])]),
	    [m4_ifval([$10],
		[AS_IF([AC_TRY_COMMAND(
		    [grep "AS_ESCAPE([$10])" conftest.err >/dev/null 2>&1])],
		    [cs_build_ok=no], [cs_build_ok=yes])],
		[cs_build_ok=yes])])
	AS_IF([test $cs_build_ok = yes], [break])
    done

    m4_ifval([$10], [m4_popdef([AC_TRY_EVAL]) rm -f conftest.err])
    CS_LANG_CFLAGS=$cs_cflags_save
    LDFLAGS=$cs_lflags_save
    LIBS=$cs_libs_save
    AC_LANG_POP(m4_default([$3],[C]))

    AS_IF([test $cs_build_ok = yes],
	[cs_build_cflags=CS_TRIM([$cs_cflags_test[]m4_ifval([$9],[],[ $6])])
	cs_build_lflags=CS_TRIM([$cs_lflags_test[]m4_ifval([$9],[],[ $7])])
	cs_build_libs=CS_TRIM([$cs_libs_test[]m4_ifval([$9],[],[ $8])])
	$4],
	[$5])])



#------------------------------------------------------------------------------
# CS_CHECK_BUILD(MESSAGE, CACHE-VAR, [PROGRAM], [FLAGS], [LANGUAGE],
#                [ACTION-IF-BUILT], [ACTION-IF-NOT-BUILT], [IGNORE-CACHE],
#                [OTHER-CFLAGS], [OTHER-LFLAGS], [OTHER-LIBS],
#                [INHIBIT-OTHER-FLAGS], [ERROR-REGEX])
#	Like CS_BUILD_IFELSE() but also prints "checking" and result messages,
#	and optionally respects the cache.  Sets CACHE-VAR to "yes" upon
#	success, else "no" upon failure.  Additionally, sets CACHE-VAR_cflags,
#	CACHE-VAR_lflags, and CACHE-VAR_libs to the values which resulted in a
#	successful build.  If IGNORE-CACHE is "yes", then the cache variables
#	are ignored upon entry to this macro, however they are still set to
#	appropriate values upon exit.
#------------------------------------------------------------------------------
AC_DEFUN([CS_CHECK_BUILD],
    [AS_IF([test "$8" != yes],
	[AC_CACHE_CHECK([$1], [$2],
	    [CS_BUILD_IFELSE([$3], [$4], [$5],
		[$2=yes
		$2_cflags=$cs_build_cflags
		$2_lflags=$cs_build_lflags
		$2_libs=$cs_build_libs],
		[$2=no], [$9], [$10], [$11], [$12], [$13])])],
	[AC_MSG_CHECKING([$1])
	    CS_BUILD_IFELSE([$3], [$4], [$5],
		[$2=yes
		$2_cflags=$cs_build_cflags
		$2_lflags=$cs_build_lflags
		$2_libs=$cs_build_libs],
		[$2=no], [$9], [$10], [$11], [$12], [$13])
	    AC_MSG_RESULT([$$2])])
    AS_IF([test $$2 = yes], [$6],
	[$2_cflags=''
	$2_lflags=''
	$2_libs=''
	$7])])



#------------------------------------------------------------------------------
# CS_CHECK_BUILD_FLAGS(MESSAGE, CACHE-VAR, FLAGS, [LANGUAGE],
#                     [ACTION-IF-RECOGNIZED], [ACTION-IF-NOT-RECOGNIZED],
#                     [OTHER-CFLAGS], [OTHER-LFLAGS], [OTHER-LIBS],
#                     [ERROR-REGEX])
#	Like CS_CHECK_BUILD(), but checks only if the compiler or linker
#	recognizes a command-line option or options.  MESSAGE is the "checking"
#	message.  CACHE-VAR is the shell cache variable which receives the flag
#	or flags recognized by the compiler or linker.  FLAGS is a
#	whitespace-delimited list of build tuples created with
#	CS_CREATE_TUPLE().  Each tuple from FLAGS is attempted in order until
#	one is found which is recognized by the compiler.  After that, no
#	further flags are checked.  LANGUAGE is typically either C or C++ and
#	specifies which compiler to use for the test.  If LANGUAGE is omitted,
#	C is used.  If a command-line option is recognized, then CACHE-VAR is
#	set to the composite value of $cs_build_cflags, $cs_build_lflags, and
#	$cs_build_libs of the FLAGS element which succeeded (not including the
#	"other" flags) and ACTION-IF-RECOGNIZED is invoked.  If no options are
#	recognized, then CACHE-VAR is set to the empty string, and
#	ACTION-IF-NOT-RECOGNIZED is invoked. As a convenience, in case
#	comparing CACHE-VAR against the empty string to test for failure is
#	undesirable, a second variable named CACHE-VAR_ok is set to the literal
#	"no" upon failure, and to the same value as CACHE-VAR upon success.
#------------------------------------------------------------------------------
AC_DEFUN([CS_CHECK_BUILD_FLAGS],
    [AC_CACHE_CHECK([$1], [$2_ok],
	[CS_BUILD_IFELSE([], [$3], [$4],
	    [$2=CS_TRIM([$cs_build_cflags $cs_build_lflags $cs_build_libs])
	    $2_ok="$$2"],
	    [$2=''
	    $2_ok=no], [$7], [$8], [$9], [Y], [$10])])
    AS_IF([test "$$2_ok" != no], [$5], [$6])])
