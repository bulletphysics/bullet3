# emit.m4                                                      -*- Autoconf -*-
#==============================================================================
# Copyright (C)2003-2005 by Eric Sunshine <sunshine@sunshineco.com>
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
# CS_EMIT_BUILD_PROPERTY(KEY, VALUE, [APPEND], [EMPTY-OKAY], [EMITTER],
#                        [UNCONDITIONAL])
#	A utility function which invokes an emitter to record the KEY/VALUE
#	tuple if VALUE is not the empty string (after leading and trailing
#	whitespace is stripped). If EMPTY-OKAY is not an empty string, then the
#	property is emitted even if VALUE is empty; that is, it is emitted
#	unconditionally.  If APPEND is the empty string, then the emitter sets
#	the key's value directly (though it may be overridden by the
#	environment), otherwise the emitter appends VALUE to the existing value
#	of the key.  EMITTER is a macro name, such as CS_JAMCONFIG_PROPERTY or
#	CS_MAKEFILE_PROPERTY, which performs the actual task of emitting the
#	KEY/VALUE tuple; it should also accept APPEND as an optional third
#	argument. If EMITTER is omitted, CS_JAMCONFIG_PROPERTY is used.  Some
#	emitters accept an optional fourth argument, UNCONDITIONAL, which
#	instructs it to set KEY's value unconditionally, even if KEY already
#	had been assigned a value via some other mechanism (such as imported
#	from the environment, or from Jambase, in the case of
#	CS_JAMCONFIG_PROPERTY).
#------------------------------------------------------------------------------
AC_DEFUN([CS_EMIT_BUILD_PROPERTY],
    [cs_build_prop_val="$2"
    cs_build_prop_val=CS_TRIM([$cs_build_prop_val])
    m4_ifval([$4],
	[CS_JAMCONFIG_PROPERTY([$1], [$cs_build_prop_val], [$3])],
	AS_IF([test -n "$cs_build_prop_val"],
	    [m4_default([$5],[CS_JAMCONFIG_PROPERTY])(
		[$1], [$cs_build_prop_val], [$3], [$6])]))])



#------------------------------------------------------------------------------
# CS_EMIT_BUILD_RESULT(CACHE-VAR, PREFIX, [EMITTER])
#	Record the results of CS_CHECK_BUILD() or CS_CHECK_LIB_WITH() via some
#	emitter.  If CACHE-VAR indicates that the build succeeded, then the
#	following properties are emitted:
#
#	PREFIX.AVAILABLE = yes
#	PREFIX.CFLAGS = $CACHE-VAR_cflags
#	PREFIX.LFLAGS = $CACHE-VAR_lflags $CACHE-VAR_libs
#
#	EMITTER is a macro name, such as CS_JAMCONFIG_PROPERTY or
#	CS_MAKEFILE_PROPERTY, which performs the actual task of emitting the
#	KEY/VALUE tuple. If EMITTER is omitted, CS_JAMCONFIG_PROPERTY is used.
#------------------------------------------------------------------------------
AC_DEFUN([CS_EMIT_BUILD_RESULT],
    [AS_IF([test "$$1" = yes],
	[CS_EMIT_BUILD_PROPERTY([$2.AVAILABLE], [yes], [], [], [$3])
	CS_EMIT_BUILD_PROPERTY([$2.CFLAGS], [$$1_cflags], [], [], [$3])
	CS_EMIT_BUILD_PROPERTY([$2.LFLAGS], [$$1_lflags $$1_libs],
	    [], [], [$3])])])



#------------------------------------------------------------------------------
# CS_EMIT_BUILD_FLAGS(MESSAGE, CACHE-VAR, FLAGS, [LANGUAGE], EMITTER-KEY,
#                     [APPEND], [ACTION-IF-RECOGNIZED],
#                     [ACTION-IF-NOT-RECOGNIZED], [EMITTER])
#	A convenience wrapper for CS_CHECK_BUILD_FLAGS() which also records the
#	results via CS_EMIT_BUILD_PROPERTY().  Checks if the compiler or linker
#	recognizes a command-line option.  MESSAGE is the "checking" message.
#	CACHE-VAR is the shell cache variable which receives the flag
#	recognized by the compiler or linker, or "no" if the flag was not
#	recognized.  FLAGS is a whitespace- delimited list of build tuples
#	created with CS_CREATE_TUPLE().  Each tuple from FLAGS is attempted in
#	order until one is found which is recognized by the compiler.  After
#	that, no further flags are checked.  LANGUAGE is typically either C or
#	C++ and specifies which compiler to use for the test.  If LANGUAGE is
#	omitted, C is used.  EMITTER-KEY is the name to pass as the emitter's
#	"key" argument if a usable flag is encountered.  If APPEND is not the
#	empty string, then the discovered flag is appended to the existing
#	value of the EMITTER-KEY.  If the command-line option was recognized,
#	then ACTION-IF-RECOGNIZED is invoked, otherwise
#	ACTION-IF-NOT-RECOGNIZED is invoked.  EMITTER is a macro name, such as
#	CS_JAMCONFIG_PROPERTY or CS_MAKEFILE_PROPERTY, which performs the
#	actual task of emitting the KEY/VALUE tuple; it should also accept
#	APPEND as an optional third argument. If EMITTER is omitted,
#	CS_JAMCONFIG_PROPERTY is used.
#------------------------------------------------------------------------------
AC_DEFUN([CS_EMIT_BUILD_FLAGS],
    [CS_CHECK_BUILD_FLAGS([$1], [$2], [$3], [$4],
	[CS_EMIT_BUILD_PROPERTY([$5], [$$2], [$6], [], [$9])
	    $7],
	[$8])])



#------------------------------------------------------------------------------
# CS_EMITTER_OPTIONAL([EMITTER])
#	The CS_EMIT_FOO() macros optionally accept an emitter. If no emitter is
#	supplied to those macros, then a default emitter is chosen.  Other
#	macros, however, which perform testing and optionally emit the results
#	may wish to interpret an omitted EMITTER as a request not to emit the
#	results. CS_EMITTER_OPTIONAL() is a convenience macro to help in these
#	cases. It should be passed to one of the CS_EMIT_FOO() macros in place
#	of the literal EMITTER argument. It functions by re-interpretating
#	EMITTER as follows:
#
#	- If EMITTER is omitted, then CS_NULL_EMITTER is returned, effectively
#	  disabling output by the CS_EMIT_FOO() macro.
#	- If EMITTER is the literal string "emit" or "yes", then it returns an
#	  empty string, which signals to the CS_EMIT_FOO() macro that is should
#	  use its default emitter.
#	- Any other value for EMITTER is passed along as-is to the
#	  CS_EMIT_FOO() macro.
#------------------------------------------------------------------------------
AC_DEFUN([CS_EMITTER_OPTIONAL],
    [m4_case([$1],
	[], [[CS_NULL_EMITTER]],
	[emit], [],
	[yes], [],
	[[$1]])])



#------------------------------------------------------------------------------
# CS_NULL_EMITTER(KEY, VALUE, [APPEND])
#	A do-nothing emitter suitable for use as the EMITTER argument of one of
#	the CS_EMIT_FOO() macros.
#------------------------------------------------------------------------------
AC_DEFUN([CS_NULL_EMITTER], [:
])



#------------------------------------------------------------------------------
# CS_SUBST_EMITTER(KEY, VALUE, [APPEND])
#	An emitter wrapped around AC_SUBST(). Invokes
#	AC_SUBST(AS_TR_SH(KEY),VALUE).  The APPEND argument is ignored.
#	Suitable for use as the EMITTER argument of one of the CS_EMIT_FOO()
#	macros.  The call to AS_TR_SH() ensures that KEY is transformed into a
#	valid shell variable. For instance, if a macro attempts to emit
#	MYLIB.CFLAGS and MYLIB.LFLAGS via CS_SUBST_EMITTER(), then the names
#	will be transformed to MYLIB_CFLAGS and MYLIB_LFLAGS, respectively, for
#	the invocation of AC_SUBST().
#------------------------------------------------------------------------------
AC_DEFUN([CS_SUBST_EMITTER], [AC_SUBST(AS_TR_SH([$1]),[$2])])



#------------------------------------------------------------------------------
# CS_DEFINE_EMITTER(KEY, VALUE, [APPEND])
#	An emitter wrapped around AC_DEFINE_UNQUOTED(). Invokes
#	AC_DEFINE_UNQUOTED(AS_TR_CPP(KEY),VALUE).  The APPEND argument is
#	ignored.  Suitable for use as the EMITTER argument of one of the
#	CS_EMIT_FOO() macros. The call to AS_TR_CPP() ensures that KEY is a
#	well-formed token for the C-preprocessor.
#------------------------------------------------------------------------------
AC_DEFUN([CS_DEFINE_EMITTER],
    [AC_DEFINE_UNQUOTED(AS_TR_CPP([$1]),[$2],
	[Define when feature is available])])
