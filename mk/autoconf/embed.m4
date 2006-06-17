# embed.m4                                                     -*- Autoconf -*-
#==============================================================================
# Copyright (C)2003,2005 by Eric Sunshine <sunshine@sunshineco.com>
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
# CS_META_INFO_EMBED([EMITTER], [GPL-OKAY])
#	Determine if plugin meta-information should be embedded or if it should
#	exist in a stand-alone .csplugin file, and check if necessary tools and
#	libraries are present.  Sets the shell variable
#	enable_meta_info_embedding to "yes" if the user requested embedding or
#	if it was enabled by default; otherwise sets it to "no".
#
#	If EMITTER is provided, then a subset of the following variables
#	(depending upon platform and availability) are recorded by invoking
#	CS_EMIT_BUILD_PROPERTY() with EMITTER.  As a convenience, if EMITTER is
#	the literal value "emit" or "yes", then CS_EMIT_BUILD_RESULT()'s
#	default emitter will be used.
#
#	EMBED_META := yes or no
#	EMBED_META.CFLAGS := compiler flags
#	EMBED_META.LFLAGS := linker flags
#	CMD.WINDRES := windres.exe
#	OBJCOPY.AVAILABLE := yes or no
#	CMD.OBJCOPY := objcopy.exe
#	LIBBFD.AVAILABLE := yes or no
#	LIBBFD.CFLAGS := libbfd compiler flags
#	LIBBFD.LFLAGS := libbfd linker flags
#	ELF.AVAILABLE := yes or no
#
#	In general, clients need only concern themselves with the various
#	EMBED_META-related variables. For building plugin modules, utilize
#	EMBED_META.CFLAGS when compiling, and EMBED_META.LFLAGS when linking.
#
#	On Unix, when CS' own ELF metadata reader can't be used (because the
#	necessary header file elf.h was not found) embedding is accomplished
#	via libbfd, which carries a GPL license. Projects which carry licenses
#	not compatible with GPL should consider carefully before enabling
#	embedding on Unix. If your project is GPL-compatible, then set GPL-OKAY
#	to "yes". This will indicate that it is safe to use libbfd if the ELF
#	reader can not be used.  If your project is not GPL-compatible, then
#	set it to "no" in order to disable embedding on Unix if the ELF reader
#	is not usable. (The user can still manually override the setting via
#	the --enable-meta-info-embedding option.)
#
# IMPLEMENTATION NOTES
#
#	Recent versions of Mingw supply libbfd and libiberty.  Since Crystal
#	Space uses native Win32 API for meta-information embedding on Windows,
#	we do not require these libraries on Windows.  More importantly, users
#	do not want to see these GPL-licensed libraries appear in the link
#	statement for plugin modules, thus we explicitly disable the libbfd
#	test on Windows.
#------------------------------------------------------------------------------
AC_DEFUN([CS_META_INFO_EMBED],
    [AC_REQUIRE([AC_CANONICAL_HOST])
    _CS_META_INFO_EMBED_ENABLE([$1], [$2])
    AS_IF([test $enable_meta_info_embedding = yes],
        [_CS_META_INFO_EMBED_TOOLS([$1])
        AS_IF([test $cs_header_elf_h = yes],
	    [CS_EMIT_BUILD_PROPERTY([ELF.AVAILABLE], [yes], [], [],
		CS_EMITTER_OPTIONAL([$1]))],
            [case $host_os in
	        mingw*|cygwin*) ;;
		*)
		    CS_CHECK_LIBBFD([$1],
			[CS_EMIT_BUILD_PROPERTY([EMBED_META.CFLAGS],
			    [$cs_cv_libbfd_ok_cflags], [+], [],
			    CS_EMITTER_OPTIONAL([$1]))
			CS_EMIT_BUILD_PROPERTY([EMBED_META.LFLAGS],
			    [$cs_cv_libbfd_ok_lflags $cs_cv_libbfd_ok_libs],
			    [+], [], CS_EMITTER_OPTIONAL([$1]))])
		    ;;
	    esac])])])


#------------------------------------------------------------------------------
# _CS_META_INFO_EMBED_ENABLE([EMITTER], [GPL-OKAY])
#	Helper for CS_META_INFO_EMBED which adds an
#	--enable-meta-info-embedding option to the configure script allowing
#	the user to control embedding.  Sets the shell variable
#	enable_meta_info_embedding to yes or no.
#
# IMPLEMENTATION NOTES
#
#	On Unix, embedding is enabled by default if elf.h is found and disabled
#	by default unless overridden via GPL-OKAY because libbfd carries a GPL
#	license which may be incompatible with a project's own license (such as
#	LGPL).
#------------------------------------------------------------------------------
AC_DEFUN([_CS_META_INFO_EMBED_ENABLE],
    [AC_REQUIRE([CS_CHECK_HOST])
    AC_CHECK_HEADERS([elf.h], [cs_header_elf_h=yes], [cs_header_elf_h=no])
    AC_MSG_CHECKING([whether to embed plugin meta-information])
    case $cs_host_target in
	unix) AS_IF([test $cs_header_elf_h = yes],
              [cs_embed_meta_info_default=yes],
              [cs_embed_meta_info_default=m4_ifval([$2],[$2],[no])]) ;;
	*) cs_embed_meta_info_default=yes ;;
    esac
    AC_ARG_ENABLE([meta-info-embedding],
	[AC_HELP_STRING([--enable-meta-info-embedding],
	    [store plugin meta-information directly inside plugin modules if
	    supported by platform; if disabled, meta-information is stored in
	    stand-alone .csplugin files; this option is enabled by default for
	    non-Unix platforms and on Unix platforms with ELF-format object
	    files; it is disabled by default on Unix platforms if ELF is not
	    available and the project uses a non-GPL-compatible license (such
	    as LGPL) since the non-ELF Unix embedding technology requires the
	    GPL-licensed libbfd library; if ELF is not available, enable this
	    option on Unix only if you are certain you want a GPL-licensed
	    library infecting your project])],
	[], [enable_meta_info_embedding=$cs_embed_meta_info_default])
    AC_MSG_RESULT([$enable_meta_info_embedding])
    CS_EMIT_BUILD_PROPERTY([EMBED_META], [$enable_meta_info_embedding],
	[], [], CS_EMITTER_OPTIONAL([$1]))])



#------------------------------------------------------------------------------
# _CS_META_INFO_EMBED_TOOLS([EMITTER])
#	Helper for CS_META_INFO_EMBED() which searches for tools required for
#	plugin meta-info embedding.
#------------------------------------------------------------------------------
AC_DEFUN([_CS_META_INFO_EMBED_TOOLS],
    [CS_CHECK_TOOLS([WINDRES], [windres])
    CS_EMIT_BUILD_PROPERTY([CMD.WINDRES], [$WINDRES], [], [],
	CS_EMITTER_OPTIONAL([$1]))

    CS_CHECK_TOOLS([OBJCOPY], [objcopy])
    AS_IF([test -n "$OBJCOPY"],
        [CS_EMIT_BUILD_PROPERTY([OBJCOPY.AVAILABLE], [yes], [], [],
	    CS_EMITTER_OPTIONAL([$1]))
        CS_EMIT_BUILD_PROPERTY([CMD.OBJCOPY], [$OBJCOPY], [], [],
	    CS_EMITTER_OPTIONAL([$1]))])])



#------------------------------------------------------------------------------
# CS_CHECK_LIBBFD([EMITTER], [ACTION-IF-FOUND], [ACTION-IF-NOT-FOUND])
#	Exhaustive check for a usable GPL-licensed libbfd, the Binary File
#	Descriptor library, a component of binutils, which allows low-level
#	manipulation of executable and object files.  If EMITTER is provided,
#	then the following variables are recorded by invoking
#	CS_EMIT_BUILD_PROPERTY() with EMITTER.  As a convenience, if EMITTER is
#	the literal value "emit" or "yes", then CS_EMIT_BUILD_RESULT()'s
#	default emitter will be used.
#
#	LIBBFD.AVAILABLE := yes or no
#	LIBBFD.CFLAGS := libbfd compiler flags
#	LIBBFD.LFLAGS := libbfd linker flags
#
#	The shell variable cs_cv_libbfd_ok is set to yes if a usable libbfd was
#	discovered, else no. If found, the additional shell variables
#	cs_cv_libbfd_ok_cflags, cs_cv_libbfd_ok_lflags, and
#	cs_cv_libbfd_ok_libs are also set.
#
# WARNING
#
#	libbfd carries a GPL license which is incompatible with the LGPL
#	license of Crystal Space. Do not use this library with projects under
#	less restrictive licenses, such as LGPL.
#
# IMPLEMENTATION NOTES
#
#	It seems that some platforms have two version of libiberty installed:
#	one from binutils and one from gcc.  The binutils version resides in
#	/usr/lib, whereas the gcc version resides in the gcc installation
#	directory.  The gcc version, by default, takes precedence at link time
#	over the binutils version.  Unfortunately, in broken cases, the gcc
#	version of libiberty is missing htab_create_alloc() which is required
#	by some libbfd functions.  The extensive secondary check of libbfd
#	catches this anomalous case of broken gcc libiberty.  It turns out that
#	it is possible to make the linker prefer the binutils version by
#	specifying -L/usr/lib, thus the extensive test attempts to do so in an
#	effort to resolve this unfortunate issue.
#------------------------------------------------------------------------------
AC_DEFUN([CS_CHECK_LIBBFD],
    [CS_CHECK_LIB_WITH([bfd],
	[AC_LANG_PROGRAM([[#include <bfd.h>]], [bfd_init();])],
	[], [], [], [], [], [], [-liberty])

    AS_IF([test $cs_cv_libbfd = yes],
	[CS_CHECK_BUILD([if libbfd is usable], [cs_cv_libbfd_ok],
	    [AC_LANG_PROGRAM([[#include <bfd.h>]],
		[bfd* p;
		asection* s;
		bfd_init();
		p = bfd_openr(0,0);
		bfd_check_format(p,bfd_object);
		bfd_get_section_by_name(p,0);
		bfd_section_size(p,s);
		bfd_get_section_contents(p,s,0,0,0);
		bfd_close(p);])],
	    [CS_CREATE_TUPLE() CS_CREATE_TUPLE([],[-L/usr/lib],[])],
	    [], [], [], [],
	    [$cs_cv_libbfd_cflags],
	    [$cs_cv_libbfd_lflags],
	    [$cs_cv_libbfd_libs])],
	[cs_cv_libbfd_ok=no])

    AS_IF([test $cs_cv_libbfd_ok = yes],
	[CS_EMIT_BUILD_RESULT([cs_cv_libbfd_ok], [LIBBFD],
	    CS_EMITTER_OPTIONAL([$1]))
	$2],
	[$3])])
