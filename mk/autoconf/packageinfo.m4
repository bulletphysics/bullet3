#==============================================================================
# packageinfo.m4
#    Macros for setting general info on the package, such as name and version
#    numbers and propagate them to the generated make and Jam property files.
#
# Copyright (C)2003 by Matthias Braun <matze@braunis.de>
# Copyright (C)2003,2004 by Eric Sunshine <sunshine@sunshineco.com>
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

#------------------------------------------------------------------------------
# CS_PACKAGEINFO([LONGNAME], [COPYRIGHT, [HOMEPAGE])
#	Set additional information for the package.  Note that the version
#	number of your application should only contain numbers, because on
#	Windows you can only set numerical values in some of the file
#	properties (such as versioninfo .rc files).
#------------------------------------------------------------------------------
AC_DEFUN([CS_PACKAGEINFO],
    [PACKAGE_LONGNAME="[$1]"
    PACKAGE_COPYRIGHT="[$2]"
    PACKAGE_HOMEPAGE="[$3]"
])


#------------------------------------------------------------------------------
# CS_EMIT_PACKAGEINFO([EMITTER])
#	Emit extended package information using the provided EMITTER.  EMITTER
#	is a macro name, such as CS_JAMCONFIG_PROPERTY or CS_MAKEFILE_PROPERTY,
#	which performs the actual task of emitting the KEY/VALUE tuple.  If
#	EMITTER is omitted, CS_JAMCONFIG_PROPERTY is used.  For backward
#	compatibility, if EMITTER is the literal value "jam", then
#	CS_JAMCONFIG_PROPERTY is used; if it is "make", then
#	CS_MAKEFILE_PROPERTY is used; however use of these literal names is
#	highly discouraged.
#------------------------------------------------------------------------------
AC_DEFUN([CS_EMIT_PACKAGEINFO],
    [_CS_EMIT_PACKAGEINFO([$1], [PACKAGE_NAME], [$PACKAGE_NAME])
    _CS_EMIT_PACKAGEINFO([$1], [PACKAGE_VERSION], [$PACKAGE_VERSION])
    _CS_EMIT_PACKAGEINFO([$1], [PACKAGE_STRING], [$PACKAGE_STRING])
    _CS_EMIT_PACKAGEINFO([$1], [PACKAGE_BUGREPORT], [$PACKAGE_BUGREPORT])
    _CS_EMIT_PACKAGEINFO([$1], [PACKAGE_LONGNAME], [$PACKAGE_LONGNAME])
    _CS_EMIT_PACKAGEINFO([$1], [PACKAGE_HOMEPAGE], [$PACKAGE_HOMEPAGE])
    _CS_EMIT_PACKAGEINFO([$1], [PACKAGE_COPYRIGHT], [$PACKAGE_COPYRIGHT])
    for cs_veritem in m4_translit(AC_PACKAGE_VERSION, [.], [ ]); do
	_CS_EMIT_PACKAGEINFO([$1], [PACKAGE_VERSION_LIST], [$cs_veritem], [+])
    done
    ])

AC_DEFUN([_CS_EMIT_PACKAGEINFO],
    [m4_case([$1],
	[make], [CS_MAKEFILE_PROPERTY([$2], [$3], [$4])],
	[jam], [CS_JAMCONFIG_PROPERTY([$2], [$3], [$4])],
	[], [CS_JAMCONFIG_PROPERTY([$2], [$3], [$4])],
	[$1([$2], [$3], [$4])])])
