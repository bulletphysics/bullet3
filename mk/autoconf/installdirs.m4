#-----------------------------------------------------------------------------
# installdirs.m4 (c) Matze Braun <matze@braunis.de>
# Macro for emitting the installation paths gathered by Autoconf.
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
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
# CS_OUTPUT_INSTALLDIRS([EMITTER], [RAW-BACKSLASHES])
#   Emit installation directories collected by Autoconf.  EMITTER is a macro
#   name, such as CS_JAMCONFIG_PROPERTY or CS_MAKEFILE_PROPERTY, which performs
#   the actual task of emitting the KEY/VALUE tuple.  If EMITTER is omitted,
#   CS_JAMCONFIG_PROPERTY is used.  If RAW-BACKSLASHES is not provided, then
#   backslashes in emitted values are each escaped with an additional
#   backslash. If RAW-BACKSLASHES is not the null value, then backslashes are
#   emitted raw.  The following properties are emitted:
#
#       prefix
#       exec_prefix
#       bindir
#       sbindir
#       libexecdir
#       datadir
#       sysconfdir
#       sharedstatedir
#       localstatedir
#       libdir
#       includedir
#       oldincludedir
#       infodir
#       mandir
#-----------------------------------------------------------------------------
AC_DEFUN([CS_OUTPUT_INSTALLDIRS],[
# Handle the case when no prefix is given, and the special case when a path
# contains more than 2 slashes, these paths seem to be correct but Jam fails
# on them.
AS_IF([test $prefix = NONE],
    [cs_install_prefix="$ac_default_prefix"],
    [cs_install_prefix=`echo "$prefix" | sed -e 's:///*:/:g'`])
AS_IF([test $exec_prefix = NONE],
    [cs_install_exec_prefix="AS_ESCAPE([$(prefix)])"],
    [cs_install_exec_prefix=`echo "$exec_prefix" | sed -e 's:///*:/:g'`])

_CS_OUTPUT_INSTALL_DIRS([$1], [prefix],
    [CS_PREPARE_INSTALLPATH([$cs_install_prefix], [$2])])
_CS_OUTPUT_INSTALL_DIRS([$1], [exec_prefix],
    [CS_PREPARE_INSTALLPATH([$cs_install_exec_prefix], [$2])])
_CS_OUTPUT_INSTALL_DIRS([$1], [bindir],
    [CS_PREPARE_INSTALLPATH([$bindir], [$2])])
_CS_OUTPUT_INSTALL_DIRS([$1], [sbindir],
    [CS_PREPARE_INSTALLPATH([$sbindir], [$2])])
_CS_OUTPUT_INSTALL_DIRS([$1], [libexecdir],
    [CS_PREPARE_INSTALLPATH([$libexecdir], [$2])])
_CS_OUTPUT_INSTALL_DIRS([$1], [datadir],
    [CS_PREPARE_INSTALLPATH([$datadir], [$2])])
_CS_OUTPUT_INSTALL_DIRS([$1], [sysconfdir], 
    [CS_PREPARE_INSTALLPATH([$sysconfdir], [$2])])
_CS_OUTPUT_INSTALL_DIRS([$1], [sharedstatedir], 
    [CS_PREPARE_INSTALLPATH([$sharedstatedir], [$2])])
_CS_OUTPUT_INSTALL_DIRS([$1], [localstatedir], 
    [CS_PREPARE_INSTALLPATH([$localstatedir], [$2])])
_CS_OUTPUT_INSTALL_DIRS([$1], [libdir],
    [CS_PREPARE_INSTALLPATH([$libdir], [$2])])
_CS_OUTPUT_INSTALL_DIRS([$1], [includedir], 
    [CS_PREPARE_INSTALLPATH([$includedir], [$2])])
_CS_OUTPUT_INSTALL_DIRS([$1], [oldincludedir], 
    [CS_PREPARE_INSTALLPATH([$oldincludedir], [$2])])
_CS_OUTPUT_INSTALL_DIRS([$1], [infodir],
    [CS_PREPARE_INSTALLPATH([$infodir], [$2])])
_CS_OUTPUT_INSTALL_DIRS([$1], [mandir],
    [CS_PREPARE_INSTALLPATH([$mandir], [$2])])
])

AC_DEFUN([_CS_OUTPUT_INSTALL_DIRS],
    [m4_default([$1], [CS_JAMCONFIG_PROPERTY])([$2], [$3])])


#-----------------------------------------------------------------------------
# CS_PREPARE_INSTALLPATH(VALUE, [RAW-BACKSLASHES])
#   Transform variable references of the form ${bla} to $(bla) in VALUE and
#   correctly quotes backslashes.  This is needed if you need to emit some of
#   the paths from Autoconf. RAW-BACKSLASHES has the same meaning as in
#   CS_OUTPUT_INSTALLDIRS.
#-----------------------------------------------------------------------------
AC_DEFUN([CS_PREPARE_INSTALLPATH],
[`echo "$1" | sed 's/\${\([[a-zA-Z_][a-zA-Z_]]*\)}/$(\1)/g;m4_ifval([$2],
    [s/\\/\\\\/g], [s/\\\\/\\\\\\\\/g])'`])
