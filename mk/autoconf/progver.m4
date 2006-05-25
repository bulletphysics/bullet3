###############################################################################
# progver.m4
# Written by Norman Kramer <norman@users.sourceforge.net>
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
###############################################################################
#
# From the input pattern we create regular expressions we send through sed
# to extract the version information from the standard input to sed.
# Then we extract from the resulting version string subparts.
# The same happens with the supplied version string. It too is split into its
# subparts according to the pattern.
# Then the subparts from the gathered version string and the supplied one are
# compared.
#
# How does the pattern look like ?
# It is a sequence of 9s and _s and separators.
# 9 denotes a non empty sequence of digits.
# _ denotes a non empty sequence of characters from the class [a-zA-Z].
# | everything behind is optional
# Everything else is treated as a separator.
# Consecutive 9s and _s are compressed to contain only one of each type.
# For instance "99_.9.__abc9_" will become "9_.9._abc9_".
#
# How we find the parts we compare ?
# From this transformed string we yield the parts we will later compare.
# We break up the string as follows:
# Any sequence of separators represent one breakup. Additional breakups are
# placed behind every 9 and _ .
# So the example from above will give:
#
# "99_.9.__abc9_"  ===compress==> "9_.9._abc9_" ===breakup==> "9" "_" "9" "_" "9" "_"
#
# How we create the regular expressions ?
# We take the compressed pattern and quote every separator.
# The we replace the 9s with [0-9][0-9]*
# and the _s with [a-zA-Z][a-zA-Z]* .
# The above example will become:
#
# "99_.9.__abc9_"  ===compress==> "9_.9._abc9_" ===rexify==>
# [0-9][0-9]*[a-zA-Z][a-zA-Z]*\.[0-9][0-9]*\.[a-zA-Z][a-zA-Z]*\a\b\c[0-9][0-9]*[a-zA-Z][a-zA-Z]*
#
# Voila.
#
# To yield the subparts from the string we additionally enclose the
# 9s and _s with \( and \).
#
###############################################################################

# ****************************************************************
# **                      helper definitions                    **
# ****************************************************************
m4_define([CS_VCHK_RUNTH], [m4_pushdef([i], [$1])m4_if($1,0,,[CS_VCHK_RUNTH(m4_decr($1), [$2])][$2])m4_popdef([i])])
m4_define([CS_VCHK_PREFIX], [])
m4_define([CS_VCHK_SUFFIX], [])
m4_define([CS_VCHK_GROUPPREFIX], [\(])
m4_define([CS_VCHK_GROUPSUFFIX], [\)])
m4_define([CS_VCHK_CHAR], [[[[a-zA-Z]]]])
m4_define([CS_VCHK_DIGIT], [[[0-9]]])
m4_define([CS_VCHK_SEQUENCE], [CS_VCHK_PREFIX[]CS_VCHK_SINGLE[]CS_VCHK_SINGLE[]*CS_VCHK_SUFFIX[]])
m4_define([CS_VCHK_OPTSEQUENCE], [CS_VCHK_PREFIX[]CS_VCHK_SINGLE[]*CS_VCHK_SUFFIX[]])
m4_define([CS_VCHK_REXSEQ], [m4_bpatsubst($1, [$2], [[]CS_VCHK_SEQUENCE[]])])
m4_define([CS_VCHK_GROUPINGON], [m4_pushdef([CS_VCHK_PREFIX], [CS_VCHK_GROUPPREFIX])m4_pushdef([CS_VCHK_SUFFIX], [CS_VCHK_GROUPSUFFIX])])
m4_define([CS_VCHK_GROUPINGOFF], [m4_popdef([CS_VCHK_SUFFIX])m4_popdef([CS_VCHK_PREFIX])])
m4_define([CS_VCHK_OPTON], [m4_pushdef([CS_VCHK_SEQUENCE], [CS_VCHK_OPTSEQUENCE])])
m4_define([CS_VCHK_OPTOFF], [m4_popdef([CS_VCHK_SEQUENCE])])
m4_define([CS_VCHK_RMOPT], [CS_VCHK_RMCHAR([$1], m4_index([$1], [|]))])
m4_define([CS_VCHK_RMCHAR], [m4_if($2,-1,[$1],m4_substr([$1], 0, $2)[]m4_substr([$1], m4_incr($2)))])
m4_define([CS_VCHK_RMALL], [m4_translit([$1], [|], [])])
m4_define([CS_VCHK_CUTOFF], [m4_if(m4_index($1,[|]),-1, [$1], [m4_substr($1, 0, m4_index($1,[|]))])])
m4_define([CS_VCHK_CYCLEOPT], [
m4_if($2,-1,, [m4_pushdef([i], CS_VCHK_CUTOFF([$1])) m4_pushdef([j], CS_VCHK_DUMMY_TAIL([$1])) CS_VCHK_CYCLEOPT( CS_VCHK_RMOPT([$1]), m4_index($1, [|]), [$3])$3 m4_popdef([i]) m4_popdef([j])])
])
m4_define([CS_VCHK_TAIL], [m4_if(m4_index($1,[|]),-1, [], [m4_substr($1, m4_incr(m4_index($1,[|])))])])
m4_define([CS_VCHK_DUMMY_COMPRESS], [m4_bpatsubst(m4_bpatsubst([$1], [__*], [A]), [99*], [0])])
m4_define([CS_VCHK_DUMMY_TAIL], [CS_VCHK_DUMMY_COMPRESS(m4_translit(CS_VCHK_TAIL([$1]), [|], []))])

# ****************************************************************
# **                      FlagsOn / FlagsOff                    **
# ****************************************************************
m4_define([CS_VCHK_FLAGSON],
[m4_if($#, 0, [],
       $1, [], [],
       [$1], [group], [CS_VCHK_GROUPINGON[]],
       [$1], [opt], [CS_VCHK_OPTON[]])dnl
m4_if($#, 0, [], $1, [], [], [CS_VCHK_FLAGSON(m4_shift($@))])])

m4_define([CS_VCHK_FLAGSOFF],
[m4_if($#, 0, [],
       $1, [], [],
       $1, [group], [CS_VCHK_GROUPINGOFF[]],
       [$1], [opt], [CS_VCHK_OPTOFF[]])dnl
m4_if($#, 0, [], $1, [], [], [CS_VCHK_FLAGSOFF(m4_shift($@))])])

# ****************************************************************
# **                      rexify / sedify                       **
# ****************************************************************
m4_define([CS_VCHK_REXIFY],
[m4_pushdef([CS_VCHK_SINGLE], [$1])dnl
CS_VCHK_FLAGSON(m4_shift(m4_shift(m4_shift($@))))dnl
CS_VCHK_REXSEQ([$3], [$2])dnl
CS_VCHK_FLAGSOFF(m4_shift(m4_shift(m4_shift($@))))dnl
m4_popdef([CS_VCHK_SINGLE])])

m4_define([CS_VCHK_QUOTESEP], [m4_bpatsubst($1, [[^9_]], [\\\&])])

m4_define([CS_VCHK_REXCHAR], [CS_VCHK_REXIFY([CS_VCHK_CHAR], [__*], $@)])
m4_define([CS_VCHK_REXDIGIT],  [CS_VCHK_REXIFY([CS_VCHK_DIGIT], [99*], $@)])
m4_define([CS_VCHK_SEDIFY], [CS_VCHK_REXDIGIT([CS_VCHK_REXCHAR([CS_VCHK_QUOTESEP([$1])], m4_shift($@))], m4_shift($@))])
m4_define([CS_VCHK_SEDEXPRALL], [/CS_VCHK_SEDIFY([$1])/!d;s/.*\(CS_VCHK_SEDIFY([$1])\).*/\1/;q])
m4_define([CS_VCHK_SEDEXPRNTH], [/CS_VCHK_SEDIFY([$1])/!d;s/.*CS_VCHK_SEDIFY([$1],[group]).*/\$2/])

# ****************************************************************
# **                      Pattern splitting                     **
# ****************************************************************
m4_define([CS_VCHK_SPLITSEP], [CS_VCHK_REXIFY([s], [[^9_][^9_]*], $@)])
m4_define([CS_VCHK_SPLITDIGIT], [CS_VCHK_REXIFY([d], [99*], $@)])
m4_define([CS_VCHK_SPLITCHAR], [CS_VCHK_REXIFY([c], [__*], $@)])

# ****************************************************************
# ** return a list of 's' 'd' 'c' 'e' chars denoting the kind   **
# ** pattern parts: separator, digit, char, end                 **
# ****************************************************************
m4_define([CS_VCHK_PATTERNLIST], [m4_pushdef([CS_VCHK_SEQUENCE], [CS_VCHK_SINGLE ])dnl
m4_translit(CS_VCHK_SPLITDIGIT([CS_VCHK_SPLITCHAR([CS_VCHK_SPLITSEP([$1])])]), [ ], m4_if([$2],[],[ ],[$2]))e[]dnl
m4_popdef([CS_VCHK_SEQUENCE])])

# ****************************************************************
# ** Build the shell commands we emit to the configure script.  **
# ****************************************************************
m4_define([CS_VCHK_PATCOUNT], [m4_len(m4_bpatsubst(CS_VCHK_PATTERNLIST([$1]), [[^dc]]))])

# ****************************************************************************************
# ** CS_VCHK_EXTRACTVERSION(EXTRACT_CALL, MIN_VERSION, PATTERN, PRGPREFIX, COMPARISION) **
# ****************************************************************************************
m4_define([CS_VCHK_EXTRACTVERSION],
[cs_prog_$4_is_version=
cs_prog_$4_min_version=
cs_prog_$4_is_suffix=
cs_prog_$4_min_suffix=
cs_prog_$4_is_suffix_done=
cs_prog_$4_min_suffix_done=
CS_VCHK_CYCLEOPT([$3], [], 
[test -z $cs_prog_$4_is_version && cs_prog_$4_is_version=`$1 | sed 'CS_VCHK_SEDEXPRALL([i])'`
test -n "$cs_prog_$4_is_version" && test -z $cs_prog_$4_is_suffix_done  && { cs_prog_$4_is_suffix_done=yes ; cs_prog_$4_is_suffix=j ; }
])
CS_VCHK_CYCLEOPT([$3], , 
[test -z $cs_prog_$4_min_version && cs_prog_$4_min_version=`echo $2 | sed 'CS_VCHK_SEDEXPRALL([i])'`
test -n "$cs_prog_$4_min_version" && test -z $cs_prog_$4_min_suffix_done  && { cs_prog_$4_min_suffix_done=yes ; cs_prog_$4_min_suffix=j ; }
])
CS_VCHK_RUNTH([CS_VCHK_PATCOUNT([$3])],
    [cs_prog_$4_is_ver_[]i=`echo ${cs_prog_$4_is_version}${cs_prog_$4_is_suffix} | sed 'CS_VCHK_SEDEXPRNTH([CS_VCHK_RMALL([$3])], [i])'`
])
CS_VCHK_RUNTH([CS_VCHK_PATCOUNT([$3])],
    [cs_prog_$4_min_ver_[]i=`echo $cs_prog_$4_min_version${cs_prog_$4_min_suffix} | sed 'CS_VCHK_SEDEXPRNTH([CS_VCHK_RMALL([$3])], [i])'`
])
cs_cv_prog_$4_version_ok=''
CS_VCHK_RUNTH([CS_VCHK_PATCOUNT([$3])],
[test -z "$cs_cv_prog_$4_version_ok" && { expr "$cs_prog_$4_is_ver_[]i" "$5" "$cs_prog_$4_min_ver_[]i" >/dev/null || cs_cv_prog_$4_version_ok=no ; }
test -z "$cs_cv_prog_$4_version_ok" && { expr "$cs_prog_$4_min_ver_[]i" "$5" "$cs_prog_$4_is_ver_[]i" >/dev/null || cs_cv_prog_$4_version_ok=yes ; }
])
AS_IF([test -z "$cs_cv_prog_$4_version_ok"], [cs_cv_prog_$4_version_ok=yes])
cs_cv_prog_$4_version_ok_annotated="$cs_cv_prog_$4_version_ok"
AS_IF([test -n "$cs_prog_$4_is_version"],
    [cs_cv_prog_$4_version_ok_annotated="$cs_cv_prog_$4_version_ok_annotated (version $cs_prog_$4_is_version)"])
])

##############################################################################
# CS_CHECK_PROG_VERSION(PROG, EXTRACT_CALL, VERSION, PATTERN,
#                       [ACTION-IF-OKAY], [ACTION-IF-NOT-OKAY], [CMP])
# Check the version of a program PROG.
# Version information is emitted by EXTRACT_CALL (for instance "bison -V").
# The discovered program version is compared against VERSION.
# The pattern of the version string matches PATTERN
# The extracted version and the supplied version are compared with the CMP
# operator. i.e. EXTRACTED_VERSION CMP SUPPLIED_VERSION
# CMP defaults to >= if not specified.
# ACTION-IF-OKAY is invoked if comparision yields true, otherwise
# ACTION-IF-NOT-OKAY is invoked.
#
# PATTERN literals: 9 .. marks a non empty sequence of digits
#                   _ .. marks a non empty sequence of characters from [a-zA-Z]
#                   | .. everything behind is optional
#                     .. everything else is taken as separator - it is better
#                        to not try stuff like space, slash or comma.
#
# The test results in cs_cv_prog_PROG_version_ok being either yes or no.
##############################################################################
AC_DEFUN([CS_CHECK_PROG_VERSION],
[AC_CACHE_CHECK([if $1 version m4_default([$7],[>=]) $3],
    [AS_TR_SH([cs_cv_prog_$1_version_ok_annotated])],
    [CS_VCHK_EXTRACTVERSION([$2], [$3], [$4], AS_TR_SH([$1]),
	m4_default([$7],[>=]))])
AS_IF([test "$AS_TR_SH([cs_cv_prog_$1_version_ok])" = yes], [$5], [$6])])
