# Makefile for testsuite for the SPU SIMD math library
#   Copyright (C) 2006, 2007 Sony Computer Entertainment Inc.
#   All rights reserved.
#
#   Redistribution and use in source and binary forms,
#   with or without modification, are permitted provided that the
#   following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Sony Computer Entertainment Inc nor the names
#      of its contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
#   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.

topdir = ../..
ARCH = spu

include $(topdir)/Make.defs

TESTS = fabsd2 fabsf4 truncf4 divf4 recipd2 divd2 sqrtf4 \
	absi4 sqrtd2 rsqrtf4 rsqrtd2 copysignf4 remainderf4 \
	recipf4 copysignd2 negatef4 negated2 modff4 ceilf4 \
	fminf4_fmaxf4 floorf4 negatei4 divi4 llroundd2 llroundf4 \
    llrintf4 isequalf4 isequald2 islessgreaterf4 islessgreaterd2 \
    islessf4 islessd2 isgreaterf4 isgreaterd2 islessequalf4 islessequald2 \
    isgreaterequalf4 isgreaterequald2 isnanf4 isnand2 isinff4 isinfd2 \
    isfinitef4 isfinited2 isnormalf4 isnormald2 isunorderedf4 isunorderedd2 \
    is0denormf4 is0denormd2 signbitf4 signbitd2 llrintd2 \
    roundf4 iroundf4 rintf4 irintf4 fmind2_fmaxd2 fdimd2 \
    fmad2 nextafterd2 nextafterf4 fpclassifyf4 fpclassifyd2 nearbyintd2 nearbyintf4 \
    llabsi2 truncd2 roundd2 rintd2 negatell2 hypotf4 hypotd2 divu4 modfd2 \
    lldivu2 ceild2 floord2 ldexpd2 ilogbd2 ilogbf4 logbf4 logbd2 scalbnf4 \
    scalbllnd2 lldivi2 frexpf4 frexpd2 remquof4 remquod2 fmodd2 remainderd2

OBJS = testutils.o

include $(topdir)/Make.test
