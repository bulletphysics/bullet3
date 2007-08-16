# make file to build the libsimdmath library for SPU
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

topdir = ..
ARCH = spu

include $(topdir)/Make.defs

# All that you do to add a file is edit OBJS, the rest will just work

OBJS = fabsd2.o fabsf4.o truncf4.o divf4.o tanf4.o isnanf4.o isnand2.o isinff4.o isinfd2.o \
	is0denormf4.o is0denormd2.o recipd2.o divd2.o tand2.o sqrtf4.o absi4.o sqrtd2.o \
	sinf4.o isgreaterd2.o sind2.o sincosd2.o sincosf4.o rsqrtf4.o signbitf4.o signbitd2.o \
	rsqrtd2.o copysignf4.o remainderf4.o recipf4.o copysignd2.o log2f4.o \
	negatef4.o negated2.o modff4.o asinf4.o frexpf4.o frexpd2.o ldexpf4.o cbrtf4.o \
	cosd2.o cosf4.o hypotf4.o hypotd2.o ceilf4.o fmaf4.o fmaxf4.o fminf4.o floorf4.o \
	fdimf4.o fmodf4.o negatei4.o logf4.o log1pf4.o log10f4.o expm1f4.o \
	expf4.o divi4.o exp2f4.o powf4.o atanf4.o atan2f4.o acosf4.o ilogbf4.o ilogbd2.o \
	logbf4.o logbd2.o llroundd2.o llroundf4.o llrintf4.o isequalf4.o isequald2.o \
	islessgreaterf4.o islessgreaterd2.o islessf4.o islessd2.o isgreaterf4.o \
	isgreaterd2.o islessequalf4.o islessequald2.o isgreaterequalf4.o isgreaterequald2.o \
	isfinitef4.o isfinited2.o isnormalf4.o isnormald2.o isunorderedf4.o isunorderedd2.o \
	llrintd2.o roundf4.o rintf4.o irintf4.o iroundf4.o fmad2.o fmaxd2.o fmind2.o fdimd2.o \
	nextafterd2.o fpclassifyf4.o fpclassifyd2.o nearbyintd2.o nextafterf4.o nearbyintf4.o \
	llabsi2.o truncd2.o roundd2.o rintd2.o negatell2.o divu4.o modfd2.o lldivu2.o \
	ceild2.o floord2.o ldexpd2.o scalbnf4.o scalbllnd2.o lldivi2.o remquof4.o remquod2.o\
	fmodd2.o remainderd2.o

include $(topdir)/Make.rules

$(OBJS): ../common/simdmath.h

# Some Objects have special header files.
sinf4.o sind2.o sincosf4.o cosd2.o: ../common/simdmath/_sincos.h
lldivu2.o lldivi2.o : simdmath/_lldiv.h
