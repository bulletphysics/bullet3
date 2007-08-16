# make file to build the libsimdmath library for PPU
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
ARCH = ppu

include $(topdir)/Make.defs

# All that you do to add a file is edit OBJS, the rest will just work

OBJS = fabsf4.o absi4.o truncf4.o sqrtf4.o tanf4.o \
	negatef4.o fmaf4.o copysignf4.o modff4.o \
	fmaxf4.o fminf4.o fdimf4.o sinf4.o asinf4.o \
	floorf4.o recipf4.o ceilf4.o divf4.o divi4.o \
	cosf4.o hypotf4.o cbrtf4.o logf4.o sincosf4.o \
	rsqrtf4.o log2f4.o ldexpf4.o expf4.o frexpf4.o \
	expm1f4.o logbf4.o log1pf4.o log10f4.o ilogbf4.o \
	fmodf4.o negatei4.o exp2f4.o powf4.o atanf4.o \
	atan2f4.o acosf4.o

include $(topdir)/Make.rules

$(OBJS): ../common/simdmath.h simdmath/_vec_utils.h


# Some Objects have special header files.
sinf4.o cosf4.o sincosf4.o tanf4.o: ../common/simdmath/_sincos.h
