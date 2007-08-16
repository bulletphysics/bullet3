# Makefile for vector math library testsuite.
#
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

ARCH = scalar

TESTS_all = \
	test1_aos_c.elf \
	test1_soa_c.elf \
	test2_aos_c.elf \
	test2_soa_c.elf \
	test3_aos_c.elf \
	test3_soa_c.elf \
	test4_aos_c.elf \
	test4_soa_c.elf \
	test1_aos_cpp.elf \
	test1_soa_cpp.elf \
	test2_aos_cpp.elf \
	test2_soa_cpp.elf \
	test3_aos_cpp.elf \
	test3_soa_cpp.elf \
	test4_aos_cpp.elf \
	test4_soa_cpp.elf

TESTS_ppu = $(TESTS_all)
ARCH_CFLAGS_ppu = -maltivec -mabi=altivec -I$(SIMDMATH_DIR)/common
ARCH_LDFLAGS_ppu = -L$(SIMDMATH_DIR)/ppu -lsimdmath -static

CROSS_spu = spu-
TESTS_spu = $(TESTS_all)
ARCH_CFLAGS_spu = -I$(SIMDMATH_DIR)/common
ARCH_LDFLAGS_spu = -L$(SIMDMATH_DIR)/spu -lsimdmath

TESTS_SSE = \
	test1_aos_cpp.elf \
	test2_aos_cpp.elf \
	test3_aos_cpp.elf \
	test4_aos_cpp.elf
ARCH_CFLAGS_SSE = -msse

TESTS_scalar = \
	test1_aos_c.elf \
	test2_aos_c.elf \
	test3_aos_c.elf \
	test4_aos_c.elf \
	test1_aos_cpp.elf \
	test2_aos_cpp.elf \
	test3_aos_cpp.elf \
	test4_aos_cpp.elf

TESTS = $(TESTS_$(ARCH))
ARCH_CFLAGS = $(ARCH_CFLAGS_$(ARCH))
ARCH_LDFLAGS = $(ARCH_LDFLAGS_$(ARCH))

SIMDMATH_DIR = $(topdir)/../simdmathlibrary

RESULTS = $(TESTS:.elf=.$(ARCH).out)
DIFFS = $(RESULTS:.out=.cmp)

CROSS = $(CROSS_$(ARCH))
CC = $(CROSS)gcc
CXX = $(CROSS)g++
LD = $(CC)
LDXX = $(CXX)

CFLAGS = -O2 -W -Wall -D_VECTORMATH_DEBUG $(ARCH_CFLAGS)
LDFLAGS = -lm $(ARCH_LDFLAGS)

C_INCLUDES = -I$(topdir)/include/vectormath/c
CXX_INCLUDES = -I$(topdir)/include/vectormath/cpp

PERL = perl

all: $(TESTS)

check: $(DIFFS)

clean:
	-rm -f *.elf

distclean: clean
	-rm -f *.out *.cmp *~

%_c.elf: %_c.c
	$(CC) $(CFLAGS) $(C_INCLUDES) -o $@ $< $(LDFLAGS)

%_cpp.elf: %_cpp.cpp
	$(CXX) $(CFLAGS) $(CXX_INCLUDES) -o $@ $< $(LDFLAGS)

%.$(ARCH).out: %.elf
	./$< > $@

test1_%.cmp: test1_%.out
	$(PERL) ./clean.pl < $< > $<.tmp
	$(PERL) ./compare.pl $<.tmp test1_reference.txt | tee $@
	rm $<.tmp

test2_%.cmp: test2_%.out
	$(PERL) ./clean.pl < $< > $<.tmp
	$(PERL) ./compare.pl $<.tmp test2_reference.txt | tee $@
	rm $<.tmp

test3_%.cmp: test3_%.out
	$(PERL) ./clean.pl < $< > $<.tmp
	$(PERL) ./compare.pl $<.tmp test3_reference.txt | tee $@
	rm $<.tmp

test4_%.cmp: test4_%.out
	$(PERL) ./clean.pl < $< > $<.tmp
	$(PERL) ./compare.pl $<.tmp test4_reference.txt | tee $@
	rm $<.tmp
