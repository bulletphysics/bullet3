# Makefile for testsuite for the PPU SIMD math library
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

TESTS = main_vmtest

STATIC_TESTS = $(TESTS)
SHARED_TESTS = $(TESTS:=.shared)
ALL_TESTS = $(STATIC_TESTS) $(SHARED_TESTS)

INCLUDES_PPU = -I../simdmathlibrary/common -Iother/rs6000
#ARCH_PPU can also be 64
ARCH_PPU = 32 
CROSS_PPU = ppu-
AR_PPU = $(CROSS_PPU)ar
CC_PPU = $(CROSS_PPU)gcc
CXX_PPU = $(CROSS_PPU)g++
TEST_CMD_PPU =

ARCH_CFLAGS_PPU = -m$(ARCH_PPU) -maltivec -mabi=altivec
CFLAGS_PPU = $(INCLUDES_PPU) -O2 -W -Wall $(ARCH_CFLAGS_PPU)
STATIC_LDFLAGS_PPU = -static
SHARED_LDFLAGS_PPU = -Wl,-rpath=..
LDFLAGS_PPU = $(ARCH_CFLAGS_PPU) -L../simdmathlibrary/ppu -l$(LIB_BASE) -lm -lstdc++

MAKE_DEFS = \
	LIB_BASE='$(LIB_BASE)' \
	LIB_NAME='$(LIB_NAME)' \
	STATIC_LIB='$(STATIC_LIB)' \
	SHARED_LIB='$(SHARED_LIB)' \
	ARCH_PPU='$(ARCH_PPU)' \
	ARCH_CFLAGS_PPU='$(ARCH_CFLAGS_PPU)' \
	CROSS_PPU='$(CROSS_PPU)' \
	AR_PPU='$(AR_PPU)' \
	CC_PPU='$(CC_PPU)' \
	CXX_PPU='$(CXX_PPU)' \
	TEST_CMD_PPU='$(TEST_CMD_PPU)'

LIB_BASE = simdmath
LIB_NAME = lib$(LIB_BASE)
STATIC_LIB = $(LIB_NAME).a
SHARED_LIB = $(LIB_NAME).so

TEST_CMD = $(TEST_CMD_PPU)

#COMMON_OBJS = testutils.o


all: $(ALL_TESTS)


$(STATIC_TESTS): %: %.o ../simdmathlibrary/ppu/$(STATIC_LIB) $(COMMON_OBJS)
	$(CC_PPU) $*.o $(COMMON_OBJS) $(LDFLAGS_PPU) $(STATIC_LDFLAGS_PPU) -o $@

$(SHARED_TESTS): %.shared: %.o ../simdmathlibrary/ppu/$(SHARED_LIB) $(COMMON_OBJS)
	$(CC_PPU) $*.o $(COMMON_OBJS) $(LDFLAGS_PPU) $(SHARED_LDFLAGS_PPU) -o $@

clean:
	rm -f *.o
	rm -f $(STATIC_TESTS) $(SHARED_TESTS)
	rm -f core*

check: $(ALL_TESTS)
	for test in $(ALL_TESTS); do	\
		echo "TEST $${test}";		\
		if $(TEST_CMD) ./$${test}; then \
			pass="$$pass $$test"; \
		else \
			fail="$$fail $$test"; \
		fi \
	done; \
	echo; echo "PASS:$$pass"; echo "FAIL:$$fail"; \
	test -z "$$fail"

static_check:
	$(MAKE) $(MAKE_DEFS) ALL_TESTS="$(STATIC_TESTS)" check

shared_check:
	$(MAKE) $(MAKE_DEFS) ALL_TESTS="$(SHARED_TESTS)" check

../$(STATIC_LIB):
	cd ../;$(MAKE) $(MAKE_DEFS) $(STATIC_LIB)

../$(SHARED_LIB):
	cd ../;$(MAKE) $(MAKE_DEFS) $(SHARED_LIB)

%.o: %.c  common-test.h testutils.h
	$(CC_PPU) $(CFLAGS_PPU) -c $<

#----------
#   C++
#----------
%.o: %.C
	$(CXX_PPU) $(CFLAGS_PPU) -c $<

%.o: %.cpp
	$(CXX_PPU) $(CFLAGS_PPU) -c $<

%.o: %.cc
	$(CXX_PPU) $(CFLAGS_PPU) -c $<

%.o: %.cxx
	$(CXX_PPU) $(CFLAGS_PPU) -c $<

