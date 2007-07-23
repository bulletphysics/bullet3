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



# All that you do to add a file is edit OBJS, the rest will just work

prefix = /usr
DESTDIR =

OBJS = fabsf4.o absi4.o truncf4.o sqrtf4.o tanf4.o \
	negatef4.o fmaf4.o copysignf4.o modff4.o \
	fmaxf4.o fminf4.o fdimf4.o sinf4.o asinf4.o \
	floorf4.o recipf4.o ceilf4.o divf4.o divi4.o \
	cosf4.o hypotf4.o cbrtf4.o logf4.o sincosf4.o \
	rsqrtf4.o log2f4.o ldexpf4.o expf4.o frexpf4.o \
	expm1f4.o logbf4.o log1pf4.o log10f4.o ilogbf4.o \
	fmodf4.o negatei4.o exp2f4.o powf4.o atanf4.o \
	atan2f4.o acosf4.o

INCLUDES_PPU = -I../

ARCH_PPU = 64
CROSS_PPU = ppu-
AR_PPU = $(CROSS_PPU)ar
CC_PPU = $(CROSS_PPU)gcc
CXX_PPU = $(CROSS_PPU)g++
RANLIB_PPU = $(CROSS_PPU)ranlib
TEST_CMD_PPU =

ARCH_CFLAGS_PPU = -m$(ARCH_PPU) -maltivec -mabi=altivec
CFLAGS_PPU=$(INCLUDES_PPU)  -O2 -W -Wall -std=gnu99 $(ARCH_CFLAGS_PPU) -fPIC
LDFLAGS_PPU = $(ARCH_CFLAGS_PPU)

INSTALL = install

MAKE_DEFS = \
	prefix='$(prefix)' \
	DESTDIR='$(DESTDIR)' \
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
	RANLIB_PPU='$(RANLIB_PPU)' \
	TEST_CMD_PPU='$(TEST_CMD_PPU)' \
	INSTALL='$(INSTALL)'

LIB_MAJOR_VERSION = 1
LIB_MINOR_VERSION = 0

LIB_BASE = simdmath
LIB_NAME = lib$(LIB_BASE)
STATIC_LIB = $(LIB_NAME).a
SHARED_LIB = $(LIB_NAME).so
SHARED_LIB_SONAME = $(SHARED_LIB).$(LIB_MAJOR_VERSION)
SHARED_LIB_FULL = $(SHARED_LIB).$(LIB_MAJOR_VERSION).$(LIB_MINOR_VERSION)

ALL_LIBS = $(STATIC_LIB) $(SHARED_LIB) $(SHARED_LIB_FULL) $(SHARED_LIB_SONAME)

all: $(ALL_LIBS)

static: $(STATIC_LIB)

shared: $(SHARED_LIB)

$(STATIC_LIB): $(OBJS)
	$(AR_PPU) cr $@ $(OBJS)
	$(RANLIB_PPU) $@

$(SHARED_LIB): $(OBJS)
	$(CC_PPU) -shared $(OBJS) -o $@ $(LDFLAGS_PPU) -Wl,-h,$(SHARED_LIB_SONAME)

$(SHARED_LIB_SONAME) $(SHARED_LIB_FULL): $(SHARED_LIB)
	ln -fs $(SHARED_LIB) $@

install: $(ALL_LIBS)
	$(INSTALL) -m 755 -d $(DESTDIR)$(prefix)/include
	$(INSTALL) -m 644 ../simdmath.h $(DESTDIR)$(prefix)/include/
	$(INSTALL) -m 755 -d $(DESTDIR)$(prefix)/lib
	$(INSTALL) -m 644 $(STATIC_LIB) $(DESTDIR)$(prefix)/lib/$(STATIC_LIB)
	$(INSTALL) -m 755 $(SHARED_LIB) $(DESTDIR)$(prefix)/lib/$(SHARED_LIB_FULL)
	ln -fs $(SHARED_LIB_FULL) $(DESTDIR)$(prefix)/lib/$(SHARED_LIB_SONAME)
	ln -fs $(SHARED_LIB_SONAME) $(DESTDIR)$(prefix)/lib/$(SHARED_LIB)

clean:
	cd tests; $(MAKE) $(MAKE_DEFS) clean
	rm -f $(OBJS)
	rm -f $(STATIC_LIB) $(SHARED_LIB) $(SHARED_LIB_SONAME) $(SHARED_LIB_FULL)

$(OBJS): ../simdmath.h common-types.h

check: $(ALL_LIBS)
	cd tests; $(MAKE) $(MAKE_DEFS) all; $(MAKE) $(MAKE_DEFS) check


# Some Objects have special header files.
sinf4.o cosf4.o sincosf4.o tanf4.o: sincos_c.h


%.o: %.c
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
