# Toplevel make file to build the libsimdmath library for both SPU and PPU
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

# How to build:
#
#   To build 32 bit libraries:
#
#       make ARCH_PPU=32
#
#   To use "gcc" instead of "ppu-gcc".
#
#       make CROSS_PPU=

prefix = /usr
DESTDIR =

ARCH_PPU = 64
CROSS_PPU = ppu-
AR_PPU = $(CROSS_PPU)ar
CC_PPU = $(CROSS_PPU)gcc
CXX_PPU = $(CROSS_PPU)g++
RANLIB_PPU = $(CROSS_PPU)ranlib
TEST_CMD_PPU =

ARCH_CFLAGS_PPU = -m$(ARCH_PPU) -maltivec -mabi=altivec

CROSS_SPU = spu-
AR_SPU = $(CROSS_SPU)ar
CC_SPU = $(CROSS_SPU)gcc
CXX_SPU = $(CROSS_SPU)g++
RANLIB_SPU = $(CROSS_SPU)ranlib
TEST_CMD_SPU =

INSTALL = install

MAKE_DEFS = \
	prefix='$(prefix)' \
	DESTDIR='$(DESTDIR)' \
	LIB_MAJOR_VERSION='$(LIB_MAJOR_VERSION)' \
	LIB_MINOR_VERSION='$(LIB_MINOR_VERSION)' \
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
	CROSS_SPU='$(CROSS_SPU)' \
	AR_SPU='$(AR_SPU)' \
	CC_SPU='$(CC_SPU)' \
	CXX_SPU='$(CXX_SPU)' \
	RANLIB_SPU='$(RANLIB_SPU)' \
	TEST_CMD_SPU='$(TEST_CMD_SPU)' \
	INSTALL='$(INSTALL)'

LIB_MAJOR_VERSION = 1
LIB_MINOR_VERSION = 0

LIB_BASE = simdmath
LIB_NAME = lib$(LIB_BASE)
STATIC_LIB = $(LIB_NAME).a
SHARED_LIB = $(LIB_NAME).so

all: spu_library ppu_library

spu_library:
	cd spu; $(MAKE) $(MAKE_DEFS)

ppu_library:
	cd ppu; $(MAKE) $(MAKE_DEFS)

install: spu_install ppu_install

spu_install:
	cd spu; $(MAKE) $(MAKE_DEFS) install

ppu_install:
	cd ppu; $(MAKE) $(MAKE_DEFS) install

clean:
	cd spu; $(MAKE) $(MAKE_DEFS) clean
	cd ppu; $(MAKE) $(MAKE_DEFS) clean

check: check_ppu check_spu

check_ppu:
	cd ppu; $(MAKE) $(MAKE_DEFS) check

check_spu:
	cd spu; $(MAKE) $(MAKE_DEFS) check
