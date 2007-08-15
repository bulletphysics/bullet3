# Makefile for vector math library.
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

# How to build:
#
#   Nothing to do (The all of the library is implemented as inline).
#
# How to install:
#
#   To install the library:
#
#      make ARCH=<ARCHITECTURE> install
#
#   where <ARCHITECTURE> must be one of:
#
#      ppu (PowerPC)
#      spu
#      SSE
#      scalar (generic)
#
#   e.g.) make ARCH=cell install
#

topdir = .
ARCH = scalar

prefix_spu = /usr/spu

ARCH_DIRS = $(ARCH)
ARCH_INSTALL= $(ARCH_INSTALL_$(ARCH))
ARCH_CHECK= $(ARCH_CHECK_$(ARCH))

prefix = $(if $(prefix_$(ARCH)),$(prefix_$(ARCH)),/usr)
DESTDIR =

COMMON_DIRS = scalar

INSTALL = install

LIB_MAJOR_VERSION = 1
LIB_MINOR_VERSION = 0
LIB_RELEASE = 1
LIB_FULL_VERSION = $(LIB_MAJOR_VERSION).$(LIB_MINOR_VERSION).$(LIB_RELEASE)

LIB_BASE = vectormath

TAR_NAME = $(LIB_BASE)-$(LIB_FULL_VERSION)
TAR_BALL = $(TAR_NAME).tar.gz

all:
	@true

install:
	$(INSTALL) -m 755 -d $(DESTDIR)$(prefix)/include/vectormath/c
	$(INSTALL) -m 755 -d $(DESTDIR)$(prefix)/include/vectormath/cpp
	$(INSTALL) -m 644 include/vectormath/c/*.h $(DESTDIR)$(prefix)/include/vectormath/c/
	$(INSTALL) -m 644 include/vectormath/cpp/*.h $(DESTDIR)$(prefix)/include/vectormath/cpp/
	$(INSTALL) -m 755 -d $(DESTDIR)$(prefix)/include/vectormath/cpp
	for _d in $(ARCH_DIRS) $(COMMON_DIRS); do \
		if test -d include/vectormath/$$_d/c; then \
			$(INSTALL) -m 755 -d $(DESTDIR)$(prefix)/include/vectormath/$$_d/c && \
			$(INSTALL) -m 644 include/vectormath/$$_d/c/*.h \
				$(DESTDIR)$(prefix)/include/vectormath/$$_d/c/ || exit 1; \
		fi; \
		if test -d include/vectormath/$$_d/cpp; then \
			$(INSTALL) -m 755 -d $(DESTDIR)$(prefix)/include/vectormath/$$_d/cpp && \
			$(INSTALL) -m 644 include/vectormath/$$_d/cpp/*.h \
				$(DESTDIR)$(prefix)/include/vectormath/$$_d/cpp/ || exit 1; \
		fi; \
	done

check:
	$(MAKE) -C tests ARCH=$(ARCH) check

clean:
	$(MAKE) -C tests clean
	-rm -f $(TAR_BALL)

distclean:
	$(MAKE) -C tests distclean

dist:
	-rm -rf .dist
	mkdir -p .dist/$(TAR_NAME)
	find . -name .dist -prune -o \
		-name .CVS -prune -o -name .svn -prune -o \
		-name .pc -prune  -o -name patches -prune -o \
		'(' -name README -o -name LICENSE -o \
			-name Makefile -o -name '*.[ch]' -o -name '*.cpp' -o \
			-name '*.pl' -o -name '*.txt' -o -name '*.pdf' -o -name '*.spec' ')' \
		-print | tar -T - -cf - | tar xf - -C .dist/$(TAR_NAME)
	tar zcf $(TAR_BALL) -C .dist $(TAR_NAME)
	-rm -rf .dist
