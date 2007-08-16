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

topdir = .
ARCH = unknown

include $(topdir)/Make.defs

SUBDIRS = common $(ARCH_DIR)

all install check:
	@for _d in $(SUBDIRS); do $(MAKE) -C $$_d $@ || exit 1; done

clean:
	-@for _d in $(SUBDIRS); do $(MAKE) -C $$_d $@; done
	-rm -rf $(TAR_BALL) .dist

dist:
	-rm -rf .dist
	mkdir -p .dist/$(TAR_NAME)
	find . -name .dist -prune -o -name '*~' -prune -o \
		-name .CVS -prune -o -name .svn -prune -o \
		-name .pc -prune  -o -name patches -prune -o \
		'(' -name README -o -name LICENSE -o \
			-name 'Make*' -o -name '*.[ch]' -o -name '*.spec' ')' \
		-print | tar -T - -cf - | tar xf - -C .dist/$(TAR_NAME)
	tar zcf $(TAR_BALL) -C .dist $(TAR_NAME)
	-rm -rf .dist
