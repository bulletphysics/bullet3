#!/usr/bin/make


LIBNAME = libxml2

prefix=..

INCPATH=$(prefix)/include
LIBPATH=$(prefix)/lib

PPU_INCDIRS=-I$(INCPATH) -Iinclude
PPU_OPTIMIZE_LV= -O3 -funroll-loops

PPU_LIB_TARGET=$(LIBNAME).a
PPU_SRCS = SAX.c entities.c \
	encoding.c error.c \
	parserInternals.c parser.c tree.c \
	hash.c list.c xmlIO.c xmlmemory.c \
	uri.c valid.c xlink.c \
	HTMLparser.c HTMLtree.c debugXML.c \
	xinclude.c \
	nanohttp.c nanoftp.c DOCBparser.c \
	catalog.c globals.c threads.c \
	c14n.c xmlstring.c xmlregexp.c \
	xmlunicode.c xmlreader.c relaxng.c \
	dict.c SAX2.c xmlwriter.c \
	legacy.c chvalid.c pattern.c \
	xmlsave.c


