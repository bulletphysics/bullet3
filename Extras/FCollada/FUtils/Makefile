#!/usr/bin/make
# FUtils makefile for PS3

include ../../MakeDefs

LIBNAME = libfutils

prefix=..

INCPATH=$(prefix)/include
LIBPATH=$(prefix)/lib

PPU_INCDIRS=-I$(INCPATH) -I.. -I../LibXML/include
PPU_OPTIMIZE_LV= -O3 -funroll-loops

PPU_LIB_TARGET=$(LIBNAME).a
PPU_SRCS = \
	FUCrc32.cpp \
	FUDaeEnum.cpp \
	FUDaeParser.cpp \
	FUDaeWriter.cpp \
	FUDateTime.cpp \
	FUDebug.cpp \
	FUFile.cpp \
	FUFileManager.cpp \
	FULogFile.cpp \
	FUObject.cpp \
	FUStringConversion.cpp \
	FUXmlParser.cpp \
	FUXmlWriter.cpp \
	StdAfx.cpp

include ../../MakeRules
