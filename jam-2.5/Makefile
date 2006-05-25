# Makefile for jam

CC = cc
CFLAGS =
EXENAME = ./jam0
TARGET = -o $(EXENAME)

# Special flavors - uncomment appropriate lines

# NCR seems to have a broken readdir() -- use gnu
#CC = gcc

# AIX needs -lbsd, and has no identifying cpp symbol
# Use _AIX41 if you're not on 3.2 anymore.
#LINKLIBS = -lbsd
#CFLAGS = -D_AIX

# NT (with Microsoft compiler)
# Use FATFS if building on a DOS FAT file system
#Lib = $(MSVCNT)/lib
#Include = $(MSVCNT)/include
#CC = cl /nologo
#CFLAGS = -I $(Include) -DNT 
#TARGET = /Fejam0
#LINKLIBS = $(Lib)/oldnames.lib $(Lib)/kernel32.lib $(Lib)/libc.lib
#EXENAME = .\jam0.exe

# NT (with Microsoft compiler)
# People with DevStudio settings already in shell environment.
#CC = cl /nologo
#CFLAGS = -DNT 
#TARGET = /Fejam0
#EXENAME = .\jam0.exe

# Interix - gcc
#CC = gcc

# Cygwin - gcc & cygwin
#CC = gcc
#CFLAGS = -D__cygwin__

# MingW32
#CC = gcc
#CFLAGS = -DMINGW

# MPEIX
#CC = gcc
#CFLAGS = -I/usr/include -D_POSIX_SOURCE

# QNX rtp (neutrino)
#CC = gcc

SOURCES = \
	builtins.c \
	command.c compile.c execunix.c execvms.c expand.c \
	filent.c fileos2.c fileunix.c filevms.c glob.c hash.c \
	headers.c jam.c jambase.c jamgram.c lists.c make.c make1.c \
	newstr.c option.c parse.c pathunix.c pathvms.c regexp.c \
	rules.c scan.c search.c timestamp.c variable.c

all: $(EXENAME)
	$(EXENAME)

$(EXENAME):
	$(CC) $(TARGET) $(CFLAGS) $(SOURCES) $(LINKLIBS)
