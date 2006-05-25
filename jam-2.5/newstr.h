/*
 * Copyright 1993, 1995 Christopher Seiwald.
 *
 * This file is part of Jam - see jam.c for Copyright information.
 */

/*
 * newstr.h - string manipulation routines
 *
 * 11/04/02 (seiwald) - const-ing for string literals
 */

const char *newstr( const char *string );
const char *copystr( const char *s );
void freestr( const char *s );
void donestr();
