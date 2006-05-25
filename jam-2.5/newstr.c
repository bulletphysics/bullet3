/*
 * Copyright 1993, 1995 Christopher Seiwald.
 *
 * This file is part of Jam - see jam.c for Copyright information.
 */

/*
 * newstr.c - string manipulation routines
 *
 * To minimize string copying, string creation, copying, and freeing
 * is done through newstr.
 *
 * External functions:
 *
 *    newstr() - return a malloc'ed copy of a string
 *    copystr() - return a copy of a string previously returned by newstr()
 *    freestr() - free a string returned by newstr() or copystr()
 *    donestr() - free string tables
 *
 * Once a string is passed to newstr(), the returned string is readonly.
 *
 * This implementation builds a hash table of all strings, so that multiple 
 * calls of newstr() on the same string allocate memory for the string once.
 * Strings are never actually freed.
 *
 * 11/04/02 (seiwald) - const-ing for string literals
 */

# include "jam.h"
# include "newstr.h"
# include "hash.h"

typedef const char *STRING;

static struct hash *strhash = 0;
static int strtotal = 0;

/*
 * newstr() - return a malloc'ed copy of a string
 */

const char *
newstr( const char *string )
{
	STRING str, *s = &str;

	if( !strhash )
	    strhash = hashinit( sizeof( STRING ), "strings" );

	*s = string;

	if( hashenter( strhash, (HASHDATA **)&s ) )
	{
	    int l = strlen( string );
	    char *m = (char *)malloc( l + 1 );

	    if (DEBUG_MEM)
		    printf("newstr: allocating %d bytes\n", l + 1 );

	    strtotal += l + 1;
	    memcpy( m, string, l + 1 );
	    *s = m;
	}

	return *s;
}

/*
 * copystr() - return a copy of a string previously returned by newstr()
 */

const char *
copystr( const char *s )
{
	return s;
}

/*
 * freestr() - free a string returned by newstr() or copystr()
 */

void
freestr( const char *s )
{
}

/*
 * donestr() - free string tables
 */

void
donestr()
{
	hashdone( strhash );

	if( DEBUG_MEM )
	    printf( "%dK in strings\n", strtotal / 1024 );
}
