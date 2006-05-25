/*
 * Copyright 1993-2002 Christopher Seiwald and Perforce Software, Inc.
 *
 * This file is part of Jam - see jam.c for Copyright information.
 */

/*
 * variable.c - handle jam multi-element variables
 *
 * External routines:
 *
 *	var_defines() - load a bunch of variable=value settings
 *	var_string() - expand a string with variables in it
 *	var_get() - get value of a user defined symbol
 *	var_set() - set a variable in jam's user defined symbol table
 *	var_swap() - swap a variable's value with the given one
 *	var_done() - free variable tables
 *
 * Internal routines:
 *
 *	var_enter() - make new var symbol table entry, returning var ptr
 *	var_dump() - dump a variable to stdout
 *
 * 04/13/94 (seiwald) - added shorthand L0 for null list pointer
 * 08/23/94 (seiwald) - Support for '+=' (append to variable)
 * 01/22/95 (seiwald) - split environment variables at blanks or :'s
 * 05/10/95 (seiwald) - split path variables at SPLITPATH (not :)
 * 09/11/00 (seiwald) - defunct var_list() removed
 * 10/22/02 (seiwald) - list_new() now does its own newstr()/copystr()
 * 11/04/02 (seiwald) - const-ing for string literals
 */

# include "jam.h"
# include "lists.h"
# include "parse.h"
# include "variable.h"
# include "expand.h"
# include "hash.h"
# include "newstr.h"

static struct hash *varhash = 0;

/*
 * VARIABLE - a user defined multi-value variable
 */

typedef struct _variable VARIABLE ;

struct _variable {
	const char	*symbol;
	LIST		*value;
} ;

static VARIABLE *var_enter( const char *symbol );
static void var_dump( const char *symbol, LIST *value, const char *what );



/*
 * var_defines() - load a bunch of variable=value settings
 *
 * If variable name ends in PATH, split value at :'s.  
 * Otherwise, split at blanks.
 */

void
var_defines( const char **e )
{
	for( ; *e; e++ )
	{
	    const char *val;

	    /* Just say "no": windows defines this in the env, */
	    /* but we don't want it to override our notion of OS. */

	    if( !strcmp( *e, "OS=Windows_NT" ) )
		continue;

# ifdef OS_MAC
	    /* On the mac (MPW), the var=val is actually var\0val */
	    /* Think different. */
	
	    if( ( val = strchr( *e, '=' ) ) || ( val = *e + strlen( *e ) ) )
# else
	    if( val = strchr( *e, '=' ) )
# endif
	    {
		LIST *l = L0;
		const char *pp, *p;
# ifdef OS_MAC
		char split = ',';
# else
		char split = ' ';	
# endif
		char buf[ MAXSYM ];

		/* Split *PATH at :'s, not spaces */

		if( val - 4 >= *e )
		{
		    if( !strncmp( val - 4, "PATH", 4 ) ||
		        !strncmp( val - 4, "Path", 4 ) ||
		        !strncmp( val - 4, "path", 4 ) )
			    split = SPLITPATH;
		}

		/* Do the split */

		for( pp = val + 1; p = strchr( pp, split ); pp = p + 1 )
		{
		    strncpy( buf, pp, p - pp );
		    buf[ p - pp ] = '\0';
		    l = list_new( l, buf, 0 );
		}

		l = list_new( l, pp, 0 );

		/* Get name */

		strncpy( buf, *e, val - *e );
		buf[ val - *e ] = '\0';

		var_set( buf, l, VAR_SET );
	    }
	}
}

/*
 * var_string() - expand a string with variables in it
 *
 * Copies in to out; doesn't modify targets & sources.
 */

int
var_string(
	const char *in,
	char	*out,
	int	outsize,
	LOL	*lol )
{
	char 	*out0 = out;
	char	*oute = out + outsize - 1;

	while( *in )
	{
	    char	*lastword;
	    int		dollar = 0;

	    /* Copy white space */

	    while( isspace( *in ) )
	    {
		if( out >= oute )
		    return -1;

		*out++ = *in++;
	    }

	    lastword = out;

	    /* Copy non-white space, watching for variables */

	    while( *in && !isspace( *in ) )
	    {
	        if( out >= oute )
		    return -1;

		if( in[0] == '$' && in[1] == '(' )
		    dollar++;

		*out++ = *in++;
	    }

	    /* If a variable encountered, expand it and and embed the */
	    /* space-separated members of the list in the output. */

	    if( dollar )
	    {
		LIST *l = var_expand( L0, lastword, out, lol, 0 );

		out = lastword;

		while( l )
		{
		    int so = strlen( l->string );

		    if( out + so >= oute )
			return -1;

		    strcpy( out, l->string );
		    out += so;

		    /* Separate with space */

		    if( l = list_next( l ) )
			*out++ = ' ';
		}

		list_free( l );
	    }
	}

	if( out >= oute )
	    return -1;

	*out++ = '\0';

	return out - out0;
}

/*
 * var_get() - get value of a user defined symbol
 *
 * Returns NULL if symbol unset.
 */

LIST *
var_get( const char *symbol )
{
	VARIABLE var, *v = &var;

	v->symbol = symbol;

	if( varhash && hashcheck( varhash, (HASHDATA **)&v ) )
	{
	    if( DEBUG_VARGET )
		var_dump( v->symbol, v->value, "get" );
	    return v->value;
	}
    
	return 0;
}

/*
 * var_set() - set a variable in jam's user defined symbol table
 *
 * 'flag' controls the relationship between new and old values of
 * the variable: SET replaces the old with the new; APPEND appends
 * the new to the old; DEFAULT only uses the new if the variable
 * was previously unset.
 *
 * Copies symbol.  Takes ownership of value.
 */

void
var_set(
	const char *symbol,
	LIST	*value,
	int	flag )
{
	VARIABLE *v = var_enter( symbol );

	if( DEBUG_VARSET )
	    var_dump( symbol, value, "set" );

	switch( flag )
	{
	case VAR_SET:
	    /* Replace value */
	    list_free( v->value );
	    v->value = value;
	    break;

	case VAR_APPEND:
	    /* Append value */
	    v->value = list_append( v->value, value );
	    break;

	case VAR_DEFAULT:
	    /* Set only if unset */
	    if( !v->value )
		v->value = value;
	    else
		list_free( value );
	    break;
	}
}

/*
 * var_swap() - swap a variable's value with the given one
 */

LIST *
var_swap(
	const char *symbol,
	LIST	*value )
{
	VARIABLE *v = var_enter( symbol );
	LIST 	 *oldvalue = v->value;

	if( DEBUG_VARSET )
	    var_dump( symbol, value, "set" );

	v->value = value;

	return oldvalue;
}



/*
 * var_enter() - make new var symbol table entry, returning var ptr
 */

static VARIABLE *
var_enter( const char *symbol )
{
	VARIABLE var, *v = &var;

	if( !varhash )
	    varhash = hashinit( sizeof( VARIABLE ), "variables" );

	v->symbol = symbol;
	v->value = 0;

	if( hashenter( varhash, (HASHDATA **)&v ) )
	    v->symbol = newstr( symbol );	/* never freed */

	return v;
}

/*
 * var_dump() - dump a variable to stdout
 */

static void
var_dump(
	const char	*symbol,
	LIST		*value,
	const char	*what )
{
	printf( "%s %s = ", what, symbol );
	list_print( value );
	printf( "\n" );
}

/*
 * var_done() - free variable tables
 */

void
var_done()
{
	hashdone( varhash );
}
