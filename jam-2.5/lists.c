/*
 * Copyright 1993, 1995 Christopher Seiwald.
 *
 * This file is part of Jam - see jam.c for Copyright information.
 */

/*
 * lists.c - maintain lists of strings
 *
 * This implementation essentially uses a singly linked list, but
 * guarantees that the head element of every list has a valid pointer
 * to the tail of the list, so the new elements can efficiently and 
 * properly be appended to the end of a list.
 *
 * To avoid massive allocation, list_free() just tacks the whole freed
 * chain onto freelist and list_new() looks on freelist first for an
 * available list struct.  list_free() does not free the strings in the 
 * chain: it lazily lets list_new() do so.
 *
 * 08/23/94 (seiwald) - new list_append()
 * 09/07/00 (seiwald) - documented lol_*() functions
 * 10/22/02 (seiwald) - list_new() now does its own newstr()/copystr()
 * 11/04/02 (seiwald) - const-ing for string literals
 * 12/09/02 (seiwald) - new list_printq() for writing lists to Jambase
 */

# include "jam.h"
# include "newstr.h"
# include "lists.h"

static LIST *freelist = 0;	/* junkpile for list_free() */

/*
 * list_append() - append a list onto another one, returning total
 */

LIST *
list_append( 
	LIST	*l,
	LIST	*nl )
{
	if( !nl )
	{
	    /* Just return l */
	}
	else if( !l )
	{
	    l = nl;
	}
	else
	{
	    /* Graft two non-empty lists. */
	    l->tail->next = nl;
	    l->tail = nl->tail;
	}

	return l;
}

/*
 * list_new() - tack a string onto the end of a list of strings
 */

LIST *
list_new( 
	LIST	*head,
	const char *string,
	int	copy )
{
	LIST *l;

	if( DEBUG_LISTS )
	    printf( "list > %s <\n", string );

	/* Copy/newstr as needed */

	string = copy ? copystr( string ) : newstr( string );

	/* Get list struct from freelist, if one available.  */
	/* Otherwise allocate. */
	/* If from freelist, must free string first */

	if( freelist )
	{
	    l = freelist;
	    freestr( l->string );
	    freelist = freelist->next;
	}
	else
	{
	    l = (LIST *)malloc( sizeof( *l ) );
	}

	/* If first on chain, head points here. */
	/* If adding to chain, tack us on. */
	/* Tail must point to this new, last element. */

	if( !head ) head = l;
	else head->tail->next = l;
	head->tail = l;
	l->next = 0;

	l->string = string;

	return head;
}

/*
 * list_copy() - copy a whole list of strings (nl) onto end of another (l)
 */

LIST *
list_copy( 
	LIST	*l,
	LIST 	*nl )
{
	for( ; nl; nl = list_next( nl ) )
	    l = list_new( l, nl->string, 1 );

	return l;
}

/*
 * list_sublist() - copy a subset of a list of strings
 */

LIST *
list_sublist( 
	LIST	*l,
	int	start,
	int	count )
{
	LIST	*nl = 0;

	for( ; l && start--; l = list_next( l ) )
	    ;

	for( ; l && count--; l = list_next( l ) )
	    nl = list_new( nl, l->string, 1 );

	return nl;
}

/*
 * list_free() - free a list of strings
 */

void
list_free( LIST	*head )
{
	/* Just tack onto freelist. */

	if( head )
	{
	    head->tail->next = freelist;
	    freelist = head;
	}
}

/*
 * list_print() - print a list of strings to stdout
 */

void
list_print( LIST *l )
{
	for( ; l; l = list_next( l ) )
	    printf( "%s ", l->string );
}

/*
 * list_printq() - print a list of safely quoted strings to a file
 */

void
list_printq( FILE *out, LIST *l )
{
	/* Dump each word, enclosed in "s */
	/* Suitable for Jambase use. */

	for( ; l; l = list_next( l ) )
	{
	    const char *p = l->string;
	    const char *ep = p + strlen( p );
	    const char *op = p;

	    fputc( '\n', out );
	    fputc( '\t', out );
	    fputc( '"', out );

	    /* Any embedded "'s?  Escape them */

	    while( p = (char *)memchr( op, '"',  ep - op ) )
	    {
		fwrite( op, p - op, 1, out );
		fputc( '\\', out );
		fputc( '"', out );
		op = p + 1;
	    }

	    /* Write remainder */

	    fwrite( op, ep - op, 1, out );
	    fputc( '"', out );
	    fputc( ' ', out );
	}
}

/*
 * list_length() - return the number of items in the list
 */

int
list_length( LIST *l )
{
	int n = 0;

	for( ; l; l = list_next( l ), ++n )
	    ;

	return n;
}

/*
 * lol_init() - initialize a LOL (list of lists)
 */

void
lol_init( LOL *lol )
{
	lol->count = 0;
}

/*
 * lol_add() - append a LIST onto an LOL
 */

void
lol_add( 
	LOL	*lol,
	LIST	*l )
{
	if( lol->count < LOL_MAX )
	    lol->list[ lol->count++ ] = l;
}

/*
 * lol_free() - free the LOL and its LISTs
 */

void
lol_free( LOL *lol )
{
	int i;

	for( i = 0; i < lol->count; i++ )
	    list_free( lol->list[i] );

	lol->count = 0;
}

/*
 * lol_get() - return one of the LISTs in the LOL
 */

LIST *
lol_get( 
	LOL	*lol,
	int	i )
{
	return i < lol->count ? lol->list[i] : 0;
}

/*
 * lol_print() - debug print LISTS separated by ":"
 */

void
lol_print( LOL *lol )
{
	int i;

	for( i = 0; i < lol->count; i++ )
	{
	    if( i )
		printf( " : " );
	    list_print( lol->list[i] );
	}
}
