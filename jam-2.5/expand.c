/*
 * Copyright 1993-2002 Christopher Seiwald and Perforce Software, Inc.
 *
 * This file is part of Jam - see jam.c for Copyright information.
 */

/*
 * expand.c - expand a buffer, given variable values
 *
 * External routines:
 *
 *	var_expand() - variable-expand input string into list of strings
 *
 * Internal routines:
 *
 *	var_edit_parse() - parse : modifiers into PATHNAME structure
 *	var_edit_file() - copy input target name to output, modifying filename
 *	var_edit_shift() - do upshift/downshift mods
 *
 * 01/25/94 (seiwald) - $(X)$(UNDEF) was expanding like plain $(X)
 * 04/13/94 (seiwald) - added shorthand L0 for null list pointer
 * 01/20/00 (seiwald) - Upgraded from K&R to ANSI C
 * 01/11/01 (seiwald) - added support for :E=emptyvalue, :J=joinval
 * 01/13/01 (seiwald) - :UDJE work on non-filename strings
 * 02/19/01 (seiwald) - make $($(var):J=x) join multiple values of var
 * 01/25/02 (seiwald) - fixed broken $(v[1-]), by ian godin
 * 10/22/02 (seiwald) - list_new() now does its own newstr()/copystr()
 * 11/04/02 (seiwald) - const-ing for string literals
 * 12/30/02 (armstrong) - fix out-of-bounds access in var_expand()
 */

# include "jam.h"
# include "lists.h"
# include "variable.h"
# include "expand.h"
# include "pathsys.h"
# include "newstr.h"

typedef struct {
	PATHNAME	f;		/* :GDBSMR -- pieces */
	char		parent;		/* :P -- go to parent directory */
	char		filemods;	/* one of the above applied */
	char		downshift;	/* :L -- downshift result */
	char		upshift;	/* :U -- upshift result */
	PATHPART	empty;		/* :E -- default for empties */
	PATHPART	join;		/* :J -- join list with char */
} VAR_EDITS ;

static void var_edit_parse( const char *mods, VAR_EDITS *edits );
static void var_edit_file( const char *in, char *out, VAR_EDITS *edits );
static void var_edit_shift( char *out, VAR_EDITS *edits );

# define MAGIC_COLON	'\001'
# define MAGIC_LEFT	'\002'
# define MAGIC_RIGHT	'\003'

/*
 * var_expand() - variable-expand input string into list of strings
 *
 * Would just copy input to output, performing variable expansion, 
 * except that since variables can contain multiple values the result
 * of variable expansion may contain multiple values (a list).  Properly
 * performs "product" operations that occur in "$(var1)xxx$(var2)" or
 * even "$($(var2))".
 *
 * Returns a newly created list.
 */

LIST *
var_expand( 
	LIST		*l,
	const char 	*in,
	const char 	*end,
	LOL		*lol,
	int		cancopyin )
{
	char out_buf[ MAXSYM ];
	char *out = out_buf;
	const char *inp = in;
	char *ov;		/* for temp copy of variable in outbuf */
	int depth;

	if( DEBUG_VAREXP )
	    printf( "expand '%.*s'\n", end - in, in );

	/* This gets alot of cases: $(<) and $(>) */

	if( end - in == 4 && in[0] == '$' && in[1] == '(' && in[3] == ')' )
	{
	    switch( in[2] )
	    {
	    case '1':
	    case '<':
		return list_copy( l, lol_get( lol, 0 ) );

	    case '2':
	    case '>':
		return list_copy( l, lol_get( lol, 1 ) );
	    }
	}

	/* Just try simple copy of in to out. */

	while( in < end )
	    if( ( *out++ = *in++ ) == '$' && *in == '(' ) 
		goto expand;

	/* No variables expanded - just add copy of input string to list. */

	/* Cancopyin is an optimization: if the input was already a list */
	/* item, we can use the copystr() to put it on the new list. */
	/* Otherwise, we use the slower newstr(). */

	*out = '\0';

	if( cancopyin )
	    return list_new( l, inp, 1 );
	else
	    return list_new( l, out_buf, 0 );

    expand:
	/*
	 * Input so far (ignore blanks):
	 *
	 *	stuff-in-outbuf $(variable) remainder
	 *			 ^	             ^
	 *			 in		     end
	 * Output so far:
	 *
	 *	stuff-in-outbuf $
	 *	^	         ^
	 *	out_buf          out
	 *
	 *
	 * We just copied the $ of $(...), so back up one on the output.
	 * We now find the matching close paren, copying the variable and
	 * modifiers between the $( and ) temporarily into out_buf, so that
	 * we can replace :'s with MAGIC_COLON.  This is necessary to avoid
	 * being confused by modifier values that are variables containing
	 * :'s.  Ugly.
	 */

	depth = 1;
	out--, in++;
	ov = out;

	while( in < end && depth )
	{
	    switch( *ov++ = *in++ )
	    {
	    case '(': depth++; break;
	    case ')': depth--; break;
	    case ':': ov[-1] = MAGIC_COLON; break;
	    case '[': ov[-1] = MAGIC_LEFT; break;
	    case ']': ov[-1] = MAGIC_RIGHT; break;
	    }
	}

	/* Copied ) - back up. */

	ov--;

	/*
	 * Input so far (ignore blanks):
	 *
	 *	stuff-in-outbuf $(variable) remainder
	 *			            ^        ^
	 *			            in       end
	 * Output so far:
	 *
	 *	stuff-in-outbuf variable
	 *	^	        ^       ^
	 *	out_buf         out	ov
	 *
	 * Later we will overwrite 'variable' in out_buf, but we'll be
	 * done with it by then.  'variable' may be a multi-element list, 
	 * so may each value for '$(variable element)', and so may 'remainder'.
	 * Thus we produce a product of three lists.
	 */

	{
	    LIST *variables = 0;
	    LIST *remainder = 0;
	    LIST *vars;

	    /* Recursively expand variable name & rest of input */

	    if( out < ov )
		variables = var_expand( L0, out, ov, lol, 0 );
	    if( in < end )
		remainder = var_expand( L0, in, end, lol, 0 );

	    /* Now produce the result chain */

	    /* For each variable name */

	    for( vars = variables; vars; vars = list_next( vars ) )
	    {
		LIST *value, *evalue = 0;
		char *colon;
		char *bracket;
		char varname[ MAXSYM ];
		int sub1 = 0, sub2 = -1;
		VAR_EDITS edits;

		/* Look for a : modifier in the variable name */
		/* Must copy into varname so we can modify it */

		strcpy( varname, vars->string );

		if( colon = strchr( varname, MAGIC_COLON ) )
		{
		    *colon = '\0';
		    var_edit_parse( colon + 1, &edits );
		}

		/* Look for [x-y] subscripting */
		/* sub1 is x (0 default) */
		/* sub2 is length (-1 means forever) */

		if( bracket = strchr( varname, MAGIC_LEFT ) )
		{
		    char *dash;

		    if( dash = strchr( bracket + 1, '-' ) )
			*dash = '\0';

		    sub1 = atoi( bracket + 1 ) - 1;

		    if( !dash )
			sub2 = 1;
		    else if( !dash[1] || dash[1] == MAGIC_RIGHT )
			sub2 = -1;
		    else
			sub2 = atoi( dash + 1 ) - sub1;

		    *bracket = '\0';
		}

		/* Get variable value, specially handling $(<), $(>), $(n) */
		
		if( varname[0] == '<' && !varname[1] )
		    value = lol_get( lol, 0 );
		else if( varname[0] == '>' && !varname[1] )
		    value = lol_get( lol, 1 );
		else if( varname[0] >= '1' && varname[0] <= '9' && !varname[1] )
		    value = lol_get( lol, varname[0] - '1' );
		else 
		    value = var_get( varname );

		/* The fast path: $(x) - just copy the variable value. */
		/* This is only an optimization */

		if( out == out_buf && !bracket && !colon && in == end )
		{
		    l = list_copy( l, value );
		    continue;
		}

		/* Handle start subscript */

		while( sub1 > 0 && value )
		    --sub1, value = list_next( value );

		/* Empty w/ :E=default? */

		if( !value && colon && edits.empty.ptr )
		    evalue = value = list_new( L0, edits.empty.ptr, 0 );

		/* For each variable value */

		for( ; value; value = list_next( value ) )
		{
		    LIST *rem;
		    char *out1;

		    /* Handle end subscript (length actually) */

		    if( sub2 >= 0 && --sub2 < 0 )
			break;

		    /* Apply : mods, if present */

		    if( colon && edits.filemods )
			var_edit_file( value->string, out, &edits );
		    else
			strcpy( out, value->string );

		    if( colon && ( edits.upshift || edits.downshift ) )
			var_edit_shift( out, &edits );

		    /* Handle :J=joinval */
		    /* If we have more values for this var, just */
		    /* keep appending them (with the join value) */
		    /* rather than creating separate LIST elements. */

		    if( colon && edits.join.ptr && 
		      ( list_next( value ) || list_next( vars ) ) )
		    {
			out += strlen( out );
			strcpy( out, edits.join.ptr );
			out += strlen( out );
			continue;
		    }

		    /* If no remainder, append result to output chain. */

		    if( in == end )
		    {
			l = list_new( l, out_buf, 0 );
			continue;
		    }

		    /* For each remainder, append the complete string */
		    /* to the output chain. */
		    /* Remember the end of the variable expansion so */
		    /* we can just tack on each instance of 'remainder' */

		    out1 = out + strlen( out );

		    for( rem = remainder; rem; rem = list_next( rem ) )
		    {
			strcpy( out1, rem->string );
			l = list_new( l, out_buf, 0 );
		    }
		}

		/* Toss used empty */

		if( evalue )
		    list_free( evalue );
	    }

	    /* variables & remainder were gifts from var_expand */
	    /* and must be freed */

	    if( variables )
		list_free( variables );
	    if( remainder)
		list_free( remainder );

	    if( DEBUG_VAREXP )
	    {
		printf( "expanded to " );
		list_print( l );
		printf( "\n" );
	    }

	    return l;
	}
}

/*
 * var_edit_parse() - parse : modifiers into PATHNAME structure
 *
 * The : modifiers in a $(varname:modifier) currently support replacing
 * or omitting elements of a filename, and so they are parsed into a 
 * PATHNAME structure (which contains pointers into the original string).
 *
 * Modifiers of the form "X=value" replace the component X with
 * the given value.  Modifiers without the "=value" cause everything 
 * but the component X to be omitted.  X is one of:
 *
 *	G <grist>
 *	D directory name
 *	B base name
 *	S .suffix
 *	M (member)
 *	R root directory - prepended to whole path
 *
 * This routine sets:
 *
 *	f->f_xxx.ptr = 0
 *	f->f_xxx.len = 0
 *		-> leave the original component xxx
 *
 *	f->f_xxx.ptr = string
 *	f->f_xxx.len = strlen( string )
 *		-> replace component xxx with string
 *
 *	f->f_xxx.ptr = ""
 *	f->f_xxx.len = 0
 *		-> omit component xxx
 *
 * var_edit_file() below and path_build() obligingly follow this convention.
 */

static void
var_edit_parse(
	const char	*mods,
	VAR_EDITS	*edits )
{
	int havezeroed = 0;
	memset( (char *)edits, 0, sizeof( *edits ) );

	while( *mods )
	{
	    char *p;
	    PATHPART *fp;

	    switch( *mods++ )
	    {
	    case 'L': edits->downshift = 1; continue;
	    case 'U': edits->upshift = 1; continue;
	    case 'P': edits->parent = edits->filemods = 1; continue;
	    case 'E': fp = &edits->empty; goto strval;
	    case 'J': fp = &edits->join; goto strval;
	    case 'G': fp = &edits->f.f_grist; goto fileval;
	    case 'R': fp = &edits->f.f_root; goto fileval;
	    case 'D': fp = &edits->f.f_dir; goto fileval;
	    case 'B': fp = &edits->f.f_base; goto fileval;
	    case 'S': fp = &edits->f.f_suffix; goto fileval;
	    case 'M': fp = &edits->f.f_member; goto fileval;

	    default: return; /* should complain, but so what... */
	    }

	fileval:

	    /* Handle :CHARS, where each char (without a following =) */
	    /* selects a particular file path element.  On the first such */
	    /* char, we deselect all others (by setting ptr = "", len = 0) */
	    /* and for each char we select that element (by setting ptr = 0) */

	    edits->filemods = 1;

	    if( *mods != '=' )
	    {
		int i;

		if( !havezeroed++ )
		    for( i = 0; i < 6; i++ )
		{
		    edits->f.part[ i ].len = 0;
		    edits->f.part[ i ].ptr = "";
		}

		fp->ptr = 0;
		continue;
	    }

	strval:

	    /* Handle :X=value, or :X */

	    if( *mods != '=' )
	    {
		fp->ptr = "";
		fp->len = 0;
	    }
	    else if( p = strchr( mods, MAGIC_COLON ) )
	    {
		*p = 0;
		fp->ptr = ++mods;
		fp->len = p - mods;
		mods = p + 1;
	    }
	    else
	    {
		fp->ptr = ++mods;
		fp->len = strlen( mods );
		mods += fp->len;
	    }
	}
}

/*
 * var_edit_file() - copy input target name to output, modifying filename
 */
	
static void
var_edit_file( 
	const char *in,
	char	*out,
	VAR_EDITS *edits )
{
	PATHNAME pathname;

	/* Parse apart original filename, putting parts into "pathname" */

	path_parse( in, &pathname );

	/* Replace any pathname with edits->f */

	if( edits->f.f_grist.ptr )
	    pathname.f_grist = edits->f.f_grist;

	if( edits->f.f_root.ptr )
	    pathname.f_root = edits->f.f_root;

	if( edits->f.f_dir.ptr )
	    pathname.f_dir = edits->f.f_dir;

	if( edits->f.f_base.ptr )
	    pathname.f_base = edits->f.f_base;

	if( edits->f.f_suffix.ptr )
	    pathname.f_suffix = edits->f.f_suffix;

	if( edits->f.f_member.ptr )
	    pathname.f_member = edits->f.f_member;

	/* If requested, modify pathname to point to parent */

	if( edits->parent )
	    path_parent( &pathname );

	/* Put filename back together */

	path_build( &pathname, out, 0 );
}

/*
 * var_edit_shift() - do upshift/downshift mods
 */

static void
var_edit_shift( 
	char	*out,
	VAR_EDITS *edits )
{
	/* Handle upshifting, downshifting now */

	if( edits->upshift )
	{
	    for( ; *out; ++out )
		*out = toupper( *out );
	}
	else if( edits->downshift )
	{
	    for( ; *out; ++out )
		*out = tolower( *out );
	}
}
