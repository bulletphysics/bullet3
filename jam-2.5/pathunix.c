/*
 * Copyright 1993-2002 Christopher Seiwald and Perforce Software, Inc.
 *
 * This file is part of Jam - see jam.c for Copyright information.
 */

/*
 * pathunix.c - manipulate file names on UNIX, NT, OS2, AmigaOS
 *
 * External routines:
 *
 *	path_parse() - split a file name into dir/base/suffix/member
 *	path_build() - build a filename given dir/base/suffix/member
 *	path_parent() - make a PATHNAME point to its parent dir
 *
 * File_parse() and path_build() just manipuate a string and a structure;
 * they do not make system calls.
 *
 * 04/08/94 (seiwald) - Coherent/386 support added.
 * 12/26/93 (seiwald) - handle dir/.suffix properly in path_build()
 * 12/19/94 (mikem) - solaris string table insanity support
 * 12/21/94 (wingerd) Use backslashes for pathnames - the NT way.
 * 02/14/95 (seiwald) - parse and build /xxx properly
 * 02/23/95 (wingerd) Compilers on NT can handle "/" in pathnames, so we
 *                    should expect hdr searches to come up with strings
 *                    like "thing/thing.h". So we need to test for "/" as
 *                    well as "\" when parsing pathnames.
 * 03/16/95 (seiwald) - fixed accursed typo on line 69.
 * 05/03/96 (seiwald) - split from filent.c, fileunix.c
 * 12/20/96 (seiwald) - when looking for the rightmost . in a file name,
 *		      don't include the archive member name.
 * 01/13/01 (seiwald) - turn off \ handling on UNIX, on by accident
 * 11/04/02 (seiwald) - const-ing for string literals
 */

# include "jam.h"
# include "pathsys.h"

# ifdef USE_PATHUNIX

/*
 * path_parse() - split a file name into dir/base/suffix/member
 */

void
path_parse( 
	const char *file,
	PATHNAME *f )
{
	const char *p, *q;
	const char *end;
	
	memset( (char *)f, 0, sizeof( *f ) );

	/* Look for <grist> */

	if( file[0] == '<' && ( p = strchr( file, '>' ) ) )
	{
	    f->f_grist.ptr = file;
	    f->f_grist.len = p - file;
	    file = p + 1;
	}

	/* Look for dir/ */

	p = strrchr( file, '/' );

# if PATH_DELIM == '\\'
	/* On NT, look for dir\ as well */
	{
	    char *p1 = strrchr( file, '\\' );
	    p = p1 > p ? p1 : p;
	}
# endif

	if( p )
	{
	    f->f_dir.ptr = file;
	    f->f_dir.len = p - file;
	
	    /* Special case for / - dirname is /, not "" */

	    if( !f->f_dir.len )
		f->f_dir.len = 1;

# if PATH_DELIM == '\\'
	    /* Special case for D:/ - dirname is D:/, not "D:" */

	    if( f->f_dir.len == 2 && file[1] == ':' )
		f->f_dir.len = 3;
# endif

	    file = p + 1;
	}

	end = file + strlen( file );

	/* Look for (member) */

	if( ( p = strchr( file, '(' ) ) && end[-1] == ')' )
	{
	    f->f_member.ptr = p + 1;
	    f->f_member.len = end - p - 2;
	    end = p;
	} 

	/* Look for .suffix */
	/* This would be memrchr() */

	p = 0;
	q = file;

	while( q = (char *)memchr( q, '.', end - q ) )
	    p = q++;

	if( p )
	{
	    f->f_suffix.ptr = p;
	    f->f_suffix.len = end - p;
	    end = p;
	}

	/* Leaves base */

	f->f_base.ptr = file;
	f->f_base.len = end - file;
}

/*
 * path_build() - build a filename given dir/base/suffix/member
 */

void
path_build(
	PATHNAME *f,
	char	*file,
	int	binding )
{
	/* Start with the grist.  If the current grist isn't */
	/* surrounded by <>'s, add them. */

	if( f->f_grist.len )
	{
	    if( f->f_grist.ptr[0] != '<' ) *file++ = '<';
	    memcpy( file, f->f_grist.ptr, f->f_grist.len );
	    file += f->f_grist.len;
	    if( file[-1] != '>' ) *file++ = '>';
	}

	/* Don't prepend root if it's . or directory is rooted */

# if PATH_DELIM == '/'

	if( f->f_root.len 
	    && !( f->f_root.len == 1 && f->f_root.ptr[0] == '.' )
	    && !( f->f_dir.len && f->f_dir.ptr[0] == '/' ) )

# else /* unix */

	if( f->f_root.len 
	    && !( f->f_root.len == 1 && f->f_root.ptr[0] == '.' )
	    && !( f->f_dir.len && f->f_dir.ptr[0] == '/' )
	    && !( f->f_dir.len && f->f_dir.ptr[0] == '\\' )
	    && !( f->f_dir.len && f->f_dir.ptr[1] == ':' ) )

# endif /* unix */

	{
	    memcpy( file, f->f_root.ptr, f->f_root.len );
	    file += f->f_root.len;
	    *file++ = PATH_DELIM;
	}

	if( f->f_dir.len )
	{
	    memcpy( file, f->f_dir.ptr, f->f_dir.len );
	    file += f->f_dir.len;
	}

	/* UNIX: Put / between dir and file */
	/* NT:   Put \ between dir and file */

	if( f->f_dir.len && ( f->f_base.len || f->f_suffix.len ) )
	{
	    /* UNIX: Special case for dir \ : don't add another \ */
	    /* NT:   Special case for dir / : don't add another / */

# if PATH_DELIM == '\\'
	    if( !( f->f_dir.len == 3 && f->f_dir.ptr[1] == ':' ) )
# endif
		if( !( f->f_dir.len == 1 && f->f_dir.ptr[0] == PATH_DELIM ) )
		    *file++ = PATH_DELIM;
	}

	if( f->f_base.len )
	{
	    memcpy( file, f->f_base.ptr, f->f_base.len );
	    file += f->f_base.len;
	}

	if( f->f_suffix.len )
	{
	    memcpy( file, f->f_suffix.ptr, f->f_suffix.len );
	    file += f->f_suffix.len;
	}

	if( f->f_member.len )
	{
	    *file++ = '(';
	    memcpy( file, f->f_member.ptr, f->f_member.len );
	    file += f->f_member.len;
	    *file++ = ')';
	}
	*file = 0;
}

/*
 *	path_parent() - make a PATHNAME point to its parent dir
 */

void
path_parent( PATHNAME *f )
{
	/* just set everything else to nothing */

	f->f_base.ptr =
	f->f_suffix.ptr =
	f->f_member.ptr = "";

	f->f_base.len = 
	f->f_suffix.len = 
	f->f_member.len = 0;
}

# endif /* unix, NT, OS/2, AmigaOS */
