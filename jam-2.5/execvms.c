/* 
 * Copyright 1993, 1995 Christopher Seiwald.
 *
 * This file is part of Jam - see jam.c for Copyright information.
 */

/*
 * execvms.c - execute a shell script, ala VMS
 *
 * The approach is this:
 *
 *	If the command is a single line, and shorter than WRTLEN (what we 
 *	believe to be the maximum line length), we just system() it.
 *
 *	If the command is multi-line, or longer than WRTLEN, we write the 
 *	command block to a temp file, splitting long lines (using "-" at 
 *	the end of the line to indicate contiuation), and then source that 
 *	temp file.  We use special logic to make sure we don't continue in
 *	the middle of a quoted string.
 *
 * 05/04/94 (seiwald) - async multiprocess interface; noop on VMS
 * 12/20/96 (seiwald) - rewritten to handle multi-line commands well
 * 01/14/96 (seiwald) - don't put -'s between "'s
 * 01/20/00 (seiwald) - Upgraded from K&R to ANSI C
 */

# include "jam.h"
# include "lists.h"
# include "execcmd.h"

# ifdef OS_VMS

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iodef.h>
#include <ssdef.h>
#include <descrip.h>
#include <dvidef.h>
#include <clidef.h>

#define WRTLEN 240

#define MIN( a, b )	((a) < (b) ? (a) : (b))

/* 1 for the @ and 4 for the .com */

char tempnambuf[ L_tmpnam + 1 + 4 ] = {0};

void
execcmd( 
	char *string,
	void (*func)( void *closure, int status ),
	void *closure,
	LIST *shell )
{
	char *s, *e, *p;
	int rstat = EXEC_CMD_OK;
	int status;

	/* See if string is more than one line */
	/* discounting leading/trailing white space */

	for( s = string; *s && isspace( *s ); s++ )
		;

	e = p = strchr( s, '\n' );

	while( p && isspace( *p ) )
		++p;

	/* If multi line or long, write to com file. */
	/* Otherwise, exec directly. */

	if( p && *p || e - s > WRTLEN )
	{
	    FILE *f;

	    /* Create temp file invocation "@sys$scratch:tempfile.com" */

	    if( !*tempnambuf )
	    {
		tempnambuf[0] = '@';
		(void)tmpnam( tempnambuf + 1 );
		strcat( tempnambuf, ".com" );
	    }
	    
	    /* Open tempfile */

	    if( !( f = fopen( tempnambuf + 1, "w" ) ) )
	    {
		printf( "can't open command file\n" );
		(*func)( closure, EXEC_CMD_FAIL );
		return;
	    }

	    /* For each line of the string */

	    while( *string )
	    {
		char *s = strchr( string, '\n' );
		int len = s ? s + 1 - string : strlen( string );

		fputc( '$', f );

		/* For each chunk of a line that needs to be split */

		while( len > 0 )
		{
		    char *q = string;
		    char *qe = string + MIN( len, WRTLEN );
		    char *qq = q;
		    int quote = 0;

		    /* Look for matching "'s */

		    for( ; q < qe; q++ )
			if( *q == '"' && ( quote = !quote ) )
			    qq = q;

		    /* Back up to opening quote, if in one */

		    if( quote )
			q = qq;

		    fwrite( string, ( q - string ), 1, f );

		    len -= ( q - string );
		    string = q;

		    if( len )
		    {
			fputc( '-', f );
			fputc( '\n', f );
		    }
		}
	    }

	    fclose( f );

	    status = system( tempnambuf ) & 0x07;

	    unlink( tempnambuf + 1 );
	}
	else
	{
	    /* Execute single line command */
	    /* Strip trailing newline before execing */
	    if( e ) *e = 0;
	    status = system( s ) & 0x07;
	}

	/* Fail for error or fatal error */
	/* OK on OK, warning, or info exit */

	if( status == 2 || status == 4 )
	    rstat = EXEC_CMD_FAIL;

	(*func)( closure, rstat );
}

int 
execwait()
{
	return 0;
}

# endif /* VMS */
