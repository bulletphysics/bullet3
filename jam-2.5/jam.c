/*
 * /+\
 * +\	Copyright 1993-2002 Christopher Seiwald and Perforce Software, Inc.
 * \+/
 *
 * This file is part of jam.
 *
 * License is hereby granted to use this software and distribute it
 * freely, as long as this copyright notice is retained and modifications 
 * are clearly marked.
 *
 * ALL WARRANTIES ARE HEREBY DISCLAIMED.
 */

/*
 * jam.c - make redux
 *
 * See Jam.html for usage information.
 *
 * These comments document the code.
 *
 * The top half of the code is structured such:
 *
 *                       jam 
 *                      / | \ 
 *                 +---+  |  \
 *                /       |   \ 
 *         jamgram     option  \ 
 *        /  |   \              \
 *       /   |    \              \
 *      /    |     \             |
 *  scan     |     compile      make
 *   |       |    /  | \       / |  \
 *   |       |   /   |  \     /  |   \
 *   |       |  /    |   \   /   |    \
 * jambase parse     |   rules  search make1
 *                   |           |      |   \
 *                   |           |      |    \
 *                   |           |      |     \
 *               builtins    timestamp command execute
 *                               |
 *                               |
 *                               |
 *                             filesys
 *
 *
 * The support routines are called by all of the above, but themselves
 * are layered thus:
 *
 *                     variable|expand
 *                      /  |   |   |
 *                     /   |   |   |
 *                    /    |   |   |
 *                 lists   |   |   pathsys
 *                    \    |   |
 *                     \   |   |
 *                      \  |   |
 *                     newstr  |
 *                        \    |
 *                         \   |
 *                          \  |
 *                          hash
 *
 * Roughly, the modules are:
 *
 *	builtins.c - jam's built-in rules
 *	command.c - maintain lists of commands
 *	compile.c - compile parsed jam statements
 *	execunix.c - execute a shell script on UNIX
 *	execvms.c - execute a shell script, ala VMS
 *	expand.c - expand a buffer, given variable values
 *	file*.c - scan directories and archives on *
 *	hash.c - simple in-memory hashing routines 
 *	headers.c - handle #includes in source files
 *	jambase.c - compilable copy of Jambase
 *	jamgram.y - jam grammar
 *	lists.c - maintain lists of strings
 *	make.c - bring a target up to date, once rules are in place
 *	make1.c - execute command to bring targets up to date
 *	newstr.c - string manipulation routines
 *	option.c - command line option processing
 *	parse.c - make and destroy parse trees as driven by the parser
 *	path*.c - manipulate file names on *
 *	hash.c - simple in-memory hashing routines 
 *	regexp.c - Henry Spencer's regexp
 *	rules.c - access to RULEs, TARGETs, and ACTIONs
 *	scan.c - the jam yacc scanner
 *	search.c - find a target along $(SEARCH) or $(LOCATE) 
 *	timestamp.c - get the timestamp of a file or archive member
 *	variable.c - handle jam multi-element variables
 *
 * 05/04/94 (seiwald) - async multiprocess (-j) support
 * 02/08/95 (seiwald) - -n implies -d2.
 * 02/22/95 (seiwald) - -v for version info.
 * 09/11/00 (seiwald) - PATCHLEVEL folded into VERSION.
 * 01/10/01 (seiwald) - pathsys.h split from filesys.h
 * 01/21/02 (seiwald) - new -q to quit quickly on build failure
 * 03/16/02 (seiwald) - support for -g (reorder builds by source time)
 * 09/19/02 (seiwald) - new -d displays
 * 10/22/02 (seiwald) - list_new() now does its own newstr()/copystr()
 * 11/04/02 (seiwald) - const-ing for string literals
 */

# include "jam.h"
# include "option.h"
# include "patchlevel.h"

/* These get various function declarations. */

# include "lists.h"
# include "parse.h"
# include "variable.h"
# include "compile.h"
# include "builtins.h"
# include "rules.h"
# include "newstr.h"
# include "scan.h"
# include "timestamp.h"
# include "make.h"

/* Macintosh is "special" */

# ifdef OS_MAC
# include <QuickDraw.h>
# endif

/* And UNIX for this */

# ifdef unix
# include <sys/utsname.h>
# endif

struct globs globs = {
	0,			/* noexec */
	1,			/* jobs */
	0,			/* quitquick */
	0,			/* newestfirst */
# ifdef OS_MAC
	{ 0 },			/* display - suppress actions output */
# else
	{ 0, 1 }, 		/* display actions  */
# endif
	0			/* output commands, not run them */
} ;

/* Symbols to be defined as true for use in Jambase */

static const char *othersyms[] = { OSMAJOR, OSMINOR, OSPLAT, JAMVERSYM, 0 } ;

/* Known for sure: 
 *	mac needs arg_enviro
 *	OS2 needs extern environ
 */

# ifdef OS_MAC
# define use_environ arg_environ
# ifdef MPW
QDGlobals qd;
# endif
# endif

# ifndef use_environ
# define use_environ environ
# if !defined( __WATCOM__ ) && !defined( OS_OS2 ) && !defined( OS_NT ) 
extern char **environ;
# endif
# endif

main( int argc, char **argv, char **arg_environ )
{
	int		n;
	const char	*s;
	struct option	optv[N_OPTS];
	const char	*all = "all";
	int		anyhow = 0;
	int		status;

# ifdef OS_MAC
	InitGraf(&qd.thePort);
# endif

	argc--, argv++;

	if( ( n = getoptions( argc, argv, "d:j:f:gs:t:ano:qv", optv ) ) < 0 )
	{
	    printf( "\nusage: jam [ options ] targets...\n\n" );

            printf( "-a      Build all targets, even if they are current.\n" );
            printf( "-dx     Display (a)actions (c)causes (d)dependencies\n" );
	    printf( "        (m)make tree (x)commands (0-9) debug levels.\n" );
            printf( "-fx     Read x instead of Jambase.\n" );
	    printf( "-g      Build from newest sources first.\n" );
            printf( "-jx     Run up to x shell commands concurrently.\n" );
            printf( "-n      Don't actually execute the updating actions.\n" );
            printf( "-ox     Write the updating actions to file x.\n" );
            printf( "-q      Quit quickly as soon as a target fails.\n" );
	    printf( "-sx=y   Set variable x=y, overriding environment.\n" );
            printf( "-tx     Rebuild x, even if it is up-to-date.\n" );
            printf( "-v      Print the version of jam and exit.\n\n" );

	    exit( EXITBAD );
	}

	argc -= n, argv += n;

	/* Version info. */

	if( ( s = getoptval( optv, 'v', 0 ) ) )
	{
	    printf( "Jam %s. %s. ", VERSION, OSMINOR );
	    printf( "Copyright 1993-2002 Christopher Seiwald.\n" );

	    return EXITOK;
	}

	/* Pick up interesting options */

	if( ( s = getoptval( optv, 'n', 0 ) ) )
	    globs.noexec++, DEBUG_MAKE = DEBUG_MAKEQ = DEBUG_EXEC = 1; 

	if( ( s = getoptval( optv, 'q', 0 ) ) )
	    globs.quitquick = 1;

	if( ( s = getoptval( optv, 'a', 0 ) ) )
	    anyhow++;

	if( ( s = getoptval( optv, 'j', 0 ) ) )
	    globs.jobs = atoi( s );

	if( ( s = getoptval( optv, 'g', 0 ) ) )
	    globs.newestfirst = 1;

	/* Turn on/off debugging */

	for( n = 0; s = getoptval( optv, 'd', n ); n++ )
	{
	    int i = atoi( s );

	    /* First -d, turn off defaults. */

	    if( !n )
		DEBUG_MAKE = DEBUG_MAKEQ = DEBUG_EXEC = 0;

	    /* n turns on levels 1-n */
	    /* +n turns on level n */
	    /* c turns on named display c */

	    if( i < 0 || i >= DEBUG_MAX )
	    {
		printf( "Invalid debug level '%s'.\n", s );
	    }
	    else if( *s == '+' )
	    {
		globs.debug[i] = 1;
	    }
	    else if( i ) while( i )
	    {
		globs.debug[i--] = 1;
	    }
	    else while( *s ) switch( *s++ )
	    {
	    case 'a': DEBUG_MAKE = DEBUG_MAKEQ = 1; break;
	    case 'c': DEBUG_CAUSES = 1; break;
	    case 'd': DEBUG_DEPENDS = 1; break;
	    case 'm': DEBUG_MAKEPROG = 1; break;
	    case 'x': DEBUG_EXEC = 1; break;
	    case '0': break;
	    default: printf( "Invalid debug flag '%c'.\n", s[-1] );
	    }
	}

	/* Set JAMDATE first */

	{
	    char buf[ 128 ];
	    time_t clock;
	    time( &clock );
	    strcpy( buf, ctime( &clock ) );

	    /* Trim newline from date */

	    if( strlen( buf ) == 25 )
		buf[ 24 ] = 0;

	    var_set( "JAMDATE", list_new( L0, buf, 0 ), VAR_SET );
	}

	/* And JAMUNAME */
# ifdef unix
	{
	    struct utsname u;

	    if( uname( &u ) >= 0 )
	    {
		LIST *l = L0;
		l = list_new( l, u.machine, 0 );
		l = list_new( l, u.version, 0 );
		l = list_new( l, u.release, 0 );
		l = list_new( l, u.nodename, 0 );
		l = list_new( l, u.sysname, 0 );
		var_set( "JAMUNAME", l, VAR_SET );
	    }
	}
# endif /* unix */

	/*
	 * Jam defined variables OS, OSPLAT
	 */

	var_defines( othersyms );

	/* load up environment variables */

	var_defines( (const char **)use_environ );

	/* Load up variables set on command line. */

	for( n = 0; s = getoptval( optv, 's', n ); n++ )
	{
	    const char *symv[2];
	    symv[0] = s;
	    symv[1] = 0;
	    var_defines( symv );
	}

	/* Initialize built-in rules */

	load_builtins();

	/* Parse ruleset */

	for( n = 0; s = getoptval( optv, 'f', n ); n++ )
	    parse_file( s );

	if( !n )
	    parse_file( "+" );

	status = yyanyerrors();

	/* Manually touch -t targets */

	for( n = 0; s = getoptval( optv, 't', n ); n++ )
	    touchtarget( s );

	/* If an output file is specified, set globs.cmdout to that */

	if( s = getoptval( optv, 'o', 0 ) )
	{
	    if( !( globs.cmdout = fopen( s, "w" ) ) )
	    {
		printf( "Failed to write to '%s'\n", s );
		exit( EXITBAD );
	    }
	    globs.noexec++;
	}

	/* Now make target */

	if( !argc )
	    status |= make( 1, &all, anyhow );
	else
	    status |= make( argc, (const char **)argv, anyhow );

	/* Widely scattered cleanup */

	var_done();
	donerules();
	donestamps();
	donestr();

	/* close cmdout */

	if( globs.cmdout )
	    fclose( globs.cmdout );

	return status ? EXITBAD : EXITOK;
}
