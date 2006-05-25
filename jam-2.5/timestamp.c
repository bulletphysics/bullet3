/*
 * Copyright 1993-2002 Christopher Seiwald and Perforce Software, Inc.
 *
 * This file is part of Jam - see jam.c for Copyright information.
 */

/*
 * timestamp.c - get the timestamp of a file or archive member
 *
 * 09/22/00 (seiwald) - downshift names on OS2, too
 * 01/08/01 (seiwald) - closure param for file_dirscan/file_archscan
 * 11/04/02 (seiwald) - const-ing for string literals
 */

# include "jam.h"
# include "hash.h"
# include "filesys.h"
# include "pathsys.h"
# include "timestamp.h"
# include "newstr.h"

/*
 * BINDING - all known files
 */

typedef struct _binding BINDING;

struct _binding {
	const char	*name;
	short		flags;

# define BIND_SCANNED	0x01	/* if directory or arch, has been scanned */

	short		progress;

# define BIND_INIT	0	/* never seen */
# define BIND_NOENTRY	1	/* timestamp requested but file never found */
# define BIND_SPOTTED	2	/* file found but not timed yet */
# define BIND_MISSING	3	/* file found but can't get timestamp */
# define BIND_FOUND	4	/* file found and time stamped */

	time_t		time;	/* update time - 0 if not exist */
} ;

static struct hash *bindhash = 0;
static void time_enter( void *, const char *, int , time_t  );

static const char *time_progress[] =
{
	"INIT",
	"NOENTRY",
	"SPOTTED",
	"MISSING",
	"FOUND"
} ;


/*
 * timestamp() - return timestamp on a file, if present
 */

void
timestamp( 
	char	*target,
	time_t	*time )
{
	PATHNAME f1, f2;
	BINDING	binding, *b = &binding;
	char buf[ MAXJPATH ];

# ifdef DOWNSHIFT_PATHS
	char path[ MAXJPATH ];
	char *p = path;

	do *p++ = tolower( *target );
	while( *target++ );

	target = path;
# endif 

	if( !bindhash )
	    bindhash = hashinit( sizeof( BINDING ), "bindings" );

	/* Quick path - is it there? */

	b->name = target;
	b->time = b->flags = 0;
	b->progress = BIND_INIT;

	if( hashenter( bindhash, (HASHDATA **)&b ) )
	    b->name = newstr( target );		/* never freed */

	if( b->progress != BIND_INIT )
	    goto afterscanning;

	b->progress = BIND_NOENTRY;

	/* Not found - have to scan for it */

	path_parse( target, &f1 );

	/* Scan directory if not already done so */

	{
	    BINDING binding, *b = &binding;

	    f2 = f1;
	    f2.f_grist.len = 0;
	    path_parent( &f2 );
	    path_build( &f2, buf, 0 );

	    b->name = buf;
	    b->time = b->flags = 0;
	    b->progress = BIND_INIT;

	    if( hashenter( bindhash, (HASHDATA **)&b ) )
		b->name = newstr( buf );	/* never freed */

	    if( !( b->flags & BIND_SCANNED ) )
	    {
		file_dirscan( buf, time_enter, bindhash );
		b->flags |= BIND_SCANNED;
	    }
	}

	/* Scan archive if not already done so */

	if( f1.f_member.len )
	{
	    BINDING binding, *b = &binding;

	    f2 = f1;
	    f2.f_grist.len = 0;
	    f2.f_member.len = 0;
	    path_build( &f2, buf, 0 );

	    b->name = buf;
	    b->time = b->flags = 0;
	    b->progress = BIND_INIT;

	    if( hashenter( bindhash, (HASHDATA **)&b ) )
		b->name = newstr( buf );	/* never freed */

	    if( !( b->flags & BIND_SCANNED ) )
	    {
		file_archscan( buf, time_enter, bindhash );
		b->flags |= BIND_SCANNED;
	    }
	}

    afterscanning:

	if( b->progress == BIND_SPOTTED )
	{
	    if( file_time( b->name, &b->time ) < 0 )
		b->progress = BIND_MISSING;
	    else
		b->progress = BIND_FOUND;
	}

	*time = b->progress == BIND_FOUND ? b->time : 0;
}

static void
time_enter( 
	void		*closure,
	const char	*target,
	int		found,
	time_t		time )
{
	BINDING	binding, *b = &binding;
	struct hash *bindhash = (struct hash *)closure;

# ifdef DOWNSHIFT_PATHS
	char path[ MAXJPATH ];
	char *p = path;

	do *p++ = tolower( *target );
	while( *target++ );

	target = path;
# endif

	b->name = target;
	b->flags = 0;

	if( hashenter( bindhash, (HASHDATA **)&b ) )
	    b->name = newstr( target );		/* never freed */

	b->time = time;
	b->progress = found ? BIND_FOUND : BIND_SPOTTED;

	if( DEBUG_BINDSCAN )
	    printf( "time ( %s ) : %s\n", target, time_progress[b->progress] );
}

/*
 * donestamps() - free timestamp tables
 */

void
donestamps()
{
	hashdone( bindhash );
}
