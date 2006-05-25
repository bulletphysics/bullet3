/*
 * Copyright 1993-2002 Christopher Seiwald and Perforce Software, Inc.
 *
 * This file is part of Jam - see jam.c for Copyright information.
 */

/*
 * fileunix.c - manipulate file names and scan directories on UNIX/AmigaOS
 *
 * External routines:
 *
 *	file_dirscan() - scan a directory for files
 *	file_time() - get timestamp of file, if not done by file_dirscan()
 *	file_archscan() - scan an archive for files
 *
 * File_dirscan() and file_archscan() call back a caller provided function
 * for each file found.  A flag to this callback function lets file_dirscan()
 * and file_archscan() indicate that a timestamp is being provided with the
 * file.   If file_dirscan() or file_archscan() do not provide the file's
 * timestamp, interested parties may later call file_time().
 *
 * 04/08/94 (seiwald) - Coherent/386 support added.
 * 12/19/94 (mikem) - solaris string table insanity support
 * 02/14/95 (seiwald) - parse and build /xxx properly
 * 05/03/96 (seiwald) - split into pathunix.c
 * 11/21/96 (peterk) - BEOS does not have Unix-style archives
 * 01/08/01 (seiwald) - closure param for file_dirscan/file_archscan
 * 04/03/01 (seiwald) - AIX uses SARMAG
 * 07/16/02 (seiwald) - Support BSD style long filename in archives.
 * 11/04/02 (seiwald) - const-ing for string literals
 * 12/27/02 (seiwald) - support for AIX big archives
 * 12/30/02 (seiwald) - terminate ar_hdr for solaris sscanf()
 * 12/30/02 (seiwald) - skip solaris' empty archive member names (/, //xxx)
 */

# include "jam.h"
# include "filesys.h"
# include "pathsys.h"

# ifdef USE_FILEUNIX

# if defined( OS_SEQUENT ) || \
     defined( OS_DGUX ) || \
     defined( OS_SCO ) || \
     defined( OS_ISC ) 
# define PORTAR 1
# endif

# if defined( OS_RHAPSODY ) || \
     defined( OS_MACOSX ) || \
     defined( OS_NEXT )
/* need unistd for rhapsody's proper lseek */
# include <sys/dir.h>
# include <unistd.h>
# define STRUCT_DIRENT struct direct 
# else
# include <dirent.h>
# define STRUCT_DIRENT struct dirent 
# endif

# ifdef OS_COHERENT
# include <arcoff.h>
# define HAVE_AR
# endif

# if defined( OS_MVS ) || \
     defined( OS_INTERIX ) 

#define	ARMAG	"!<arch>\n"
#define	SARMAG	8
#define	ARFMAG	"`\n"

struct ar_hdr		/* archive file member header - printable ascii */
{
	char	ar_name[16];	/* file member name - `/' terminated */
	char	ar_date[12];	/* file member date - decimal */
	char	ar_uid[6];	/* file member user id - decimal */
	char	ar_gid[6];	/* file member group id - decimal */
	char	ar_mode[8];	/* file member mode - octal */
	char	ar_size[10];	/* file member size - decimal */
	char	ar_fmag[2];	/* ARFMAG - string to end header */
};

# define HAVE_AR
# endif

# if defined( OS_QNX ) || \
     defined( OS_BEOS ) || \
     defined( OS_MPEIX )
# define NO_AR
# define HAVE_AR
# endif

# ifndef HAVE_AR
# ifdef _AIX43
/* AIX 43 ar SUPPORTs only __AR_BIG__ */
# define __AR_BIG__
# endif
# include <ar.h>
# endif	

/*
 * file_dirscan() - scan a directory for files
 */

void
file_dirscan( 
	const char *dir,
	scanback func,
	void *closure )
{
	PATHNAME f;
	DIR *d;
	STRUCT_DIRENT *dirent;
	char filename[ MAXJPATH ];

	/* First enter directory itself */

	memset( (char *)&f, '\0', sizeof( f ) );

	f.f_dir.ptr = dir;
	f.f_dir.len = strlen(dir);

	dir = *dir ? dir : ".";

	/* Special case / : enter it */

	if( f.f_dir.len == 1 && f.f_dir.ptr[0] == '/' )
	    (*func)( closure, dir, 0 /* not stat()'ed */, (time_t)0 );

	/* Now enter contents of directory */

	if( !( d = opendir( dir ) ) )
	    return;

	if( DEBUG_BINDSCAN )
	    printf( "scan directory %s\n", dir );

	while( dirent = readdir( d ) )
	{
# ifdef old_sinix
	    /* Broken structure definition on sinix. */
	    f.f_base.ptr = dirent->d_name - 2;
# else
	    f.f_base.ptr = dirent->d_name;
# endif
	    f.f_base.len = strlen( f.f_base.ptr );

	    path_build( &f, filename, 0 );

	    (*func)( closure, filename, 0 /* not stat()'ed */, (time_t)0 );
	}

	closedir( d );
}

/*
 * file_time() - get timestamp of file, if not done by file_dirscan()
 */

int
file_time(
	const char *filename,
	time_t	*time )
{
	struct stat statbuf;

	if( stat( filename, &statbuf ) < 0 )
	    return -1;

	*time = statbuf.st_mtime;
	return 0;
}

/*
 * file_archscan() - scan an archive for files
 */

# ifndef AIAMAG	/* God-fearing UNIX */

# define SARFMAG 2
# define SARHDR sizeof( struct ar_hdr )

void
file_archscan(
	const char *archive,
	scanback func,
	void *closure )
{
# ifndef NO_AR
	struct ar_hdr ar_hdr;
	char buf[ MAXJPATH ];
	long offset;
	char    *string_table = 0;
	int fd;

	if( ( fd = open( archive, O_RDONLY, 0 ) ) < 0 )
	    return;

	if( read( fd, buf, SARMAG ) != SARMAG ||
	    strncmp( ARMAG, buf, SARMAG ) )
	{
	    close( fd );
	    return;
	}

	offset = SARMAG;

	if( DEBUG_BINDSCAN )
	    printf( "scan archive %s\n", archive );

	while( read( fd, &ar_hdr, SARHDR ) == SARHDR &&
	       !memcmp( ar_hdr.ar_fmag, ARFMAG, SARFMAG ) )
	{
	    long    lar_date;
	    long    lar_size;
	    char    lar_name[256];
	    char    *dst = lar_name;

	    /* solaris sscanf() does strlen first, so terminate somewhere */

	    ar_hdr.ar_fmag[0] = 0;

	    /* Get date & size */

	    sscanf( ar_hdr.ar_date, "%ld", &lar_date );
	    sscanf( ar_hdr.ar_size, "%ld", &lar_size );

	    /* Handle solaris string table.
	    ** The entry under the name // is the table,
	    ** and entries with the name /nnnn refer to the table.
	    */

	    if( ar_hdr.ar_name[0] != '/' )
	    {
		/* traditional archive entry names:
		** ends at the first space, /, or null.
		*/

		char *src = ar_hdr.ar_name;
		const char *e = src + sizeof( ar_hdr.ar_name );

		while( src < e && *src && *src != ' ' && *src != '/' )
		    *dst++ = *src++;
	    }
	    else if( ar_hdr.ar_name[1] == '/' )
	    {
		/* this is the "string table" entry of the symbol table,
		** which holds strings of filenames that are longer than
		** 15 characters (ie. don't fit into a ar_name)
		*/

		string_table = (char *)malloc(lar_size);

		lseek(fd, offset + SARHDR, 0);
		if( read(fd, string_table, lar_size) != lar_size )
		    printf( "error reading string table\n" );
	    }
	    else if( string_table && ar_hdr.ar_name[1] != ' ' )
	    {
		/* Long filenames are recognized by "/nnnn" where nnnn is
		** the offset of the string in the string table represented
		** in ASCII decimals.
		*/

		char *src = string_table + atoi( ar_hdr.ar_name + 1 );

		while( *src != '/' )
		    *dst++ = *src++;
	    }

	    /* Terminate lar_name */

	    *dst = 0;

	    /* Modern (BSD4.4) long names: if the name is "#1/nnnn", 
	    ** then the actual name is the nnnn bytes after the header.  
	    */

	    if( !strcmp( lar_name, "#1" ) )
	    {
		int len = atoi( ar_hdr.ar_name + 3 );
		if( read( fd, lar_name, len ) != len )
		    printf("error reading archive entry\n");
		lar_name[len] = 0;
	    }

	    /* Build name and pass it on.  */

	    if( lar_name[0] )
	    {
		if( DEBUG_BINDSCAN )
		    printf( "archive name %s found\n", lar_name );

		sprintf( buf, "%s(%s)", archive, lar_name );

		(*func)( closure, buf, 1 /* time valid */, (time_t)lar_date );
	    }

	    /* Position at next member */

	    offset += SARHDR + ( ( lar_size + 1 ) & ~1 );
	    lseek( fd, offset, 0 );
	}

	if (string_table)
	    free(string_table);

	close( fd );

# endif /* NO_AR */

}

# else /* AIAMAG - RS6000 AIX */

void
file_archscan(
	const char *archive,
	scanback func,
	void *closure )
{
	struct fl_hdr fl_hdr;

	struct {
		struct ar_hdr hdr;
		char pad[ 256 ];
	} ar_hdr ;

	char buf[ MAXJPATH ];
	long offset;
	int fd;

	if( ( fd = open( archive, O_RDONLY, 0 ) ) < 0 )
	    return;

# ifdef __AR_BIG__

	if( read( fd, (char *)&fl_hdr, FL_HSZ ) != FL_HSZ ||
	    strncmp( AIAMAGBIG, fl_hdr.fl_magic, SAIAMAG ) )
	{
	    if( strncmp( AIAMAG, fl_hdr.fl_magic, SAIAMAG ) )
		printf( "Can't read new archive %s before AIX 4.3.\n" );

	    close( fd );
	    return;
	}

# else

	if( read( fd, (char *)&fl_hdr, FL_HSZ ) != FL_HSZ ||
	    strncmp( AIAMAG, fl_hdr.fl_magic, SAIAMAG ) )
	{
	    close( fd );
	    return;
	}

# endif

	sscanf( fl_hdr.fl_fstmoff, "%ld", &offset );

	if( DEBUG_BINDSCAN )
	    printf( "scan archive %s\n", archive );

	while( offset > 0 &&
	       lseek( fd, offset, 0 ) >= 0 &&
	       read( fd, &ar_hdr, sizeof( ar_hdr ) ) >= sizeof( ar_hdr.hdr ) )
	{
	    long    lar_date;
	    int	    lar_namlen;

	    sscanf( ar_hdr.hdr.ar_namlen, "%d", &lar_namlen );
	    sscanf( ar_hdr.hdr.ar_date, "%ld", &lar_date );
	    sscanf( ar_hdr.hdr.ar_nxtmem, "%ld", &offset );

	    if( !lar_namlen )
		continue;

	    ar_hdr.hdr._ar_name.ar_name[ lar_namlen ] = '\0';

	    sprintf( buf, "%s(%s)", archive, ar_hdr.hdr._ar_name.ar_name );

	    (*func)( closure, buf, 1 /* time valid */, (time_t)lar_date );
	}

	close( fd );
}

# endif /* AIAMAG - RS6000 AIX */

# endif /* USE_FILEUNIX */

