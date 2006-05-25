/*
 * Copyright 1993-2002 Christopher Seiwald and Perforce Software, Inc.
 *
 * This file is part of Jam - see jam.c for Copyright information.
 */

/*
 * filemac.c - manipulate file names and scan directories on macintosh
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
 * 01/21/00 (malyn) - divorced from GUSI
 * 01/08/01 (seiwald) - closure param for file_dirscan/file_archscan
 * 11/04/02 (seiwald) - const-ing for string literals
 */
 
# include "jam.h"
# include "filesys.h"
# include "pathsys.h"

# ifdef OS_MAC

#include <Files.h>
#include <Folders.h>

# include <:sys:stat.h>

void CopyC2PStr(const char * cstr, StringPtr pstr)
{
	int	len;
	
	for (len = 0; *cstr && len<255; pstr[++len] = *cstr++)
		;
	
	pstr[0] = len;
}

/*
 * file_dirscan() - scan a directory for files
 */

void
file_dirscan( 
	const char *dir,
	scanback func,
	void	*closure )
{
	PATHNAME f;
	char filename[ MAXJPATH ];
	unsigned char fullPath[ 512 ];

	FSSpec spec;
	WDPBRec vol;
	Str63 volName;	
	CInfoPBRec lastInfo;
	int index = 1;
	
	/* First enter directory itself */

	memset( (char *)&f, '\0', sizeof( f ) );

	f.f_dir.ptr = dir;
	f.f_dir.len = strlen(dir);

	if( DEBUG_BINDSCAN )
	    printf( "scan directory %s\n", dir );
		
	/* Special case ":" - enter it */

	if( f.f_dir.len == 1 && f.f_dir.ptr[0] == ':' )
	    (*func)( closure, dir, 0 /* not stat()'ed */, (time_t)0 );

	/* Now enter contents of directory */

	vol.ioNamePtr = volName;
	
	if( PBHGetVolSync( &vol ) )
		return;

	CopyC2PStr( dir, fullPath );
	
	if( FSMakeFSSpec( vol.ioWDVRefNum, vol.ioWDDirID, fullPath, &spec ) )
		return;
	
      	lastInfo.dirInfo.ioVRefNum 	= spec.vRefNum;
   	lastInfo.dirInfo.ioDrDirID 	= spec.parID;
   	lastInfo.dirInfo.ioNamePtr 	= spec.name;
   	lastInfo.dirInfo.ioFDirIndex 	= 0;
   	lastInfo.dirInfo.ioACUser 	= 0;
			
   	if( PBGetCatInfoSync(&lastInfo) )
		return;

	if (!(lastInfo.dirInfo.ioFlAttrib & 0x10))
		return;

	// ioDrDirID must be reset each time.
	
	spec.parID = lastInfo.dirInfo.ioDrDirID;
		
	for( ;; )
	{
       	    lastInfo.dirInfo.ioVRefNum 	= spec.vRefNum;
	    lastInfo.dirInfo.ioDrDirID	= spec.parID;
	    lastInfo.dirInfo.ioNamePtr 	= fullPath;
	    lastInfo.dirInfo.ioFDirIndex = index++;
	   		
	    if( PBGetCatInfoSync(&lastInfo) )
		return;
			
	    f.f_base.ptr = (char *)fullPath + 1;
	    f.f_base.len = *fullPath;
	    
	    path_build( &f, filename, 0 );

	    (*func)( closure, filename, 0 /* not stat()'ed */, (time_t)0 );
	}
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

void
file_archscan(
	const char *archive,
	scanback func,
	void	*closure )
{
}


# endif /* macintosh */

