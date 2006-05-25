/*
 * Copyright 1993-2002 Christopher Seiwald and Perforce Software, Inc.
 *
 * This file is part of Jam - see jam.c for Copyright information.
 */

/*
 * pathsys.h - PATHNAME struct 
 *
 * 11/04/02 (seiwald) - const-ing for string literals
 */

/*
 * PATHNAME - a name of a file, broken into <grist>dir/base/suffix(member)
 *
 * <grist> is salt to distinguish between targets that otherwise would
 * have the same name:  it never appears in the bound name of a target.
 * (member) is an archive member name: the syntax is arbitrary, but must 
 * agree in path_parse(), path_build() and the Jambase.
 *
 * On VMS, we keep track of whether the original path was a directory
 * (without a file), so that $(VAR:D) can climb to the parent.
 */

typedef struct _pathname PATHNAME;
typedef struct _pathpart PATHPART;

struct _pathpart {
	const char *ptr;
	int	len;
};

struct _pathname {
	PATHPART	part[6];
# ifdef OS_VMS
	int		parent;
# endif

# define f_grist	part[0]
# define f_root		part[1]
# define f_dir		part[2]
# define f_base		part[3]
# define f_suffix	part[4]
# define f_member	part[5]

} ;

void path_build( PATHNAME *f, char *file, int binding );
void path_parse( const char *file, PATHNAME *f );
void path_parent( PATHNAME *f );

