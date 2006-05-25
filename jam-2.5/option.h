/*
 * Copyright 1993, 1995 Christopher Seiwald.
 *
 * This file is part of Jam - see jam.c for Copyright information.
 */

/*
 * option.h - command line option processing
 *
 * {o >o
 *  \ -) "Command line option."
 *
 * 11/04/02 (seiwald) - const-ing for string literals
 */

typedef struct option
{
	char		flag;	/* filled in by getoption() */
	const char	*val;	/* set to random address if true */
} option;

# define N_OPTS 256

int 		getoptions( int argc, char **argv, const char *opts, option *optv );
const char *	getoptval( option *optv, char opt, int subopt );
