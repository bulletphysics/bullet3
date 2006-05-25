/*
 * Copyright 1993, 1995 Christopher Seiwald.
 *
 * This file is part of Jam - see jam.c for Copyright information.
 */

/*
 * expand.h - expand a buffer, given variable values
 *
 * 11/04/02 (seiwald) - const-ing for string literals
 */

LIST *var_expand( 
	LIST		*l,
	const char	*in,
	const char	*end,
	LOL 		*lol,
	int 		cancopyin );
