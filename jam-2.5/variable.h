/*
 * Copyright 1993, 2000 Christopher Seiwald.
 *
 * This file is part of Jam - see jam.c for Copyright information.
 */

/*
 * variable.h - handle jam multi-element variables
 *
 * 11/04/02 (seiwald) - const-ing for string literals
 */

void 	var_defines( const char **e );
int 	var_string( const char *in, char *out, int outsize, LOL *lol );
LIST * 	var_get( const char *symbol );
void 	var_set( const char *symbol, LIST *value, int flag );
LIST * 	var_swap( const char *symbol, LIST *value );
void 	var_done();

/*
 * Defines for var_set().
 */

# define VAR_SET	0	/* override previous value */
# define VAR_APPEND	1	/* append to previous value */
# define VAR_DEFAULT	2	/* set only if no previous value */

