/*
 * Copyright 1993, 1995 Christopher Seiwald.
 *
 * This file is part of Jam - see jam.c for Copyright information.
 */

/*
 * search.h - find a target along $(SEARCH) or $(LOCATE) 
 *
 * 11/04/02 (seiwald) - const-ing for string literals
 */

const char *search( const char *target, time_t *time );
