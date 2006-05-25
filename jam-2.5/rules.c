/*
 * Copyright 1993, 1995 Christopher Seiwald.
 *
 * This file is part of Jam - see jam.c for Copyright information.
 */

/*
 * rules.c - access to RULEs, TARGETs, and ACTIONs
 *
 * External routines:
 *
 *    bindrule() - return pointer to RULE, creating it if necessary
 *    bindtarget() - return pointer to TARGET, creating it if necessary
 *    copytarget() - make a new target with the old target's name
 *    touchtarget() - mark a target to simulate being new
 *    targetlist() - turn list of target names into a TARGET chain
 *    targetentry() - add a TARGET to a chain of TARGETS
 *    targetchain() - append two TARGET chains
 *    actionlist() - append to an ACTION chain
 *    addsettings() - add a deferred "set" command to a target
 *    copysettings() - copy a settings list for temp use
 *    pushsettings() - set all target specific variables
 *    popsettings() - reset target specific variables to their pre-push values
 *    freesettings() - delete a settings list
 *    donerules() - free RULE and TARGET tables
 *
 * 04/12/94 (seiwald) - actionlist() now just appends a single action.
 * 08/23/94 (seiwald) - Support for '+=' (append to variable)
 * 06/21/02 (seiwald) - support for named parameters
 * 11/04/02 (seiwald) - const-ing for string literals
 * 12/03/02 (seiwald) - fix odd includes support by grafting them onto depends
 * 12/17/02 (seiwald) - new copysettings() to protect target-specific vars
 * 01/14/03 (seiwald) - fix includes fix with new internal includes TARGET
 */

# include "jam.h"
# include "lists.h"
# include "parse.h"
# include "variable.h"
# include "rules.h"
# include "newstr.h"
# include "hash.h"

static struct hash *rulehash = 0;
static struct hash *targethash = 0;


/*
 * bindrule() - return pointer to RULE, creating it if necessary
 */

RULE *
bindrule( const char *rulename )
{
	RULE rule, *r = &rule;

	if( !rulehash )
	    rulehash = hashinit( sizeof( RULE ), "rules" );

	r->name = rulename;

	if( hashenter( rulehash, (HASHDATA **)&r ) )
	{
	    r->name = newstr( rulename );	/* never freed */
	    r->procedure = (PARSE *)0;
	    r->actions = (char *)0;
	    r->bindlist = L0;
	    r->params = L0;
	    r->flags = 0;
	}

	return r;
}

/*
 * bindtarget() - return pointer to TARGET, creating it if necessary
 */

TARGET *
bindtarget( const char *targetname )
{
	TARGET target, *t = &target;

	if( !targethash )
	    targethash = hashinit( sizeof( TARGET ), "targets" );

	t->name = targetname;

	if( hashenter( targethash, (HASHDATA **)&t ) )
	{
	    memset( (char *)t, '\0', sizeof( *t ) );
	    t->name = newstr( targetname );	/* never freed */
	    t->boundname = t->name;		/* default for T_FLAG_NOTFILE */
	}

	return t;
}

/*
 * copytarget() - make a new target with the old target's name
 *
 * Not entered into hash table -- for internal nodes.
 */

TARGET *
copytarget( const TARGET *ot )
{
	TARGET *t;

	t = (TARGET *)malloc( sizeof( *t ) );
	memset( (char *)t, '\0', sizeof( *t ) );
	t->name = copystr( ot->name );
	t->boundname = t->name;

	t->flags |= T_FLAG_NOTFILE | T_FLAG_INTERNAL;

	return t;
}

/*
 * touchtarget() - mark a target to simulate being new
 */

void
touchtarget( const char *t )
{
	bindtarget( t )->flags |= T_FLAG_TOUCHED;
}

/*
 * targetlist() - turn list of target names into a TARGET chain
 *
 * Inputs:
 *	chain	existing TARGETS to append to
 *	targets	list of target names
 */

TARGETS *
targetlist( 
	TARGETS	*chain,
	LIST 	*targets )
{
	for( ; targets; targets = list_next( targets ) )
	    chain = targetentry( chain, bindtarget( targets->string ) );

	return chain;
}

/*
 * targetentry() - add a TARGET to a chain of TARGETS
 *
 * Inputs:
 *	chain	exisitng TARGETS to append to
 *	target	new target to append
 */

TARGETS *
targetentry( 
	TARGETS	*chain,
	TARGET	*target )
{
	TARGETS *c;

	c = (TARGETS *)malloc( sizeof( TARGETS ) );
	c->target = target;

	if( !chain ) chain = c;
	else chain->tail->next = c;
	chain->tail = c;
	c->next = 0;

	return chain;
}

/*
 * targetchain() - append two TARGET chains
 *
 * Inputs:
 *	chain	exisitng TARGETS to append to
 *	target	new target to append
 */

TARGETS *
targetchain( 
	TARGETS	*chain,
	TARGETS	*targets )
{
	TARGETS *c;

	if( !targets )
	    return chain;
	else if( !chain )
	    return targets;

	chain->tail->next = targets;
	chain->tail = targets->tail;

	return chain;
}

/*
 * actionlist() - append to an ACTION chain
 */

ACTIONS *
actionlist(
	ACTIONS	*chain,
	ACTION	*action )
{
	ACTIONS *actions = (ACTIONS *)malloc( sizeof( ACTIONS ) );

	actions->action = action;

	if( !chain ) chain = actions;
	else chain->tail->next = actions;
	chain->tail = actions;
	actions->next = 0;

	return chain;
}

/*
 * addsettings() - add a deferred "set" command to a target
 *
 * Adds a variable setting (varname=list) onto a chain of settings
 * for a particular target.  Replaces the previous previous value,
 * if any, unless 'append' says to append the new list onto the old.
 * Returns the head of the chain of settings.
 */

SETTINGS *
addsettings(
	SETTINGS *head,
	int	setflag,
	const char *symbol,
	LIST	*value )
{
	SETTINGS *v;
	
	/* Look for previous setting */

	for( v = head; v; v = v->next )
	    if( !strcmp( v->symbol, symbol ) )
		break;

	/* If not previously set, alloc a new. */
	/* If appending, do so. */
	/* Else free old and set new. */

	if( !v )
	{
	    v = (SETTINGS *)malloc( sizeof( *v ) );
	    v->symbol = newstr( symbol );
	    v->value = value;
	    v->next = head;
	    head = v;
	}
	else switch( setflag )
	{
	case VAR_SET:
	    /* Toss old, set new */
	    list_free( v->value );
	    v->value = value;
	    break;

	case VAR_APPEND:
	    /* Append new to old */
	    v->value = list_append( v->value, value );
	    break;

	case VAR_DEFAULT:
	    /* Toss new, old already set */
	    list_free( value );
	    break;
	}

	/* Return (new) head of list. */

	return head;
}

/*
 * copysettings() - copy a settings list for temp use
 *
 * When target-specific variables are pushed into place with pushsettings(),
 * any global variables with the same name are swapped onto the target's
 * SETTINGS chain.  If that chain gets modified (by using the "on target"
 * syntax), popsettings() would wrongly swap those modified values back 
 * as the new global values.  
 *
 * copysettings() protects the target's SETTINGS chain by providing a
 * copy of the chain to pass to pushsettings() and popsettings(), so that
 * the target's original SETTINGS chain can be modified using the usual
 * "on target" syntax.
 */

SETTINGS *
copysettings( SETTINGS *from )
{
	SETTINGS *head = 0, *v;

	for( ; from; from = from->next )
	{
	    SETTINGS *v = (SETTINGS *)malloc( sizeof( *v ) );
	    v->symbol = copystr( from->symbol );
	    v->value = list_copy( 0, from->value );
	    v->next = head;
	    head = v;
	}

	return head;
}

/*
 * pushsettings() - set all target specific variables
 */

void
pushsettings( SETTINGS *v )
{
	for( ; v; v = v->next )
	    v->value = var_swap( v->symbol, v->value );
}

/*
 * popsettings() - reset target specific variables to their pre-push values
 */

void
popsettings( SETTINGS *v )
{
	pushsettings( v );	/* just swap again */
}

/*
 *    freesettings() - delete a settings list
 */

void
freesettings( SETTINGS *v )
{
	while( v )
	{
	    SETTINGS *n = v->next;

	    freestr( v->symbol );
	    list_free( v->value );
	    free( (char *)v );

	    v = n;
	}
}

/*
 * donerules() - free RULE and TARGET tables
 */

void
donerules()
{
	hashdone( rulehash );
	hashdone( targethash );
}
