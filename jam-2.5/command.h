/*
 * Copyright 1994 Christopher Seiwald.
 *
 * This file is part of Jam - see jam.c for Copyright information.
 */

/*
 * command.h - the CMD structure and routines to manipulate them
 *
 * Both ACTION and CMD contain a rule, targets, and sources.  An
 * ACTION describes a rule to be applied to the given targets and
 * sources; a CMD is what actually gets executed by the shell.  The
 * differences are due to:
 *
 *	ACTIONS must be combined if 'actions together' is given.
 *	ACTIONS must be split if 'actions piecemeal' is given.
 *	ACTIONS must have current sources omitted for 'actions updated'.
 *
 * The CMD datatype holds a single command that is to be executed 
 * against a target, and they can chain together to represent the 
 * full collection of commands used to update a target.
 *
 * Structures:
 *
 * 	CMD - an action, ready to be formatted into a buffer and executed
 *
 * External routines:
 *
 * 	cmd_new() - return a new CMD or 0 if too many args
 *	cmd_free() - delete CMD and its parts
 *	cmd_next() - walk the CMD chain
 */

/*
 * CMD - an action, ready to be formatted into a buffer and executed
 */

typedef struct _cmd CMD;

struct _cmd
{
	CMD	*next;
	CMD	*tail;		/* valid on in head */
	RULE	*rule;		/* rule->actions contains shell script */
	LIST	*shell;		/* $(SHELL) value */
	LOL	args;		/* LISTs for $(<), $(>) */
	char	buf[ MAXLINE ];	/* actual commands */
} ;

CMD *cmd_new(
	RULE	*rule,		/* rule (referenced) */
	LIST	*targets,	/* $(<) (freed) */
	LIST	*sources,	/* $(>) (freed) */
	LIST	*shell,		/* $(SHELL) (freed) */
	int	maxline );	/* max line length */

void cmd_free( CMD *cmd );

# define cmd_next( c ) ((c)->next)
