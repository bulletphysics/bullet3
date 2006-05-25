%token _BANG_t
%token _BANG_EQUALS_t
%token _AMPER_t
%token _AMPERAMPER_t
%token _LPAREN_t
%token _RPAREN_t
%token _PLUS_EQUALS_t
%token _COLON_t
%token _SEMIC_t
%token _LANGLE_t
%token _LANGLE_EQUALS_t
%token _EQUALS_t
%token _RANGLE_t
%token _RANGLE_EQUALS_t
%token _QUESTION_EQUALS_t
%token _LBRACKET_t
%token _RBRACKET_t
%token ACTIONS_t
%token BIND_t
%token BREAK_t
%token CASE_t
%token CONTINUE_t
%token DEFAULT_t
%token ELSE_t
%token EXISTING_t
%token FOR_t
%token IF_t
%token IGNORE_t
%token IN_t
%token INCLUDE_t
%token LOCAL_t
%token MAXLINE_t
%token ON_t
%token PIECEMEAL_t
%token QUIETLY_t
%token RETURN_t
%token RULE_t
%token SWITCH_t
%token TOGETHER_t
%token UPDATED_t
%token WHILE_t
%token _LBRACE_t
%token _BAR_t
%token _BARBAR_t
%token _RBRACE_t
/*
 * Copyright 1993-2002 Christopher Seiwald and Perforce Software, Inc.
 *
 * This file is part of Jam - see jam.c for Copyright information.
 */

/*
 * jamgram.yy - jam grammar
 *
 * 04/13/94 (seiwald) - added shorthand L0 for null list pointer
 * 06/01/94 (seiwald) - new 'actions existing' does existing sources
 * 08/23/94 (seiwald) - Support for '+=' (append to variable)
 * 08/31/94 (seiwald) - Allow ?= as alias for "default =".
 * 09/15/94 (seiwald) - if conditionals take only single arguments, so
 *			that 'if foo == bar' gives syntax error (use =).
 * 02/11/95 (seiwald) - when scanning arguments to rules, only treat
 *			punctuation keywords as keywords.  All arg lists
 *			are terminated with punctuation keywords.
 * 09/11/00 (seiwald) - Support for function calls; rules return LIST *.
 * 01/22/01 (seiwald) - replace evaluate_if() with compile_eval()
 * 01/24/01 (seiwald) - 'while' statement
 * 03/23/01 (seiwald) - "[ on target rule ]" support 
 * 02/27/02 (seiwald) - un-break "expr : arg in list" syntax
 * 03/02/02 (seiwald) - rules can be invoked via variable names
 * 03/12/02 (seiwald) - set YYMAXDEPTH for big, right-recursive rules
 * 02/28/02 (seiwald) - merge EXEC_xxx flags in with RULE_xxx 
 * 06/21/02 (seiwald) - support for named parameters
 * 10/22/02 (seiwald) - working return/break/continue statements
 */

%token ARG STRING

%left _BARBAR_t _BAR_t
%left _AMPERAMPER_t _AMPER_t
%left _EQUALS_t _BANG_EQUALS_t IN_t
%left _LANGLE_t _LANGLE_EQUALS_t _RANGLE_t _RANGLE_EQUALS_t
%left _BANG_t

%{
#include "jam.h"

#include "lists.h"
#include "variable.h"
#include "parse.h"
#include "scan.h"
#include "compile.h"
#include "newstr.h"
#include "rules.h"

# define YYMAXDEPTH 10000	/* for OSF and other less endowed yaccs */

# define F0 (LIST *(*)(PARSE *, LOL *, int *))0
# define P0 (PARSE *)0
# define S0 (char *)0

# define pappend( l,r )    	parse_make( compile_append,l,r,P0,S0,S0,0 )
# define pbreak( l,f )     	parse_make( compile_break,l,P0,P0,S0,S0,f )
# define peval( c,l,r )		parse_make( compile_eval,l,r,P0,S0,S0,c )
# define pfor( s,l,r )    	parse_make( compile_foreach,l,r,P0,s,S0,0 )
# define pif( l,r,t )	  	parse_make( compile_if,l,r,t,S0,S0,0 )
# define pincl( l )       	parse_make( compile_include,l,P0,P0,S0,S0,0 )
# define plist( s )	  	parse_make( compile_list,P0,P0,P0,s,S0,0 )
# define plocal( l,r,t )  	parse_make( compile_local,l,r,t,S0,S0,0 )
# define pnull()	  	parse_make( compile_null,P0,P0,P0,S0,S0,0 )
# define pon( l,r )	  	parse_make( compile_on,l,r,P0,S0,S0,0 )
# define prule( a,p )     	parse_make( compile_rule,a,p,P0,S0,S0,0 )
# define prules( l,r )	  	parse_make( compile_rules,l,r,P0,S0,S0,0 )
# define pset( l,r,a ) 	  	parse_make( compile_set,l,r,P0,S0,S0,a )
# define pset1( l,r,t,a )	parse_make( compile_settings,l,r,t,S0,S0,a )
# define psetc( s,l,r )     	parse_make( compile_setcomp,l,r,P0,s,S0,0 )
# define psete( s,l,s1,f ) 	parse_make( compile_setexec,l,P0,P0,s,s1,f )
# define pswitch( l,r )   	parse_make( compile_switch,l,r,P0,S0,S0,0 )
# define pwhile( l,r )   	parse_make( compile_while,l,r,P0,S0,S0,0 )

# define pnode( l,r )    	parse_make( F0,l,r,P0,S0,S0,0 )
# define psnode( s,l )     	parse_make( F0,l,P0,P0,s,S0,0 )

%}

%%

run	: /* empty */
		/* do nothing */
	| rules
		{ parse_save( $1.parse ); }
	;

/*
 * block - zero or more rules
 * rules - one or more rules
 * rule - any one of jam's rules
 * right-recursive so rules execute in order.
 */

block	: /* empty */
		{ $$.parse = pnull(); }
	| rules
		{ $$.parse = $1.parse; }
	;

rules	: rule
		{ $$.parse = $1.parse; }
	| rule rules
		{ $$.parse = prules( $1.parse, $2.parse ); }
	| LOCAL_t list _SEMIC_t block
		{ $$.parse = plocal( $2.parse, pnull(), $4.parse ); }
	| LOCAL_t list _EQUALS_t list _SEMIC_t block
		{ $$.parse = plocal( $2.parse, $4.parse, $6.parse ); }
	;

rule	: _LBRACE_t block _RBRACE_t
		{ $$.parse = $2.parse; }
	| INCLUDE_t list _SEMIC_t
		{ $$.parse = pincl( $2.parse ); }
	| arg lol _SEMIC_t
		{ $$.parse = prule( $1.parse, $2.parse ); }
	| arg assign list _SEMIC_t
		{ $$.parse = pset( $1.parse, $3.parse, $2.number ); }
	| arg ON_t list assign list _SEMIC_t
		{ $$.parse = pset1( $1.parse, $3.parse, $5.parse, $4.number ); }
	| BREAK_t list _SEMIC_t
		{ $$.parse = pbreak( $2.parse, JMP_BREAK ); }
	| CONTINUE_t list _SEMIC_t
		{ $$.parse = pbreak( $2.parse, JMP_CONTINUE ); }
	| RETURN_t list _SEMIC_t
		{ $$.parse = pbreak( $2.parse, JMP_RETURN ); }
	| FOR_t ARG IN_t list _LBRACE_t block _RBRACE_t
		{ $$.parse = pfor( $2.string, $4.parse, $6.parse ); }
	| SWITCH_t list _LBRACE_t cases _RBRACE_t
		{ $$.parse = pswitch( $2.parse, $4.parse ); }
	| IF_t expr _LBRACE_t block _RBRACE_t 
		{ $$.parse = pif( $2.parse, $4.parse, pnull() ); }
	| IF_t expr _LBRACE_t block _RBRACE_t ELSE_t rule
		{ $$.parse = pif( $2.parse, $4.parse, $7.parse ); }
	| WHILE_t expr _LBRACE_t block _RBRACE_t
		{ $$.parse = pwhile( $2.parse, $4.parse ); }
	| RULE_t ARG params _LBRACE_t block _RBRACE_t
		{ $$.parse = psetc( $2.string, $3.parse, $5.parse ); }
	| ON_t arg rule
		{ $$.parse = pon( $2.parse, $3.parse ); }
	| ACTIONS_t eflags ARG bindlist _LBRACE_t
		{ yymode( SCAN_STRING ); }
	  STRING 
		{ yymode( SCAN_NORMAL ); }
	  _RBRACE_t
		{ $$.parse = psete( $3.string,$4.parse,$7.string,$2.number ); }
	;

/*
 * assign - = or +=
 */

assign	: _EQUALS_t
		{ $$.number = VAR_SET; }
	| _PLUS_EQUALS_t
		{ $$.number = VAR_APPEND; }
	| _QUESTION_EQUALS_t
		{ $$.number = VAR_DEFAULT; }
	| DEFAULT_t _EQUALS_t
		{ $$.number = VAR_DEFAULT; }
	;

/*
 * expr - an expression for if
 */

expr	: arg 
		{ $$.parse = peval( EXPR_EXISTS, $1.parse, pnull() ); }
	| expr _EQUALS_t expr 
		{ $$.parse = peval( EXPR_EQUALS, $1.parse, $3.parse ); }
	| expr _BANG_EQUALS_t expr
		{ $$.parse = peval( EXPR_NOTEQ, $1.parse, $3.parse ); }
	| expr _LANGLE_t expr
		{ $$.parse = peval( EXPR_LESS, $1.parse, $3.parse ); }
	| expr _LANGLE_EQUALS_t expr 
		{ $$.parse = peval( EXPR_LESSEQ, $1.parse, $3.parse ); }
	| expr _RANGLE_t expr 
		{ $$.parse = peval( EXPR_MORE, $1.parse, $3.parse ); }
	| expr _RANGLE_EQUALS_t expr 
		{ $$.parse = peval( EXPR_MOREEQ, $1.parse, $3.parse ); }
	| expr _AMPER_t expr 
		{ $$.parse = peval( EXPR_AND, $1.parse, $3.parse ); }
	| expr _AMPERAMPER_t expr 
		{ $$.parse = peval( EXPR_AND, $1.parse, $3.parse ); }
	| expr _BAR_t expr
		{ $$.parse = peval( EXPR_OR, $1.parse, $3.parse ); }
	| expr _BARBAR_t expr
		{ $$.parse = peval( EXPR_OR, $1.parse, $3.parse ); }
	| arg IN_t list
		{ $$.parse = peval( EXPR_IN, $1.parse, $3.parse ); }
	| _BANG_t expr
		{ $$.parse = peval( EXPR_NOT, $2.parse, pnull() ); }
	| _LPAREN_t expr _RPAREN_t
		{ $$.parse = $2.parse; }
	;

/*
 * cases - action elements inside a 'switch'
 * case - a single action element inside a 'switch'
 * right-recursive rule so cases can be examined in order.
 */

cases	: /* empty */
		{ $$.parse = P0; }
	| case cases
		{ $$.parse = pnode( $1.parse, $2.parse ); }
	;

case	: CASE_t ARG _COLON_t block
		{ $$.parse = psnode( $2.string, $4.parse ); }
	;

/*
 * params - optional parameter names to rule definition
 * right-recursive rule so that params can be added in order.
 */

params	: /* empty */
		{ $$.parse = P0; }
	| ARG _COLON_t params
		{ $$.parse = psnode( $1.string, $3.parse ); }
	| ARG
		{ $$.parse = psnode( $1.string, P0 ); }
	;

/*
 * lol - list of lists
 * right-recursive rule so that lists can be added in order.
 */

lol	: list
		{ $$.parse = pnode( P0, $1.parse ); }
	| list _COLON_t lol
		{ $$.parse = pnode( $3.parse, $1.parse ); }
	;

/*
 * list - zero or more args in a LIST
 * listp - list (in puncutation only mode)
 * arg - one ARG or function call
 */

list	: listp
		{ $$.parse = $1.parse; yymode( SCAN_NORMAL ); }
	;

listp	: /* empty */
		{ $$.parse = pnull(); yymode( SCAN_PUNCT ); }
	| listp arg
		{ $$.parse = pappend( $1.parse, $2.parse ); }
	;

arg	: ARG 
		{ $$.parse = plist( $1.string ); }
	| _LBRACKET_t { yymode( SCAN_NORMAL ); } func _RBRACKET_t
		{ $$.parse = $3.parse; }
	;

/*
 * func - a function call (inside [])
 * This needs to be split cleanly out of 'rule'
 */

func	: arg lol
		{ $$.parse = prule( $1.parse, $2.parse ); }
	| ON_t arg arg lol
		{ $$.parse = pon( $2.parse, prule( $3.parse, $4.parse ) ); }
	| ON_t arg RETURN_t list 
		{ $$.parse = pon( $2.parse, $4.parse ); }
	;

/*
 * eflags - zero or more modifiers to 'executes'
 * eflag - a single modifier to 'executes'
 */

eflags	: /* empty */
		{ $$.number = 0; }
	| eflags eflag
		{ $$.number = $1.number | $2.number; }
	;

eflag	: UPDATED_t
		{ $$.number = RULE_UPDATED; }
	| TOGETHER_t
		{ $$.number = RULE_TOGETHER; }
	| IGNORE_t
		{ $$.number = RULE_IGNORE; }
	| QUIETLY_t
		{ $$.number = RULE_QUIETLY; }
	| PIECEMEAL_t
		{ $$.number = RULE_PIECEMEAL; }
	| EXISTING_t
		{ $$.number = RULE_EXISTING; }
	| MAXLINE_t ARG
		{ $$.number = atoi( $2.string ) * RULE_MAXLINE; }
	;


/*
 * bindlist - list of variable to bind for an action
 */

bindlist : /* empty */
		{ $$.parse = pnull(); }
	| BIND_t list
		{ $$.parse = $2.parse; }
	;


