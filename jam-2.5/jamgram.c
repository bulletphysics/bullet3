#ifndef lint
static char const 
yyrcsid[] = "$FreeBSD: src/usr.bin/yacc/skeleton.c,v 1.28.2.1 2001/07/19 05:46:39 peter Exp $";
#endif
#include <stdlib.h>
#define YYBYACC 1
#define YYMAJOR 1
#define YYMINOR 9
#define YYLEX yylex()
#define YYEMPTY -1
#define yyclearin (yychar=(YYEMPTY))
#define yyerrok (yyerrflag=0)
#define YYRECOVERING() (yyerrflag!=0)
#if defined(__cplusplus) || __STDC__
static int yygrowstack(void);
#else
static int yygrowstack();
#endif
#define YYPREFIX "yy"
#line 85 "jamgram.y"
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

#line 60 "y.tab.c"
#define YYERRCODE 256
#define _BANG_t 257
#define _BANG_EQUALS_t 258
#define _AMPER_t 259
#define _AMPERAMPER_t 260
#define _LPAREN_t 261
#define _RPAREN_t 262
#define _PLUS_EQUALS_t 263
#define _COLON_t 264
#define _SEMIC_t 265
#define _LANGLE_t 266
#define _LANGLE_EQUALS_t 267
#define _EQUALS_t 268
#define _RANGLE_t 269
#define _RANGLE_EQUALS_t 270
#define _QUESTION_EQUALS_t 271
#define _LBRACKET_t 272
#define _RBRACKET_t 273
#define ACTIONS_t 274
#define BIND_t 275
#define BREAK_t 276
#define CASE_t 277
#define CONTINUE_t 278
#define DEFAULT_t 279
#define ELSE_t 280
#define EXISTING_t 281
#define FOR_t 282
#define IF_t 283
#define IGNORE_t 284
#define IN_t 285
#define INCLUDE_t 286
#define LOCAL_t 287
#define MAXLINE_t 288
#define ON_t 289
#define PIECEMEAL_t 290
#define QUIETLY_t 291
#define RETURN_t 292
#define RULE_t 293
#define SWITCH_t 294
#define TOGETHER_t 295
#define UPDATED_t 296
#define WHILE_t 297
#define _LBRACE_t 298
#define _BAR_t 299
#define _BARBAR_t 300
#define _RBRACE_t 301
#define ARG 302
#define STRING 303
const short yylhs[] = {                                        -1,
    0,    0,    2,    2,    1,    1,    1,    1,    3,    3,
    3,    3,    3,    3,    3,    3,    3,    3,    3,    3,
    3,    3,    3,   13,   14,    3,    7,    7,    7,    7,
    9,    9,    9,    9,    9,    9,    9,    9,    9,    9,
    9,    9,    9,    9,    8,    8,   15,   10,   10,   10,
    6,    6,    4,   16,   16,    5,   18,    5,   17,   17,
   17,   11,   11,   19,   19,   19,   19,   19,   19,   19,
   12,   12,
};
const short yylen[] = {                                         2,
    0,    1,    0,    1,    1,    2,    4,    6,    3,    3,
    3,    4,    6,    3,    3,    3,    7,    5,    5,    7,
    5,    6,    3,    0,    0,    9,    1,    1,    1,    2,
    1,    3,    3,    3,    3,    3,    3,    3,    3,    3,
    3,    3,    2,    3,    0,    2,    4,    0,    3,    1,
    1,    3,    1,    0,    2,    1,    0,    4,    2,    4,
    4,    0,    2,    1,    1,    1,    1,    1,    1,    2,
    0,    2,
};
const short yydefred[] = {                                      0,
   57,   62,   54,   54,    0,    0,   54,   54,    0,   54,
    0,   54,    0,    0,   56,    0,    2,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    4,    0,    6,   28,
   27,   29,    0,   54,    0,    0,   54,    0,   54,    0,
   69,   66,    0,   68,   67,   65,   64,    0,   63,   14,
   55,   15,   54,   43,    0,   54,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,   10,    0,   54,
   23,   16,    0,    0,    0,    0,    9,   30,    0,   54,
   11,    0,    0,   59,   58,   70,   54,    0,    0,   44,
   42,    0,    0,    0,   34,   35,    0,   36,   37,    0,
    0,    0,    7,    0,    0,    0,    0,    0,    0,    0,
   54,   52,   12,   54,   54,   72,   24,    0,    0,    0,
   49,    0,    0,   18,   46,   21,    0,   61,   60,    0,
    0,    0,    8,   22,    0,   13,   25,   17,   20,   47,
    0,   26,
};
const short yydgoto[] = {                                      16,
   37,   38,   18,   45,   28,   46,   47,  118,   29,   84,
   21,   98,  140,  151,  119,   23,   50,   20,   59,
};
const short yysindex[] = {                                    -40,
    0,    0,    0,    0, -295, -233,    0,    0, -240,    0,
 -284,    0, -233,  -40,    0,    0,    0,  -40,   -7, -241,
   49, -242, -240, -232, -239, -233, -233, -231,  -75, -220,
 -195,   34, -210, -243, -226,  -59,    0, -223,    0,    0,
    0,    0, -174,    0, -165, -164,    0, -240,    0, -173,
    0,    0, -199,    0,    0,    0,    0, -166,    0,    0,
    0,    0,    0,    0, -154,    0, -233, -233, -233, -233,
 -233, -233, -233, -233,  -40, -233, -233,    0,  -40,    0,
    0,    0, -153, -179, -157,  -40,    0,    0,  -42,    0,
    0, -144, -185,    0,    0,    0,    0, -171, -170,    0,
    0,  -54,   94,   94,    0,    0,  -54,    0,    0, -160,
   88,   88,    0, -140, -243,  -40, -151, -139, -157, -134,
    0,    0,    0,    0,    0,    0,    0,  -40, -149,  -40,
    0, -131,  -93,    0,    0,    0,  -85,    0,    0, -121,
 -115,   34,    0,    0,  -40,    0,    0,    0,    0,    0,
 -105,    0,
};
const short yyrindex[] = {                                    197,
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0, -103,    0,    0,    0,    3, -235,    0,
    0,    0, -110,    0,    0,    0,    0,  -94,    0,    0,
    0,    0,    0,  -96,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0, -246,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,  -95,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0, -103,    0,    0,    0,    4,    0,
    0,    0,  -84,    0,  -79, -103,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0, -202, -224, -176,    0,    0,  -81,    0,    0,    0,
 -257, -209,    0,    0,  -96, -103,    0,    0,  -79,    0,
    0,    0,    0,    0,    0,    0,    0, -103,    1,    4,
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0, -237,    0,    0,    0,    0,    0,
    0,    0,
};
const short yygindex[] = {                                      0,
   26,  -65,  -32,    5,    2,  -43,  131,  108,   66,  113,
    0,    0,    0,    0,    0,    0,    0,    0,    0,
};
#define YYTABLESIZE 364
const short yytable[] = {                                      81,
   19,   19,    5,    3,   40,   94,   25,   22,   24,  110,
   32,   30,   31,  113,   33,   19,   35,   34,   51,   19,
  120,   49,   60,   26,   61,   17,   51,   27,   54,   54,
    1,    1,   62,   19,   38,   38,   54,   38,    1,    3,
   40,   40,   40,   39,   78,   63,  122,   48,   89,   93,
  132,   92,   41,   66,   82,   33,   33,   33,   83,   33,
   15,   15,  141,    3,  143,   33,   54,   99,   15,   79,
  101,   85,   80,   38,   38,   38,   19,   87,   36,  150,
   19,  139,   39,   39,  114,   39,    1,   19,   41,   41,
   41,   64,   65,   88,  125,   33,   33,   33,   90,   95,
   91,  126,   96,   67,   68,   69,  124,  100,   97,  149,
  115,   70,   71,   72,   73,   74,   15,   19,  116,  117,
  123,   39,   39,   39,  130,  137,  127,  128,  138,   19,
  142,   19,  102,  103,  104,  105,  106,  107,  108,  109,
  129,  111,  112,   19,   76,   77,   19,   53,   53,   53,
  133,   53,   53,   53,   53,   53,   53,   53,   53,   53,
   53,  134,   53,   31,   31,   31,  136,   31,   53,  144,
  145,   31,   31,   31,   31,   31,   32,   32,   32,  146,
   32,  147,   67,   68,   69,  148,   32,   53,   53,   53,
   70,   71,   72,   73,   74,  152,    1,    3,   67,   68,
   69,   48,   71,   31,   31,   31,   70,   71,   72,   73,
   74,   70,   71,   50,   73,   74,   32,   32,   32,  121,
   40,   45,   75,   76,   77,   41,  135,  131,   42,    0,
    0,    1,    0,    2,    0,    3,   43,    4,   86,   76,
   77,    5,    6,    0,    0,    7,    8,    0,    9,    0,
    0,   10,   11,   12,    0,   40,   13,   14,    0,    0,
   41,   15,    0,   42,    0,    0,    0,    0,    0,    0,
    0,   43,   19,    0,   19,    0,   19,   19,   19,    5,
    3,   44,   19,   19,    0,    0,   19,   19,    0,   19,
    0,    0,   19,   19,   19,    0,    0,   19,   19,    0,
    0,   19,   19,    5,    3,    1,    0,    2,    0,    3,
    0,    4,    0,    0,    0,    5,    6,    0,    0,    7,
    0,    0,    9,    0,    0,   10,   11,   12,    0,   51,
   13,   14,   52,    0,    0,   15,   53,    0,   54,   55,
    0,    0,    0,   56,   57,   67,   68,   69,    0,    0,
   58,   67,    0,   70,   71,   72,   73,   74,    0,   70,
   71,   72,   73,   74,
};
const short yycheck[] = {                                      32,
    0,    0,    0,    0,  262,   49,  302,    3,    4,   75,
    9,    7,    8,   79,   10,   14,   12,  302,  265,   18,
   86,   20,  265,  257,   23,    0,  273,  261,  264,  265,
  272,  272,  265,   32,  259,  260,  272,  262,  272,  277,
  298,  299,  300,   18,  265,  285,   90,  289,   44,   48,
  116,   47,  262,  285,  265,  258,  259,  260,  302,  262,
  302,  302,  128,  301,  130,  268,  302,   63,  302,  265,
   66,  298,  268,  298,  299,  300,   75,  301,   13,  145,
   79,  125,  259,  260,   80,  262,  272,   86,  298,  299,
  300,   26,   27,  268,   93,  298,  299,  300,  264,  273,
  265,   97,  302,  258,  259,  260,  292,  262,  275,  142,
  264,  266,  267,  268,  269,  270,  302,  116,  298,  277,
  265,  298,  299,  300,  265,  121,  298,  298,  124,  128,
  280,  130,   67,   68,   69,   70,   71,   72,   73,   74,
  301,   76,   77,  142,  299,  300,  145,  258,  259,  260,
  302,  262,  263,  264,  265,  266,  267,  268,  269,  270,
  271,  301,  273,  258,  259,  260,  301,  262,  279,  301,
  264,  266,  267,  268,  269,  270,  258,  259,  260,  265,
  262,  303,  258,  259,  260,  301,  268,  298,  299,  300,
  266,  267,  268,  269,  270,  301,    0,  301,  258,  259,
  260,  298,  298,  298,  299,  300,  266,  267,  268,  269,
  270,  266,  267,  298,  269,  270,  298,  299,  300,   89,
  263,  301,  298,  299,  300,  268,  119,  115,  271,   -1,
   -1,  272,   -1,  274,   -1,  276,  279,  278,  298,  299,
  300,  282,  283,   -1,   -1,  286,  287,   -1,  289,   -1,
   -1,  292,  293,  294,   -1,  263,  297,  298,   -1,   -1,
  268,  302,   -1,  271,   -1,   -1,   -1,   -1,   -1,   -1,
   -1,  279,  272,   -1,  274,   -1,  276,  277,  278,  277,
  277,  289,  282,  283,   -1,   -1,  286,  287,   -1,  289,
   -1,   -1,  292,  293,  294,   -1,   -1,  297,  298,   -1,
   -1,  301,  302,  301,  301,  272,   -1,  274,   -1,  276,
   -1,  278,   -1,   -1,   -1,  282,  283,   -1,   -1,  286,
   -1,   -1,  289,   -1,   -1,  292,  293,  294,   -1,  281,
  297,  298,  284,   -1,   -1,  302,  288,   -1,  290,  291,
   -1,   -1,   -1,  295,  296,  258,  259,  260,   -1,   -1,
  302,  258,   -1,  266,  267,  268,  269,  270,   -1,  266,
  267,  268,  269,  270,
};
#define YYFINAL 16
#ifndef YYDEBUG
#define YYDEBUG 0
#endif
#define YYMAXTOKEN 303
#if YYDEBUG
const char * const yyname[] = {
"end-of-file",0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,"_BANG_t","_BANG_EQUALS_t",
"_AMPER_t","_AMPERAMPER_t","_LPAREN_t","_RPAREN_t","_PLUS_EQUALS_t","_COLON_t",
"_SEMIC_t","_LANGLE_t","_LANGLE_EQUALS_t","_EQUALS_t","_RANGLE_t",
"_RANGLE_EQUALS_t","_QUESTION_EQUALS_t","_LBRACKET_t","_RBRACKET_t","ACTIONS_t",
"BIND_t","BREAK_t","CASE_t","CONTINUE_t","DEFAULT_t","ELSE_t","EXISTING_t",
"FOR_t","IF_t","IGNORE_t","IN_t","INCLUDE_t","LOCAL_t","MAXLINE_t","ON_t",
"PIECEMEAL_t","QUIETLY_t","RETURN_t","RULE_t","SWITCH_t","TOGETHER_t",
"UPDATED_t","WHILE_t","_LBRACE_t","_BAR_t","_BARBAR_t","_RBRACE_t","ARG",
"STRING",
};
const char * const yyrule[] = {
"$accept : run",
"run :",
"run : rules",
"block :",
"block : rules",
"rules : rule",
"rules : rule rules",
"rules : LOCAL_t list _SEMIC_t block",
"rules : LOCAL_t list _EQUALS_t list _SEMIC_t block",
"rule : _LBRACE_t block _RBRACE_t",
"rule : INCLUDE_t list _SEMIC_t",
"rule : arg lol _SEMIC_t",
"rule : arg assign list _SEMIC_t",
"rule : arg ON_t list assign list _SEMIC_t",
"rule : BREAK_t list _SEMIC_t",
"rule : CONTINUE_t list _SEMIC_t",
"rule : RETURN_t list _SEMIC_t",
"rule : FOR_t ARG IN_t list _LBRACE_t block _RBRACE_t",
"rule : SWITCH_t list _LBRACE_t cases _RBRACE_t",
"rule : IF_t expr _LBRACE_t block _RBRACE_t",
"rule : IF_t expr _LBRACE_t block _RBRACE_t ELSE_t rule",
"rule : WHILE_t expr _LBRACE_t block _RBRACE_t",
"rule : RULE_t ARG params _LBRACE_t block _RBRACE_t",
"rule : ON_t arg rule",
"$$1 :",
"$$2 :",
"rule : ACTIONS_t eflags ARG bindlist _LBRACE_t $$1 STRING $$2 _RBRACE_t",
"assign : _EQUALS_t",
"assign : _PLUS_EQUALS_t",
"assign : _QUESTION_EQUALS_t",
"assign : DEFAULT_t _EQUALS_t",
"expr : arg",
"expr : expr _EQUALS_t expr",
"expr : expr _BANG_EQUALS_t expr",
"expr : expr _LANGLE_t expr",
"expr : expr _LANGLE_EQUALS_t expr",
"expr : expr _RANGLE_t expr",
"expr : expr _RANGLE_EQUALS_t expr",
"expr : expr _AMPER_t expr",
"expr : expr _AMPERAMPER_t expr",
"expr : expr _BAR_t expr",
"expr : expr _BARBAR_t expr",
"expr : arg IN_t list",
"expr : _BANG_t expr",
"expr : _LPAREN_t expr _RPAREN_t",
"cases :",
"cases : case cases",
"case : CASE_t ARG _COLON_t block",
"params :",
"params : ARG _COLON_t params",
"params : ARG",
"lol : list",
"lol : list _COLON_t lol",
"list : listp",
"listp :",
"listp : listp arg",
"arg : ARG",
"$$3 :",
"arg : _LBRACKET_t $$3 func _RBRACKET_t",
"func : arg lol",
"func : ON_t arg arg lol",
"func : ON_t arg RETURN_t list",
"eflags :",
"eflags : eflags eflag",
"eflag : UPDATED_t",
"eflag : TOGETHER_t",
"eflag : IGNORE_t",
"eflag : QUIETLY_t",
"eflag : PIECEMEAL_t",
"eflag : EXISTING_t",
"eflag : MAXLINE_t ARG",
"bindlist :",
"bindlist : BIND_t list",
};
#endif
#ifndef YYSTYPE
typedef int YYSTYPE;
#endif
#if YYDEBUG
#include <stdio.h>
#endif
#ifdef YYSTACKSIZE
#undef YYMAXDEPTH
#define YYMAXDEPTH YYSTACKSIZE
#else
#ifdef YYMAXDEPTH
#define YYSTACKSIZE YYMAXDEPTH
#else
#define YYSTACKSIZE 10000
#define YYMAXDEPTH 10000
#endif
#endif
#define YYINITSTACKSIZE 200
int yydebug;
int yynerrs;
int yyerrflag;
int yychar;
short *yyssp;
YYSTYPE *yyvsp;
YYSTYPE yyval;
YYSTYPE yylval;
short *yyss;
short *yysslim;
YYSTYPE *yyvs;
int yystacksize;
/* allocate initial stack or double stack size, up to YYMAXDEPTH */
static int yygrowstack()
{
    int newsize, i;
    short *newss;
    YYSTYPE *newvs;

    if ((newsize = yystacksize) == 0)
        newsize = YYINITSTACKSIZE;
    else if (newsize >= YYMAXDEPTH)
        return -1;
    else if ((newsize *= 2) > YYMAXDEPTH)
        newsize = YYMAXDEPTH;
    i = yyssp - yyss;
    newss = yyss ? (short *)realloc(yyss, newsize * sizeof *newss) :
      (short *)malloc(newsize * sizeof *newss);
    if (newss == NULL)
        return -1;
    yyss = newss;
    yyssp = newss + i;
    newvs = yyvs ? (YYSTYPE *)realloc(yyvs, newsize * sizeof *newvs) :
      (YYSTYPE *)malloc(newsize * sizeof *newvs);
    if (newvs == NULL)
        return -1;
    yyvs = newvs;
    yyvsp = newvs + i;
    yystacksize = newsize;
    yysslim = yyss + newsize - 1;
    return 0;
}

#define YYABORT goto yyabort
#define YYREJECT goto yyabort
#define YYACCEPT goto yyaccept
#define YYERROR goto yyerrlab

#ifndef YYPARSE_PARAM
#if defined(__cplusplus) || __STDC__
#define YYPARSE_PARAM_ARG void
#define YYPARSE_PARAM_DECL
#else	/* ! ANSI-C/C++ */
#define YYPARSE_PARAM_ARG
#define YYPARSE_PARAM_DECL
#endif	/* ANSI-C/C++ */
#else	/* YYPARSE_PARAM */
#ifndef YYPARSE_PARAM_TYPE
#define YYPARSE_PARAM_TYPE void *
#endif
#if defined(__cplusplus) || __STDC__
#define YYPARSE_PARAM_ARG YYPARSE_PARAM_TYPE YYPARSE_PARAM
#define YYPARSE_PARAM_DECL
#else	/* ! ANSI-C/C++ */
#define YYPARSE_PARAM_ARG YYPARSE_PARAM
#define YYPARSE_PARAM_DECL YYPARSE_PARAM_TYPE YYPARSE_PARAM;
#endif	/* ANSI-C/C++ */
#endif	/* ! YYPARSE_PARAM */

int
yyparse (YYPARSE_PARAM_ARG)
    YYPARSE_PARAM_DECL
{
    register int yym, yyn, yystate;
#if YYDEBUG
    register const char *yys;

    if ((yys = getenv("YYDEBUG")))
    {
        yyn = *yys;
        if (yyn >= '0' && yyn <= '9')
            yydebug = yyn - '0';
    }
#endif

    yynerrs = 0;
    yyerrflag = 0;
    yychar = (-1);

    if (yyss == NULL && yygrowstack()) goto yyoverflow;
    yyssp = yyss;
    yyvsp = yyvs;
    *yyssp = yystate = 0;

yyloop:
    if ((yyn = yydefred[yystate])) goto yyreduce;
    if (yychar < 0)
    {
        if ((yychar = yylex()) < 0) yychar = 0;
#if YYDEBUG
        if (yydebug)
        {
            yys = 0;
            if (yychar <= YYMAXTOKEN) yys = yyname[yychar];
            if (!yys) yys = "illegal-symbol";
            printf("%sdebug: state %d, reading %d (%s)\n",
                    YYPREFIX, yystate, yychar, yys);
        }
#endif
    }
    if ((yyn = yysindex[yystate]) && (yyn += yychar) >= 0 &&
            yyn <= YYTABLESIZE && yycheck[yyn] == yychar)
    {
#if YYDEBUG
        if (yydebug)
            printf("%sdebug: state %d, shifting to state %d\n",
                    YYPREFIX, yystate, yytable[yyn]);
#endif
        if (yyssp >= yysslim && yygrowstack())
        {
            goto yyoverflow;
        }
        *++yyssp = yystate = yytable[yyn];
        *++yyvsp = yylval;
        yychar = (-1);
        if (yyerrflag > 0)  --yyerrflag;
        goto yyloop;
    }
    if ((yyn = yyrindex[yystate]) && (yyn += yychar) >= 0 &&
            yyn <= YYTABLESIZE && yycheck[yyn] == yychar)
    {
        yyn = yytable[yyn];
        goto yyreduce;
    }
    if (yyerrflag) goto yyinrecovery;
#if defined(lint) || defined(__GNUC__)
    goto yynewerror;
#endif
yynewerror:
    yyerror("syntax error");
#if defined(lint) || defined(__GNUC__)
    goto yyerrlab;
#endif
yyerrlab:
    ++yynerrs;
yyinrecovery:
    if (yyerrflag < 3)
    {
        yyerrflag = 3;
        for (;;)
        {
            if ((yyn = yysindex[*yyssp]) && (yyn += YYERRCODE) >= 0 &&
                    yyn <= YYTABLESIZE && yycheck[yyn] == YYERRCODE)
            {
#if YYDEBUG
                if (yydebug)
                    printf("%sdebug: state %d, error recovery shifting\
 to state %d\n", YYPREFIX, *yyssp, yytable[yyn]);
#endif
                if (yyssp >= yysslim && yygrowstack())
                {
                    goto yyoverflow;
                }
                *++yyssp = yystate = yytable[yyn];
                *++yyvsp = yylval;
                goto yyloop;
            }
            else
            {
#if YYDEBUG
                if (yydebug)
                    printf("%sdebug: error recovery discarding state %d\n",
                            YYPREFIX, *yyssp);
#endif
                if (yyssp <= yyss) goto yyabort;
                --yyssp;
                --yyvsp;
            }
        }
    }
    else
    {
        if (yychar == 0) goto yyabort;
#if YYDEBUG
        if (yydebug)
        {
            yys = 0;
            if (yychar <= YYMAXTOKEN) yys = yyname[yychar];
            if (!yys) yys = "illegal-symbol";
            printf("%sdebug: state %d, error recovery discards token %d (%s)\n",
                    YYPREFIX, yystate, yychar, yys);
        }
#endif
        yychar = (-1);
        goto yyloop;
    }
yyreduce:
#if YYDEBUG
    if (yydebug)
        printf("%sdebug: state %d, reducing by rule %d (%s)\n",
                YYPREFIX, yystate, yyn, yyrule[yyn]);
#endif
    yym = yylen[yyn];
    yyval = yyvsp[1-yym];
    switch (yyn)
    {
case 2:
#line 130 "jamgram.y"
{ parse_save( yyvsp[0].parse ); }
break;
case 3:
#line 141 "jamgram.y"
{ yyval.parse = pnull(); }
break;
case 4:
#line 143 "jamgram.y"
{ yyval.parse = yyvsp[0].parse; }
break;
case 5:
#line 147 "jamgram.y"
{ yyval.parse = yyvsp[0].parse; }
break;
case 6:
#line 149 "jamgram.y"
{ yyval.parse = prules( yyvsp[-1].parse, yyvsp[0].parse ); }
break;
case 7:
#line 151 "jamgram.y"
{ yyval.parse = plocal( yyvsp[-2].parse, pnull(), yyvsp[0].parse ); }
break;
case 8:
#line 153 "jamgram.y"
{ yyval.parse = plocal( yyvsp[-4].parse, yyvsp[-2].parse, yyvsp[0].parse ); }
break;
case 9:
#line 157 "jamgram.y"
{ yyval.parse = yyvsp[-1].parse; }
break;
case 10:
#line 159 "jamgram.y"
{ yyval.parse = pincl( yyvsp[-1].parse ); }
break;
case 11:
#line 161 "jamgram.y"
{ yyval.parse = prule( yyvsp[-2].parse, yyvsp[-1].parse ); }
break;
case 12:
#line 163 "jamgram.y"
{ yyval.parse = pset( yyvsp[-3].parse, yyvsp[-1].parse, yyvsp[-2].number ); }
break;
case 13:
#line 165 "jamgram.y"
{ yyval.parse = pset1( yyvsp[-5].parse, yyvsp[-3].parse, yyvsp[-1].parse, yyvsp[-2].number ); }
break;
case 14:
#line 167 "jamgram.y"
{ yyval.parse = pbreak( yyvsp[-1].parse, JMP_BREAK ); }
break;
case 15:
#line 169 "jamgram.y"
{ yyval.parse = pbreak( yyvsp[-1].parse, JMP_CONTINUE ); }
break;
case 16:
#line 171 "jamgram.y"
{ yyval.parse = pbreak( yyvsp[-1].parse, JMP_RETURN ); }
break;
case 17:
#line 173 "jamgram.y"
{ yyval.parse = pfor( yyvsp[-5].string, yyvsp[-3].parse, yyvsp[-1].parse ); }
break;
case 18:
#line 175 "jamgram.y"
{ yyval.parse = pswitch( yyvsp[-3].parse, yyvsp[-1].parse ); }
break;
case 19:
#line 177 "jamgram.y"
{ yyval.parse = pif( yyvsp[-3].parse, yyvsp[-1].parse, pnull() ); }
break;
case 20:
#line 179 "jamgram.y"
{ yyval.parse = pif( yyvsp[-5].parse, yyvsp[-3].parse, yyvsp[0].parse ); }
break;
case 21:
#line 181 "jamgram.y"
{ yyval.parse = pwhile( yyvsp[-3].parse, yyvsp[-1].parse ); }
break;
case 22:
#line 183 "jamgram.y"
{ yyval.parse = psetc( yyvsp[-4].string, yyvsp[-3].parse, yyvsp[-1].parse ); }
break;
case 23:
#line 185 "jamgram.y"
{ yyval.parse = pon( yyvsp[-1].parse, yyvsp[0].parse ); }
break;
case 24:
#line 187 "jamgram.y"
{ yymode( SCAN_STRING ); }
break;
case 25:
#line 189 "jamgram.y"
{ yymode( SCAN_NORMAL ); }
break;
case 26:
#line 191 "jamgram.y"
{ yyval.parse = psete( yyvsp[-6].string,yyvsp[-5].parse,yyvsp[-2].string,yyvsp[-7].number ); }
break;
case 27:
#line 199 "jamgram.y"
{ yyval.number = VAR_SET; }
break;
case 28:
#line 201 "jamgram.y"
{ yyval.number = VAR_APPEND; }
break;
case 29:
#line 203 "jamgram.y"
{ yyval.number = VAR_DEFAULT; }
break;
case 30:
#line 205 "jamgram.y"
{ yyval.number = VAR_DEFAULT; }
break;
case 31:
#line 213 "jamgram.y"
{ yyval.parse = peval( EXPR_EXISTS, yyvsp[0].parse, pnull() ); }
break;
case 32:
#line 215 "jamgram.y"
{ yyval.parse = peval( EXPR_EQUALS, yyvsp[-2].parse, yyvsp[0].parse ); }
break;
case 33:
#line 217 "jamgram.y"
{ yyval.parse = peval( EXPR_NOTEQ, yyvsp[-2].parse, yyvsp[0].parse ); }
break;
case 34:
#line 219 "jamgram.y"
{ yyval.parse = peval( EXPR_LESS, yyvsp[-2].parse, yyvsp[0].parse ); }
break;
case 35:
#line 221 "jamgram.y"
{ yyval.parse = peval( EXPR_LESSEQ, yyvsp[-2].parse, yyvsp[0].parse ); }
break;
case 36:
#line 223 "jamgram.y"
{ yyval.parse = peval( EXPR_MORE, yyvsp[-2].parse, yyvsp[0].parse ); }
break;
case 37:
#line 225 "jamgram.y"
{ yyval.parse = peval( EXPR_MOREEQ, yyvsp[-2].parse, yyvsp[0].parse ); }
break;
case 38:
#line 227 "jamgram.y"
{ yyval.parse = peval( EXPR_AND, yyvsp[-2].parse, yyvsp[0].parse ); }
break;
case 39:
#line 229 "jamgram.y"
{ yyval.parse = peval( EXPR_AND, yyvsp[-2].parse, yyvsp[0].parse ); }
break;
case 40:
#line 231 "jamgram.y"
{ yyval.parse = peval( EXPR_OR, yyvsp[-2].parse, yyvsp[0].parse ); }
break;
case 41:
#line 233 "jamgram.y"
{ yyval.parse = peval( EXPR_OR, yyvsp[-2].parse, yyvsp[0].parse ); }
break;
case 42:
#line 235 "jamgram.y"
{ yyval.parse = peval( EXPR_IN, yyvsp[-2].parse, yyvsp[0].parse ); }
break;
case 43:
#line 237 "jamgram.y"
{ yyval.parse = peval( EXPR_NOT, yyvsp[0].parse, pnull() ); }
break;
case 44:
#line 239 "jamgram.y"
{ yyval.parse = yyvsp[-1].parse; }
break;
case 45:
#line 249 "jamgram.y"
{ yyval.parse = P0; }
break;
case 46:
#line 251 "jamgram.y"
{ yyval.parse = pnode( yyvsp[-1].parse, yyvsp[0].parse ); }
break;
case 47:
#line 255 "jamgram.y"
{ yyval.parse = psnode( yyvsp[-2].string, yyvsp[0].parse ); }
break;
case 48:
#line 264 "jamgram.y"
{ yyval.parse = P0; }
break;
case 49:
#line 266 "jamgram.y"
{ yyval.parse = psnode( yyvsp[-2].string, yyvsp[0].parse ); }
break;
case 50:
#line 268 "jamgram.y"
{ yyval.parse = psnode( yyvsp[0].string, P0 ); }
break;
case 51:
#line 277 "jamgram.y"
{ yyval.parse = pnode( P0, yyvsp[0].parse ); }
break;
case 52:
#line 279 "jamgram.y"
{ yyval.parse = pnode( yyvsp[0].parse, yyvsp[-2].parse ); }
break;
case 53:
#line 289 "jamgram.y"
{ yyval.parse = yyvsp[0].parse; yymode( SCAN_NORMAL ); }
break;
case 54:
#line 293 "jamgram.y"
{ yyval.parse = pnull(); yymode( SCAN_PUNCT ); }
break;
case 55:
#line 295 "jamgram.y"
{ yyval.parse = pappend( yyvsp[-1].parse, yyvsp[0].parse ); }
break;
case 56:
#line 299 "jamgram.y"
{ yyval.parse = plist( yyvsp[0].string ); }
break;
case 57:
#line 300 "jamgram.y"
{ yymode( SCAN_NORMAL ); }
break;
case 58:
#line 301 "jamgram.y"
{ yyval.parse = yyvsp[-1].parse; }
break;
case 59:
#line 310 "jamgram.y"
{ yyval.parse = prule( yyvsp[-1].parse, yyvsp[0].parse ); }
break;
case 60:
#line 312 "jamgram.y"
{ yyval.parse = pon( yyvsp[-2].parse, prule( yyvsp[-1].parse, yyvsp[0].parse ) ); }
break;
case 61:
#line 314 "jamgram.y"
{ yyval.parse = pon( yyvsp[-2].parse, yyvsp[0].parse ); }
break;
case 62:
#line 323 "jamgram.y"
{ yyval.number = 0; }
break;
case 63:
#line 325 "jamgram.y"
{ yyval.number = yyvsp[-1].number | yyvsp[0].number; }
break;
case 64:
#line 329 "jamgram.y"
{ yyval.number = RULE_UPDATED; }
break;
case 65:
#line 331 "jamgram.y"
{ yyval.number = RULE_TOGETHER; }
break;
case 66:
#line 333 "jamgram.y"
{ yyval.number = RULE_IGNORE; }
break;
case 67:
#line 335 "jamgram.y"
{ yyval.number = RULE_QUIETLY; }
break;
case 68:
#line 337 "jamgram.y"
{ yyval.number = RULE_PIECEMEAL; }
break;
case 69:
#line 339 "jamgram.y"
{ yyval.number = RULE_EXISTING; }
break;
case 70:
#line 341 "jamgram.y"
{ yyval.number = atoi( yyvsp[0].string ) * RULE_MAXLINE; }
break;
case 71:
#line 350 "jamgram.y"
{ yyval.parse = pnull(); }
break;
case 72:
#line 352 "jamgram.y"
{ yyval.parse = yyvsp[0].parse; }
break;
#line 877 "y.tab.c"
    }
    yyssp -= yym;
    yystate = *yyssp;
    yyvsp -= yym;
    yym = yylhs[yyn];
    if (yystate == 0 && yym == 0)
    {
#if YYDEBUG
        if (yydebug)
            printf("%sdebug: after reduction, shifting from state 0 to\
 state %d\n", YYPREFIX, YYFINAL);
#endif
        yystate = YYFINAL;
        *++yyssp = YYFINAL;
        *++yyvsp = yyval;
        if (yychar < 0)
        {
            if ((yychar = yylex()) < 0) yychar = 0;
#if YYDEBUG
            if (yydebug)
            {
                yys = 0;
                if (yychar <= YYMAXTOKEN) yys = yyname[yychar];
                if (!yys) yys = "illegal-symbol";
                printf("%sdebug: state %d, reading %d (%s)\n",
                        YYPREFIX, YYFINAL, yychar, yys);
            }
#endif
        }
        if (yychar == 0) goto yyaccept;
        goto yyloop;
    }
    if ((yyn = yygindex[yym]) && (yyn += yystate) >= 0 &&
            yyn <= YYTABLESIZE && yycheck[yyn] == yystate)
        yystate = yytable[yyn];
    else
        yystate = yydgoto[yym];
#if YYDEBUG
    if (yydebug)
        printf("%sdebug: after reduction, shifting from state %d \
to state %d\n", YYPREFIX, *yyssp, yystate);
#endif
    if (yyssp >= yysslim && yygrowstack())
    {
        goto yyoverflow;
    }
    *++yyssp = yystate;
    *++yyvsp = yyval;
    goto yyloop;
yyoverflow:
    yyerror("yacc stack overflow");
yyabort:
    return (1);
yyaccept:
    return (0);
}
