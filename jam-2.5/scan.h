/*
 * Copyright 1993, 1995 Christopher Seiwald.
 *
 * This file is part of Jam - see jam.c for Copyright information.
 */

/*
 * scan.h - the jam yacc scanner
 *
 * External functions:
 *
 *	yyerror( char *s ) - print a parsing error message
 *	yyfparse( char *s ) - scan include file s
 *	yylex() - parse the next token, returning its type
 *	yymode() - adjust lexicon of scanner
 *	yyparse() - declaration for yacc parser
 *	yyanyerrors() - indicate if any parsing errors occured
 *
 * The yymode() function is for the parser to adjust the lexicon of the
 * scanner.  Aside from normal keyword scanning, there is a mode to
 * handle action strings (look only for the closing }) and a mode to 
 * ignore most keywords when looking for a punctuation keyword.  This 
 * allows non-punctuation keywords to be used in lists without quoting.
 *
 * 11/04/02 (seiwald) - const-ing for string literals
 */

/*
 * YYSTYPE - value of a lexical token
 */

# define YYSTYPE YYSYMBOL

typedef struct _YYSTYPE {
	int		type;
	const char	*string;
	PARSE		*parse;
	LIST		*list;
	int		number;
} YYSTYPE;

extern YYSTYPE yylval;

void yymode( int n );
void yyerror( const char *s );
int yyanyerrors();
void yyfparse( const char *s );
int yyline();
int yylex();
int yyparse();

# define SCAN_NORMAL	0	/* normal parsing */
# define SCAN_STRING	1	/* look only for matching } */
# define SCAN_PUNCT	2	/* only punctuation keywords */
