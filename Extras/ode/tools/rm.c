/*



this is a simple replacement for the windows command line 'del' command.

this is more convenient to use in the ODE makefile because of the following

differences from 'del' :

  * filenames may contain forward slashes.

  * wildcard expansion is not performed.

  * it is called 'rm', just like in unix.



*/



#include <stdio.h>

#include <stdlib.h>



int main (int argc, char **argv)

{

  int i;

  for (i=1; i<argc; i++) unlink (argv[i]);

  return 0;

}

