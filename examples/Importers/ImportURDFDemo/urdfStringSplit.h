
#ifndef STRING_SPLIT_H
#define STRING_SPLIT_H

#include <cstring>
#include <string>
#include "LinearMath/btAlignedObjectArray.h"


void urdfStringSplit( btAlignedObjectArray<std::string>&pieces, const std::string& vector_str, const btAlignedObjectArray<std::string>& separators);

void urdfIsAnyOf(const char* seps, btAlignedObjectArray<std::string>& strArray);

///The string split C code is by Lars Wirzenius
///See http://stackoverflow.com/questions/2531605/how-to-split-a-string-with-a-delimiter-larger-than-one-single-char


/* Split a string into substrings. Return dynamic array of dynamically
 allocated substrings, or NULL if there was an error. Caller is
 expected to free the memory, for example with str_array_free. */
char**	urdfStrSplit(const char* input, const char* sep);

/* Free a dynamic array of dynamic strings. */
void urdfStrArrayFree(char** array);

/* Return length of a NULL-delimited array of strings. */
size_t urdfStrArrayLen(char** array);

#endif //STRING_SPLIT_H

