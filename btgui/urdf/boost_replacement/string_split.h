
#ifndef STRING_SPLIT_H
#define STRING_SPLIT_H

#include <cstring>
#include <vector>
#include <string>

namespace boost
{
	void split( std::vector<std::string>&pieces, const std::string& vector_str, std::vector<std::string> separators);
	std::vector<std::string> is_any_of(const char* seps);
};

///The string split C code is by Lars Wirzenius
///See http://stackoverflow.com/questions/2531605/how-to-split-a-string-with-a-delimiter-larger-than-one-single-char


/* Split a string into substrings. Return dynamic array of dynamically
 allocated substrings, or NULL if there was an error. Caller is
 expected to free the memory, for example with str_array_free. */
char**	str_split(const char* input, const char* sep);

/* Free a dynamic array of dynamic strings. */
void str_array_free(char** array);

/* Return length of a NULL-delimited array of strings. */
size_t str_array_len(char** array);

#endif //STRING_SPLIT_H

