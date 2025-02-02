

#include <assert.h>
//#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "urdfStringSplit.h"

void urdfStringSplit(btAlignedObjectArray<std::string> &pieces, const std::string &vector_str, const btAlignedObjectArray<std::string> &separators)
{
	assert(separators.size() == 1);
	if (separators.size() == 1)
	{
		char **strArray = urdfStrSplit(vector_str.c_str(), separators[0].c_str());
		int numSubStr = (int)urdfStrArrayLen(strArray);
		for (int i = 0; i < numSubStr; i++)
			pieces.push_back(std::string(strArray[i]));
		urdfStrArrayFree(strArray);
	}
}
void urdfIsAnyOf(const char *seps, btAlignedObjectArray<std::string> &strArray)
{
	int numSeps = (int)strlen(seps);
	for (int i = 0; i < numSeps; i++)
	{
		char sep2[2] = {0, 0};

		sep2[0] = seps[i];
		strArray.push_back(sep2);
	}
}

/* Append an item to a dynamically allocated array of strings. On failure,
 return NULL, in which case the original array is intact. The item
 string is dynamically copied. If the array is NULL, allocate a new
 array. Otherwise, extend the array. Make sure the array is always
 NULL-terminated. Input string might not be '\0'-terminated. */
char **urdfStrArrayAppend(char **array, size_t nitems, const char *item,
						  size_t itemlen)
{
	/* Make a dynamic copy of the item. */
	char *copy;
	if (item == NULL)
		copy = NULL;
	else
	{
		copy = (char *)malloc(itemlen + 1);
		if (copy == NULL)
			return NULL;
		memcpy(copy, item, itemlen);
		copy[itemlen] = '\0';
	}

	/* Extend array with one element. Except extend it by two elements,
	 in case it did not yet exist. This might mean it is a teeny bit
	 too big, but we don't care. */
	char** reallocated = (char **)realloc(array, (nitems + 2) * sizeof(array[0]));
	if (reallocated == NULL)
	{
		free(copy);
		return NULL;
	}
	else
		array = reallocated;

	/* Add copy of item to array, and return it. */
	array[nitems] = copy;
	array[nitems + 1] = NULL;
	return array;
}

/* Free a dynamic array of dynamic strings. */
void urdfStrArrayFree(char **array)
{
	if (array == NULL)
		return;
	size_t i = 0;
	while(array[i] != NULL)
	{
		free(array[i]);
		 ++i;
	}
	free(array);
}

/* Split a string into substrings. Return dynamic array of dynamically
 allocated substrings, or NULL if there was an error. Caller is
 expected to free the memory, for example with str_array_free. */
char **urdfStrSplit(const char *input, const char *sep)
{
	size_t nitems = 0;
	char **array = NULL;
	const char *start = input;
	const char *next = strstr(start, sep);
	size_t seplen = strlen(sep);
	const char *item;
	size_t itemlen;
	(void)next;
	
	for (;;)
	{
		next = strstr(start, sep);
		if (next == NULL)
		{
			/* Add the remaining string (or empty string, if input ends with
			 separator. */
			char **newstr = urdfStrArrayAppend(array, nitems, start, strlen(start));
			if (newstr == NULL)
			{
				urdfStrArrayFree(array);
				return NULL;
			}
			array = newstr;
			++nitems;
			break;
		}
		else if (next == input)
		{
			/* Input starts with separator. */
			item = "";
			itemlen = 0;
		}
		else
		{
			item = start;
			itemlen = (size_t)(next - item);
		}
		char **newstr = urdfStrArrayAppend(array, nitems, item, itemlen);
		if (newstr == NULL)
		{
			urdfStrArrayFree(array);
			return NULL;
		}
		array = newstr;
		++nitems;
		start = next + seplen;
	}

	if (nitems == 0)
	{
		/* Input does not contain separator at all. */
		assert(array == NULL);
		array = urdfStrArrayAppend(array, nitems, input, strlen(input));
	}

	return array;
}

/* Return length of a NULL-delimited array of strings. */
size_t urdfStrArrayLen(char **array)
{
	size_t len;

	for (len = 0; array[len] != NULL; ++len)
		continue;
	return len;
}

#ifdef UNIT_TEST_STRING_SPLIT

#define MAX_OUTPUT 20

int main(void)
{
	struct
	{
		const char *input;
		const char *sep;
		char *output[MAX_OUTPUT];
	} tab[] = {
		/* Input is empty string. Output should be a list with an empty
		 string. */
		{
			"",
			"and",
			{
				"",
				NULL,
			},
		},
		/* Input is exactly the separator. Output should be two empty
		 strings. */
		{
			"and",
			"and",
			{
				"",
				"",
				NULL,
			},
		},
		/* Input is non-empty, but does not have separator. Output should
		 be the same string. */
		{
			"foo",
			"and",
			{
				"foo",
				NULL,
			},
		},
		/* Input is non-empty, and does have separator. */
		{
			"foo bar 1 and foo bar 2",
			" and ",
			{
				"foo bar 1",
				"foo bar 2",
				NULL,
			},
		},
	};
	const int tab_len = sizeof(tab) / sizeof(tab[0]);
	bool errors;

	errors = false;

	for (int i = 0; i < tab_len; ++i)
	{
		printf("test %d\n", i);

		char **output = str_split(tab[i].input, tab[i].sep);
		if (output == NULL)
		{
			fprintf(stderr, "output is NULL\n");
			errors = true;
			break;
		}
		size_t num_output = str_array_len(output);
		printf("num_output %lu\n", (unsigned long)num_output);

		size_t num_correct = str_array_len(tab[i].output);
		if (num_output != num_correct)
		{
			fprintf(stderr, "wrong number of outputs (%lu, not %lu)\n",
					(unsigned long)num_output, (unsigned long)num_correct);
			errors = true;
		}
		else
		{
			for (size_t j = 0; j < num_output; ++j)
			{
				if (strcmp(tab[i].output[j], output[j]) != 0)
				{
					fprintf(stderr, "output[%lu] is '%s' not '%s'\n",
							(unsigned long)j, output[j], tab[i].output[j]);
					errors = true;
					break;
				}
			}
		}

		str_array_free(output);
		printf("\n");
	}

	if (errors)
		return EXIT_FAILURE;
	return 0;
}
#endif  //UNIT_TEST_STRING_SPLIT
