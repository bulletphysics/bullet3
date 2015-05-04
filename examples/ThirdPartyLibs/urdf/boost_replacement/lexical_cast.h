#ifndef BOOST_REPLACEMENT_LEXICAL_CAST_H
#define BOOST_REPLACEMENT_LEXICAL_CAST_H

#include <stdlib.h>

namespace boost
{

	template <typename T> T lexical_cast(const char* txt)
	{
		double result = atof(txt);
		return result;
	};

	struct bad_lexical_cast
	{
		const char* what()
		{
		return ("bad lexical cast\n");
		}
		
	};
	
} //namespace boost

#endif

