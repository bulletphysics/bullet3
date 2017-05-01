#ifndef B3_HASH_STRING_H
#define B3_HASH_STRING_H

#include <string>

///very basic hashable string implementation, compatible with b3HashMap
struct b3HashString
{
	std::string m_string;
	unsigned int	m_hash;

	B3_FORCE_INLINE	unsigned int getHash()const
	{
		return m_hash;
	}


	b3HashString(const char* name)
		:m_string(name)
	{

		/* magic numbers from http://www.isthe.com/chongo/tech/comp/fnv/ */
		static const unsigned int  InitialFNV = 2166136261u;
		static const unsigned int FNVMultiple = 16777619u;

		/* Fowler / Noll / Vo (FNV) Hash */
		unsigned int hash = InitialFNV;
		
		for(int i = 0; m_string[i]; i++)
		{
			hash = hash ^ (m_string[i]);       /* xor  the low 8 bits */
			hash = hash * FNVMultiple;  /* multiply by the magic number */
		}
		m_hash = hash;
	}

	int portableStringCompare(const char* src,	const char* dst) const
	{
			int ret = 0 ;

			while( ! (ret = *(unsigned char *)src - *(unsigned char *)dst) && *dst)
					++src, ++dst;

			if ( ret < 0 )
					ret = -1 ;
			else if ( ret > 0 )
					ret = 1 ;

			return( ret );
	}

	bool equals(const b3HashString& other) const
	{
		return (m_string == other.m_string);
	}

};

#endif //B3_HASH_STRING_H
