///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains hashing code.
 *	\file		IceHashing.h
 *	\author		Pierre Terdiman
 *	\date		May, 08, 1999
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef ICEHASHING_H
#define ICEHASHING_H

	#define HashSize(n)	((udword)1<<(n))
	#define HashMask(n)	(HashSize(n)-1)

	ICECORE_API udword Hash(const char* str);
	ICECORE_API udword Hash(ubyte* k, udword length, udword initval);

	// Bob Jenkin's hash
	inline_	unsigned int Hash32Bits_0(unsigned int key)
	{
		key += (key << 12);
		key ^= (key >> 22);
		key += (key << 4);
		key ^= (key >> 9);
		key += (key << 10);
		key ^= (key >> 2);
		key += (key << 7);
		key ^= (key >> 12);
		return key;
	}

	// Thomas Wang's hash
	inline_ int Hash32Bits_1(int key)
	{
		key += ~(key << 15);
		key ^=  (key >> 10);
		key +=  (key << 3);
		key ^=  (key >> 6);
		key += ~(key << 11);
		key ^=  (key >> 16);
		return key;
	}

	// Thomas Wang's hash
	inline_	__int64 Hash64Bits_0(__int64 key)
	{
		key += ~(key << 32);
		key ^= (key >> 22);
		key += ~(key << 13);
		key ^= (key >> 8);
		key += (key << 3);
		key ^= (key >> 15);
		key += ~(key << 27);
		key ^= (key >> 31);
		return key;
	}

	inline_	__int64 Hash64Bits_1(__int64 key)
	{
		__int64 c1 = 0x6e5ea73858134343L;
		__int64 c2 = 0xb34e8f99a2ec9ef5L;
		key ^= ((c1 ^ key) >> 32);
		key *= c1;
		key ^= ((c2 ^ key) >> 31);
		key *= c2;
		key ^= ((c1 ^ key) >> 32);
		return key;
	}

	inline_ udword Hash(udword id0, udword id1)
	{
		return Hash32Bits_1( (id0&0xffff)|(id1<<16) );
	}

#endif // ICEHASHING_H
