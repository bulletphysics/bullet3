/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FMArray.h
	The file contains the vector class, which improves on the standard C++ vector class.
 */

/**
	A dynamically-sized array.
	Built on top of the standard C++ vector class, this class improves on the interface
	by adding the following functionality:
	- constructor that takes in a constant-sized array.
	- comparison which a constant-sized array.

	@ingroup FMath
*/

template <class T>
class FCOLLADA_EXPORT vector : public std::vector<T>
{
public:
	/** Default constructor. */
	vector() : std::vector<T>() {}

	/** Constructor: Builds a dynamically-sized array of the wanted size.
		@param size The wanted size of the array. */
	vector(size_t size) : std::vector<T>(size) {}

	/** Constructor: Builds a dynamically-sized array of the wanted size.
		@param size The wanted size of the array
		@param defaultValue The default value to use for all the entries of the array. */
	vector(size_t size, const T& defaultValue) : std::vector<T>(size, defaultValue) {}

	/** Copy constructor.
		@param copy The dynamically-sized array to copy the values from. */
	vector(const std::vector<T>& copy) : std::vector<T>(copy) {}

	/** Constructor: Builds a dynamically-sized array from a constant-sized array.
		@param values A constant-sized array of floating-point values.
		@param count The size of the constant-sized array. */
	vector(const T* values, size_t count) : std::vector<T>()
	{
		resize(count);
		memcpy(&at(0), values, count * sizeof(T));
	}
};

/** Returns whether a dynamically-sized array is equivalent to a constant-sized array.
	@param dl A dynamically-sized array.
	@param cl A constant-sized array.
	@param count The size of the constant-sized array.
	@return Whether the two arrays are equivalent. */
template <typename T>
inline bool IsEquivalent(const vector<T>& dl, const T* cl, size_t count)
{
	if (dl.size() != count) return false;
	bool equivalent = true;
	for (size_t i = 0; i < count && equivalent; ++i) equivalent = IsEquivalent(dl.at(i), cl[i]);
	return equivalent;
}