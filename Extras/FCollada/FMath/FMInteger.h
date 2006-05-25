/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FMInteger.h
	The file containing functions and constants for integer values.
 */

/** A dynamically-sized array of 32-bit signed integer values. */
typedef vector<int32> Int32List;
/** A dynamically-sized array of 32-bit unsigned integer values. */
typedef vector<uint32> UInt32List;
/** A dynamically-sized array of 16-bit unsigned integer values. */
typedef vector<uint16> UInt16List;
/** A dynamically-sized array of 8-bit unsigned integer values. */
typedef vector<uint8> UInt8List;
/** A dynamically-sized array of 8-bit signed integer values. */
typedef vector<int8> Int8List;
/** A dynamically-sized array of boolean values. */
typedef vector<bool> BooleanList;

/** Returns whether two signed or unsigned integers are equivalent.
	For integers, this function simply wraps around the operator==.
	@param i1 A first integer.
	@param i2 A second integer.
	@return Whether the two integers are equivalent. */
inline bool IsEquivalent(int8 i1, int8 i2) { return i1 == i2; }
inline bool IsEquivalent(uint8 i1, uint8 i2) { return i1 == i2; } /**< See above. */
inline bool IsEquivalent(int16 i1, int16 i2) { return i1 == i2; } /**< See above. */
inline bool IsEquivalent(uint16 i1, uint16 i2) { return i1 == i2; } /**< See above. */
inline bool IsEquivalent(int32 i1, int32 i2) { return i1 == i2; } /**< See above. */
inline bool IsEquivalent(uint32 i1, uint32 i2) { return i1 == i2; } /**< See above. */
inline bool IsEquivalent(int64 i1, int64 i2) { return i1 == i2; } /**< See above. */
inline bool IsEquivalent(uint64 i1, uint64 i2) { return i1 == i2; } /**< See above. */

