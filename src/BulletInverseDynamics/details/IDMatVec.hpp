/// @file Built-In Matrix-Vector functions
#ifndef IDMATVEC_HPP_
#define IDMATVEC_HPP_

#include <cstdlib>

#include "../IDConfig.hpp"

namespace btInverseDynamics {
class vec3;
class vecx;
class mat33;
class matxx;

/// This is a very basic implementation to enable stand-alone use of the library.
/// The implementation is not really optimized and misses many features that you would
/// want from a "fully featured" linear math library.
class vec3 {
public:
	idScalar& operator()(int i) { return m_data[i]; }
	const idScalar& operator()(int i) const { return m_data[i]; }
	const int size() const { return 3; }
	const vec3& operator=(const vec3& rhs);
	const vec3& operator+=(const vec3& b);
	const vec3& operator-=(const vec3& b);
	vec3 cross(const vec3& b) const;
	idScalar dot(const vec3& b) const;

	friend vec3 operator*(const mat33& a, const vec3& b);
	friend vec3 operator*(const vec3& a, const idScalar& s);
	friend vec3 operator*(const idScalar& s, const vec3& a);

	friend vec3 operator+(const vec3& a, const vec3& b);
	friend vec3 operator-(const vec3& a, const vec3& b);
	friend vec3 operator/(const vec3& a, const idScalar& s);

private:
	idScalar m_data[3];
};

class mat33 {
public:
	idScalar& operator()(int i, int j) { return m_data[3 * i + j]; }
	const idScalar& operator()(int i, int j) const { return m_data[3 * i + j]; }
	const mat33& operator=(const mat33& rhs);
	mat33 transpose() const;
	const mat33& operator+=(const mat33& b);
	const mat33& operator-=(const mat33& b);

	friend mat33 operator*(const mat33& a, const mat33& b);
	friend vec3 operator*(const mat33& a, const vec3& b);
	friend mat33 operator*(const mat33& a, const idScalar& s);
	friend mat33 operator*(const idScalar& s, const mat33& a);
	friend mat33 operator+(const mat33& a, const mat33& b);
	friend mat33 operator-(const mat33& a, const mat33& b);
	friend mat33 operator/(const mat33& a, const idScalar& s);

private:
	// layout is [0,1,2;3,4,5;6,7,8]
	idScalar m_data[9];
};

class vecx {
public:
	vecx(int size) : m_size(size) {
		m_data = static_cast<idScalar*>(idMalloc(sizeof(idScalar) * size));
	}
	~vecx() { idFree(m_data); }
	const vecx& operator=(const vecx& rhs);
	idScalar& operator()(int i) { return m_data[i]; }
	const idScalar& operator()(int i) const { return m_data[i]; }
	const int& size() const { return m_size; }

	friend vecx operator*(const vecx& a, const idScalar& s);
	friend vecx operator*(const idScalar& s, const vecx& a);

	friend vecx operator+(const vecx& a, const vecx& b);
	friend vecx operator-(const vecx& a, const vecx& b);
	friend vecx operator/(const vecx& a, const idScalar& s);

private:
	int m_size;
	idScalar* m_data;
};

class matxx {
public:
	matxx(int rows, int cols) : m_rows(rows), m_cols(cols) {
		m_data = static_cast<idScalar*>(idMalloc(sizeof(idScalar) * rows * cols));
	}
	~matxx() { idFree(m_data); }
	const matxx& operator=(const matxx& rhs);
	idScalar& operator()(int row, int col) { return m_data[row * m_rows + col]; }
	const idScalar& operator()(int row, int col) const { return m_data[row * m_rows + col]; }
	const int& rows() const { return m_rows; }
	const int& cols() const { return m_cols; }

private:
	int m_rows;
	int m_cols;
	idScalar* m_data;
};

inline const vec3& vec3::operator=(const vec3& rhs) {
	if (&rhs != this) {
		memcpy(m_data, rhs.m_data, 3 * sizeof(idScalar));
	}
	return *this;
}

inline vec3 vec3::cross(const vec3& b) const {
	vec3 result;
	result.m_data[0] = m_data[1] * b.m_data[2] - m_data[2] * b.m_data[1];
	result.m_data[1] = m_data[2] * b.m_data[0] - m_data[0] * b.m_data[2];
	result.m_data[2] = m_data[0] * b.m_data[1] - m_data[1] * b.m_data[0];

	return result;
}

inline idScalar vec3::dot(const vec3& b) const {
	return m_data[0] * b.m_data[0] + m_data[1] * b.m_data[1] + m_data[2] * b.m_data[2];
}

inline const mat33& mat33::operator=(const mat33& rhs) {
	if (&rhs != this) {
		memcpy(m_data, rhs.m_data, 9 * sizeof(idScalar));
	}
	return *this;
}
inline mat33 mat33::transpose() const {
	mat33 result;
	result.m_data[0] = m_data[0];
	result.m_data[1] = m_data[3];
	result.m_data[2] = m_data[6];
	result.m_data[3] = m_data[1];
	result.m_data[4] = m_data[4];
	result.m_data[5] = m_data[7];
	result.m_data[6] = m_data[2];
	result.m_data[7] = m_data[5];
	result.m_data[8] = m_data[8];

	return result;
}

inline mat33 operator*(const mat33& a, const mat33& b) {
	mat33 result;
	result.m_data[0] =
		a.m_data[0] * b.m_data[0] + a.m_data[1] * b.m_data[3] + a.m_data[2] * b.m_data[6];
	result.m_data[1] =
		a.m_data[0] * b.m_data[1] + a.m_data[1] * b.m_data[4] + a.m_data[2] * b.m_data[7];
	result.m_data[2] =
		a.m_data[0] * b.m_data[2] + a.m_data[1] * b.m_data[5] + a.m_data[2] * b.m_data[8];
	result.m_data[3] =
		a.m_data[3] * b.m_data[0] + a.m_data[4] * b.m_data[3] + a.m_data[5] * b.m_data[6];
	result.m_data[4] =
		a.m_data[3] * b.m_data[1] + a.m_data[4] * b.m_data[4] + a.m_data[5] * b.m_data[7];
	result.m_data[5] =
		a.m_data[3] * b.m_data[2] + a.m_data[4] * b.m_data[5] + a.m_data[5] * b.m_data[8];
	result.m_data[6] =
		a.m_data[6] * b.m_data[0] + a.m_data[7] * b.m_data[3] + a.m_data[8] * b.m_data[6];
	result.m_data[7] =
		a.m_data[6] * b.m_data[1] + a.m_data[7] * b.m_data[4] + a.m_data[8] * b.m_data[7];
	result.m_data[8] =
		a.m_data[6] * b.m_data[2] + a.m_data[7] * b.m_data[5] + a.m_data[8] * b.m_data[8];

	return result;
}

inline const mat33& mat33::operator+=(const mat33& b) {
	for (int i = 0; i < 9; i++) {
		m_data[i] += b.m_data[i];
	}

	return *this;
}

inline const mat33& mat33::operator-=(const mat33& b) {
	for (int i = 0; i < 9; i++) {
		m_data[i] -= b.m_data[i];
	}
	return *this;
}

inline vec3 operator*(const mat33& a, const vec3& b) {
	vec3 result;

	result.m_data[0] =
		a.m_data[0] * b.m_data[0] + a.m_data[1] * b.m_data[1] + a.m_data[2] * b.m_data[2];
	result.m_data[1] =
		a.m_data[3] * b.m_data[0] + a.m_data[4] * b.m_data[1] + a.m_data[5] * b.m_data[2];
	result.m_data[2] =
		a.m_data[6] * b.m_data[0] + a.m_data[7] * b.m_data[1] + a.m_data[8] * b.m_data[2];

	return result;
}

inline const vec3& vec3::operator+=(const vec3& b) {
	for (int i = 0; i < 3; i++) {
		m_data[i] += b.m_data[i];
	}
	return *this;
}

inline const vec3& vec3::operator-=(const vec3& b) {
	for (int i = 0; i < 3; i++) {
		m_data[i] -= b.m_data[i];
	}
	return *this;
}

inline mat33 operator*(const mat33& a, const idScalar& s) {
	mat33 result;
	for (int i = 0; i < 9; i++) {
		result.m_data[i] = a.m_data[i] * s;
	}
	return result;
}

inline mat33 operator*(const idScalar& s, const mat33& a) { return a * s; }

inline vec3 operator*(const vec3& a, const idScalar& s) {
	vec3 result;
	for (int i = 0; i < 3; i++) {
		result.m_data[i] = a.m_data[i] * s;
	}
	return result;
}
inline vec3 operator*(const idScalar& s, const vec3& a) { return a * s; }

inline mat33 operator+(const mat33& a, const mat33& b) {
	mat33 result;
	for (int i = 0; i < 9; i++) {
		result.m_data[i] = a.m_data[i] + b.m_data[i];
	}
	return result;
}
inline vec3 operator+(const vec3& a, const vec3& b) {
	vec3 result;
	for (int i = 0; i < 3; i++) {
		result.m_data[i] = a.m_data[i] + b.m_data[i];
	}
	return result;
}

inline mat33 operator-(const mat33& a, const mat33& b) {
	mat33 result;
	for (int i = 0; i < 9; i++) {
		result.m_data[i] = a.m_data[i] - b.m_data[i];
	}
	return result;
}
inline vec3 operator-(const vec3& a, const vec3& b) {
	vec3 result;
	for (int i = 0; i < 3; i++) {
		result.m_data[i] = a.m_data[i] - b.m_data[i];
	}
	return result;
}

inline mat33 operator/(const mat33& a, const idScalar& s) {
	mat33 result;
	for (int i = 0; i < 9; i++) {
		result.m_data[i] = a.m_data[i] / s;
	}
	return result;
}

inline vec3 operator/(const vec3& a, const idScalar& s) {
	vec3 result;
	for (int i = 0; i < 3; i++) {
		result.m_data[i] = a.m_data[i] / s;
	}
	return result;
}

inline const vecx& vecx::operator=(const vecx& rhs) {
	if (size() != rhs.size()) {
		error_message("size missmatch, size()= %d but rhs.size()= %d\n", size(), rhs.size());
		abort();
	}
	if (&rhs != this) {
		memcpy(m_data, rhs.m_data, rhs.size() * sizeof(idScalar));
	}
	return *this;
}
inline vecx operator*(const vecx& a, const idScalar& s) {
	vecx result(a.size());
	for (int i = 0; i < result.size(); i++) {
		result.m_data[i] = a.m_data[i] * s;
	}
	return result;
}
inline vecx operator*(const idScalar& s, const vecx& a) { return a * s; }
inline vecx operator+(const vecx& a, const vecx& b) {
	vecx result(a.size());
	// TODO: error handling for a.size() != b.size()??
	if (a.size() != b.size()) {
		error_message("size missmatch. a.size()= %d, b.size()= %d\n", a.size(), b.size());
		abort();
	}
	for (int i = 0; i < a.size(); i++) {
		result.m_data[i] = a.m_data[i] + b.m_data[i];
	}

	return result;
}
inline vecx operator-(const vecx& a, const vecx& b) {
	vecx result(a.size());
	// TODO: error handling for a.size() != b.size()??
	if (a.size() != b.size()) {
		error_message("size missmatch. a.size()= %d, b.size()= %d\n", a.size(), b.size());
		abort();
	}
	for (int i = 0; i < a.size(); i++) {
		result.m_data[i] = a.m_data[i] - b.m_data[i];
	}
	return result;
}
inline vecx operator/(const vecx& a, const idScalar& s) {
	vecx result(a.size());
	for (int i = 0; i < result.size(); i++) {
		result.m_data[i] = a.m_data[i] / s;
	}

	return result;
}
}
#endif
