#pragma once

#include "typedefs.hpp"

namespace biss {

template<class UserVecType, class Type>
Type get(uint i, const UserVecType& v);

template<uint N, class Type>
struct Vec {
	Type point[N];

	Vec() = default;
	Vec(Type val) {
		for (uint i = 0; i != N; ++i) {
			point[i] = val;
		}
	}
	auto& operator+=(const Vec& other) {
		for (uint i = 0; i != N; ++i) {
			point[i] += other.point[i];
		}
		return *this;
	}
	auto& operator-=(const Vec& other) {
		for (uint i = 0; i != N; ++i) {
			point[i] -= other.point[i];
		}
		return *this;
	}
	auto& operator*=(Type m) {
		for (uint i = 0; i != N; ++i) {
			point[i] *= m;
		}
		return *this;
	}
	bool isZero() const {
		for (uint i = 0; i != N; ++i) {
			if (point[i] != Type{0}) {
				return false;
			}
		}
		return true;
	}

	template<class UserVectorType>
	auto& set(const UserVectorType& other) {
		for (uint i = 0; i != N; ++i) {
			point[i] = get<UserVectorType, Type>(i, other);
		}

		return *this;
	}
};
template<uint N, class Type>
Vec<N, Type> operator-(const Vec<N, Type> & a, const Vec<N, Type> & b) {
	return Vec<N, Type> {a} -= b;
}
template<uint N, class Type>
Vec<N, Type> operator+(const Vec<N, Type> & a, const Vec<N, Type> & b) {
	return Vec<N, Type> {a} += b;
}
template<uint N, class Type>
Vec<N, Type> operator*(const Vec<N, Type> & a, Type b) {
	return Vec<N, Type> {a} *= b;
}
template<uint N, class Type>
Vec<N, Type> operator*(Type b, const Vec<N, Type> & a) {
	return Vec<N, Type> {a} *= b;
}

} // namespace biss