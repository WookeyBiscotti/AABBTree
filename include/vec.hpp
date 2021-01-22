#pragma once

#include "typedefs.hpp"

namespace biss {

template<class UserVecType, class Type>
Type get(uint i, const UserVecType& v);

template<class Type, uint N>
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
template<class Type, uint N>
Vec<Type, N> operator-(const Vec<Type, N>& a, const Vec<Type, N>& b) {
	return Vec<Type, N>{a} -= b;
}
template<class Type, uint N>
Vec<Type, N> operator+(const Vec<Type, N>& a, const Vec<Type, N>& b) {
	return Vec<Type, N>{a} += b;
}
template<class Type, uint N>
Vec<Type, N> operator*(const Vec<Type, N>& a, Type b) {
	return Vec<Type, N>{a} *= b;
}
template<class Type, uint N>
Vec<Type, N> operator*(Type b, const Vec<Type, N>& a) {
	return Vec<Type, N>{a} *= b;
}

} // namespace biss