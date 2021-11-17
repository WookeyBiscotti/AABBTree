#pragma once

#include "vec.hpp"

namespace biss {

template<class UserAABBType, class Type>
Type get_lb(uint i, const UserAABBType& aabb);

template<class UserAABBType, class Type>
Type get_ub(uint i, const UserAABBType& aabb);

template<uint N, class Type>
struct AABB {
	using Vec_t = biss::Vec<N, Type>;

	Vec_t lb; // lowerBound
	Vec_t ub; // upperBound

	AABB(const Vec_t& l, const Vec_t& u): lb(l), ub(u) {}
	AABB() = default;

	auto& unite(const AABB& other) {
		for (uint i = 0; i != N; ++i) {
			if (lb.point[i] > other.lb.point[i]) {
				lb.point[i] = other.lb.point[i];
			}
			if (ub.point[i] < other.ub.point[i]) {
				ub.point[i] = other.ub.point[i];
			}
		}

		return *this;
	}

	bool isIntersecting(const AABB& other) const {
		const auto d1 = other.lb - ub;
		for (uint i = 0; i != N; ++i) {
			if (d1.point[i] > 0) {
				return false;
			}
		}

		const auto d2 = lb - other.ub;
		for (uint i = 0; i != N; ++i) {
			if (d2.point[i] > 0) {
				return false;
			}
		}

		return true;
	}

	bool contains(const AABB& other) const {
		for (uint i = 0; i != N; ++i) {
			if (lb.point[i] > other.lb.point[i] || ub.point[i] < other.ub.point[i]) {
				return false;
			}
		}
		return true;
	}

	// for 2d perimeter
	// for 3d area
	Type area() const {
		const auto d = ub - lb;
		if constexpr (N == 1) {
			return d.point[0];
		}
		// with -O2 compilers generate same code
		//		if constexpr (N == 2) {
		//			return 2 * (d.point[0] + d.point[1]);
		//		}
		//		if constexpr (N == 3) {
		//			return 2 * (d.point[0] * d.point[1] + d.point[1] * d.point[2] + d.point[2] * d.point[0]);
		//		}

		Type a = 0;
		for (uint i = 0; i != N; ++i) {
			Type subA = 1;
			for (uint j = 0; j != N; ++j) {
				if (i == j) {
					continue;
				}
				subA *= d.point[j];
			}
			a += subA;
		}

		return 2 * a;
	}

	template<class UserAABBType>
	auto& set(const UserAABBType& other) {
		for (uint i = 0; i != N; ++i) {
			lb.point[i] = get_lb<UserAABBType, Type>(i, other);
			ub.point[i] = get_ub<UserAABBType, Type>(i, other);
		}

		return *this;
	}
};

template<class Type, uint N>
AABB<N, Type> unite(const AABB<N, Type>& aabb1, const AABB<N, Type>& aabb2) {
	return AABB<N, Type>{aabb1}.unite(aabb2);
}

} // namespace biss