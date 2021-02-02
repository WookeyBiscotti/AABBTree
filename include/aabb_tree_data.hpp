#pragma once

#include "typedefs.hpp"

#include <type_traits>
#include <utility>

namespace biss {

template<class ValueType>
struct AABBTreeData {
	template<class... Args>
	explicit AABBTreeData(index_t leafIdx, Args&&... args): leafIdx(leafIdx), data(std::forward<Args>(args)...) {}

	template<std::enable_if_t<std::is_nothrow_move_constructible_v<ValueType>, bool> = true>
	explicit AABBTreeData(AABBTreeData&& other) noexcept: leafIdx(other.leafIdx), data(std::move(other.data)) {}

	template<std::enable_if_t<std::is_copy_constructible_v<ValueType>, bool> = true>
	explicit AABBTreeData(const AABBTreeData& other): leafIdx(other.leafIdx), data(other.data) {}

	index_t leafIdx;
	ValueType data;
};

} // namespace biss