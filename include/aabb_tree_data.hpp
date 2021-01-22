#pragma once

#include "typedefs.hpp"

#include <utility>

namespace biss {

template<class ValueType>
struct AABBTreeData {
	template<class... Args>
	AABBTreeData(index_t leafIdx, Args&&... args): leafIdx(leafIdx), data(std::forward<Args>(args)...) {}
	index_t leafIdx;
	ValueType data;
};

} // namespace biss