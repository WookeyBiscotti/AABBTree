#pragma once

#include "aabb_tree_data.hpp"
#include "indexer.hpp"

namespace biss {

template<class ValueType>
class AABBTreeIterator {
  public:
	AABBTreeIterator(typename Indexer<AABBTreeData<ValueType>>::Iterator it): _it(it) {}

	ValueType& operator*() const { return _it.operator->().data; }
	ValueType& operator->() const { return _it->data; }

	const auto operator++(int) { return _it++; }
	const auto operator++() { return ++_it; }

	bool operator==(const AABBTreeIterator& other) const { return _it == other._it; }
	bool operator!=(const AABBTreeIterator& other) const { return _it != other._it; }

	index_t idx() const { return _it.operator->().leafIdx; }

  private:
	typename Indexer<AABBTreeData<ValueType>>::Iterator _it;
};

} // namespace biss