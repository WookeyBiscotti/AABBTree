#pragma once

#include "growable_stack.hpp"
#include "indexer.hpp"

namespace biss {

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

template<class Type, uint N>
struct AABB {
	using Vec_t = Vec<Type, N>;

	Vec_t lb; // lowerBound
	Vec_t ub; // upperBound

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
};

template<class Type, uint N>
AABB<Type, N> unite(const AABB<Type, N>& aabb1, const AABB<Type, N>& aabb2) {
	return AABB<Type, N>{aabb1}.unite(aabb2);
}

template<class ValueType, uint N, class KeyElementType>
class AABBTree {
  public:
	using AABB_t = AABB<KeyElementType, N>;
	using index_t = typename Indexer<void*>::index_t;

	explicit AABBTree(KeyElementType aabbExtension = 0, KeyElementType aabbMultiplier = 0) noexcept;

	template<class... Args>
	index_t emplace(const AABB_t& aabb, Args&&... args);

	void remove(index_t idx);

	template<typename T>
	void query(const AABB_t& aabb, const T& callback) const;

	void update(
	    index_t idx, const AABB_t& aabb, const typename AABB_t::Vec_t& displacement = typename AABB_t::Vec_t(0));

	ValueType& operator[](index_t idx);
	const ValueType& operator[](index_t idx) const;

	uint count() const;

  private:
	struct Node {
		AABB_t aabb;

		index_t parent;
		index_t child1;
		index_t child2;

		uint height;

		index_t dataIdx;

		bool isLeaf() const {
			assert(child1 != nullindex || child1 == child2);
			return child1 == nullindex;
		}
	};

  private:
	void insertLeaf(index_t leafIdx);
	void removeLeaf(index_t leafIdx);

	index_t balance(index_t iA);

	void validate() const;

  private:
	const KeyElementType _aabbExtension;
	const KeyElementType _aabbMultiplier;

	Indexer<Node> _nodes;
	Indexer<ValueType> _data;
	index_t _root;
};

template<class ValueType, uint N, class KeyElementType>
AABBTree<ValueType, N, KeyElementType>::AABBTree(KeyElementType aabbExtension, KeyElementType aabbMultiplier) noexcept:
    _root(nullindex), _nodes(0), _aabbExtension(aabbExtension), _aabbMultiplier(aabbMultiplier) {
}

template<class ValueType, uint N, class KeyElementType>
void AABBTree<ValueType, N, KeyElementType>::insertLeaf(AABBTree::index_t leafIdx) {
	if (_root == nullindex) {
		_root = leafIdx;
		_nodes[leafIdx].parent = nullindex;

		return;
	}

	index_t siblingIdx = _root;
	{
		Node* node = &_nodes[siblingIdx];
		while (!node->isLeaf()) {
			auto& child1 = _nodes[node->child1];
			auto& child2 = _nodes[node->child2];

			const auto unitedArea = unite(node->aabb, _nodes[leafIdx].aabb).area();
			const auto cost = 2 * unitedArea;

			const auto area = node->aabb.area();
			const auto inheritanceCost = 2 * (unitedArea - area);

			Node& leaf = _nodes[leafIdx];
			const auto calcCost = [&inheritanceCost, &leaf](const Node& child) {
				const auto area = unite(leaf.aabb, child.aabb).area();
				if (child.isLeaf()) {
					return area + inheritanceCost;
				} else {
					const auto oldArea = child.aabb.area();
					const auto newArea = area;

					return (newArea - oldArea) + inheritanceCost;
				}
			};
			const auto cost1 = calcCost(child1);
			const auto cost2 = calcCost(child2);

			if (cost < cost1 && cost < cost2) {
				break;
			}

			siblingIdx = cost1 < cost2 ? node->child1 : node->child2;
			node = &_nodes[siblingIdx];
		}
	}

	const auto oldParentIdx = _nodes[siblingIdx].parent;
	const auto newParentIdx = _nodes.create();
	Node& leaf = _nodes[leafIdx];

	Node& sibling = _nodes[siblingIdx];

	Node& newParent = _nodes[newParentIdx];
	newParent.parent = oldParentIdx;
	newParent.dataIdx = nullindex;
	newParent.aabb = unite(leaf.aabb, sibling.aabb);
	newParent.height = sibling.height + 1;

	if (oldParentIdx != nullindex) {
		Node& oldParent = _nodes[oldParentIdx];
		if (oldParent.child1 == siblingIdx) {
			oldParent.child1 = newParentIdx;
		} else {
			oldParent.child2 = newParentIdx;
		}

	} else {
		_root = newParentIdx;
	}

	newParent.child1 = siblingIdx;
	newParent.child2 = leafIdx;
	sibling.parent = newParentIdx;
	leaf.parent = newParentIdx;

	validate();

	index_t idx = leaf.parent;
	while (idx != nullindex) {
		validate();
		idx = balance(idx);
		validate();
		Node& current = _nodes[idx];

		Node& child1 = _nodes[current.child1];
		Node& child2 = _nodes[current.child2];

		current.height = 1 + (child1.height > child2.height ? child1.height : child2.height);
		current.aabb = unite(child1.aabb, child2.aabb);

		idx = current.parent;
		validate();
	}
}

template<class ValueType, uint N, class KeyElementType>
template<class... Args>
typename AABBTree<ValueType, N, KeyElementType>::index_t AABBTree<ValueType, N, KeyElementType>::emplace(
    const AABBTree::AABB_t& aabb, Args&&... args) {
	const auto dataIdx = _data.emplace(std::forward<Args>(args)...);
	const auto leafIdx = _nodes.create();

	Node& leaf = _nodes[leafIdx];
	if (_aabbExtension != KeyElementType{0}) {
		const Vec<KeyElementType, N> r(_aabbExtension);
		leaf.aabb = AABB_t{aabb.lb - r, aabb.ub + r};
	} else {
		leaf.aabb = aabb;
	};
	leaf.dataIdx = dataIdx;
	leaf.height = 0;
	leaf.child1 = leaf.child2 = leaf.parent = nullindex;

	insertLeaf(leafIdx);

	return leafIdx;
}

template<class ValueType, uint N, class KeyElementType>
typename AABBTree<ValueType, N, KeyElementType>::index_t AABBTree<ValueType, N, KeyElementType>::balance(
    AABBTree::index_t iA) {
	const auto max = [](uint a, uint b) { return a > b ? a : b; };

	Node& A = _nodes[iA];
	if (A.isLeaf() || A.height < 2) {
		return iA;
	}

	const auto iB = A.child1;
	const auto iC = A.child2;
	Node& B = _nodes[iB];
	Node& C = _nodes[iC];

	// rotate A C
	const auto rotate = [&_nodes = this->_nodes, &_root = this->_root, this](
	                        Node& A, index_t iA, Node& C, index_t iC, Node& B, index_t iB) -> index_t {
		const auto max = [](uint a, uint b) { return a > b ? a : b; };

		const auto iF = C.child1;
		const auto iG = C.child2;
		Node& F = _nodes[iF];
		Node& G = _nodes[iG];
		// Swap A and C
		C.child1 = iA;
		C.parent = A.parent;
		A.parent = iC;

		// A's old parent should point to C
		if (C.parent != nullindex) {
			auto& Cparent = _nodes[C.parent];
			if (Cparent.child1 == iA) {
				Cparent.child1 = iC;
			} else {
				assert(_nodes[C.parent].child2 == iA);
				Cparent.child2 = iC;
			}
		} else {
			_root = iC;
		}

		// Rotate
		if (F.height > G.height) {
			C.child2 = iF;
			A.child2 == iC ? A.child2 = iG : A.child1 = iG;
			G.parent = iA;
			A.aabb = unite(B.aabb, G.aabb);
			C.aabb = unite(A.aabb, F.aabb);

			A.height = 1 + max(B.height, G.height);
			C.height = 1 + max(A.height, F.height);
		} else {
			C.child2 = iG;
			A.child2 == iC ? A.child2 = iF : A.child1 = iF;
			F.parent = iA;
			A.aabb = unite(B.aabb, F.aabb);
			C.aabb = unite(A.aabb, G.aabb);

			A.height = 1 + max(B.height, F.height);
			C.height = 1 + max(A.height, G.height);
		}

		assert(A.child1 != nullindex && A.child2 != nullindex);
		assert(C.child1 != nullindex && C.child2 != nullindex);

		assert(C.parent == nullindex || (_nodes[C.parent].child1 && _nodes[C.parent].child2));

		//		validate();
		return iC;
	};

	// Rotate C up
	if (C.height > B.height + 1) {
		return rotate(A, iA, C, iC, B, iB);
	}

	if ((B.height > C.height + 1)) {
		return rotate(A, iA, B, iB, C, iC);
	}

	return iA;
}

template<class ValueType, uint N, class KeyElementType>
void AABBTree<ValueType, N, KeyElementType>::removeLeaf(AABBTree::index_t leafIdx) {
	if (leafIdx == _root) {
		_root = nullindex;

		return;
	}
	const auto parentIdx = _nodes[leafIdx].parent;
	auto& parent = _nodes[parentIdx];

	const auto siblingIdx = parent.child1 == leafIdx ? parent.child2 : parent.child1;
	auto& sibling = _nodes[siblingIdx];

	const auto grandParentIdx = parent.parent;
	if (grandParentIdx != nullindex) {
		auto& grandParent = _nodes[grandParentIdx];
		// Destroy parent and connect sibling to grandParent.
		if (grandParent.child1 == parentIdx) {
			grandParent.child1 = siblingIdx;
		} else {
			grandParent.child2 = siblingIdx;
		}
		sibling.parent = grandParentIdx;

		// Adjust ancestor bounds.
		auto currentIdx = grandParentIdx;
		while (currentIdx != nullindex) {
			currentIdx = balance(currentIdx);
			auto& current = _nodes[currentIdx];

			auto& child1 = _nodes[current.child1];
			auto& child2 = _nodes[current.child2];

			current.aabb = unite(child1.aabb, child2.aabb);
			current.height = 1 + (child1.height > child2.height ? child1.height : child2.height);

			currentIdx = current.parent;
		}
	} else {
		_root = siblingIdx;
		sibling.parent = nullindex;
	}

	_nodes.remove(parentIdx);
}

template<class ValueType, uint N, class KeyElementType>
void AABBTree<ValueType, N, KeyElementType>::remove(AABBTree::index_t idx) {
	assert(_nodes[idx].isLeaf());

	removeLeaf(idx);
	_data.remove(_nodes[idx].dataIdx);
	_nodes.remove(idx);

	validate();
}

template<class ValueType, uint N, class KeyElementType>
template<typename T>
void AABBTree<ValueType, N, KeyElementType>::query(const AABBTree::AABB_t& aabb, const T& callback) const {
	GrowableStack<index_t, 256> stack;
	stack.push(_root);

	while (stack.count() > 0) {
		index_t nodeIdx = stack.pop();
		if (nodeIdx == nullindex) {
			continue;
		}

		const Node& node = _nodes[nodeIdx];

		if (node.aabb.isIntersecting(aabb)) {
			if (node.isLeaf()) {
				if (!callback(nodeIdx)) {
					return;
				}
			} else {
				stack.push(node.child1);
				stack.push(node.child2);
			}
		}
	}
}

template<class ValueType, uint N, class KeyElementType>
ValueType& AABBTree<ValueType, N, KeyElementType>::operator[](AABBTree::index_t idx) {
	assert(_nodes[idx].isLeaf());

	return _data[_nodes[idx].dataIdx];
}

template<class ValueType, uint N, class KeyElementType>
const ValueType& AABBTree<ValueType, N, KeyElementType>::operator[](AABBTree::index_t idx) const {
	assert(_nodes[idx].isLeaf());

	return _data[_nodes[idx].dataIdx];
}

template<class ValueType, uint N, class KeyElementType>
void AABBTree<ValueType, N, KeyElementType>::validate() const {
	int count = 0;
	for (const auto& node : _nodes) {
		count += node.isLeaf();
	}
	assert(count == _data.count());

	for (index_t idx = 0; idx != _nodes.capacity(); ++idx) {
		if (!_nodes.contains(idx)) {
			continue;
		}

		assert(_nodes[idx].parent == nullindex || _nodes[_nodes[idx].parent].child1 == idx ||
		       _nodes[_nodes[idx].parent].child2 == idx);
	}
}

template<class ValueType, uint N, class KeyElementType>
uint AABBTree<ValueType, N, KeyElementType>::count() const {
	return _data.count();
}

template<class ValueType, uint N, class KeyElementType>
void AABBTree<ValueType, N, KeyElementType>::update(
    AABBTree::index_t idx, const AABBTree::AABB_t& aabb, const typename AABB_t::Vec_t& displacement) {
	AABB_t extAABB;
	const typename AABB_t::Vec_t r(_aabbExtension);
	if (_aabbExtension != KeyElementType{0}) {
		extAABB.lb = aabb.lb - r;
		extAABB.ub = aabb.ub + r;
	} else {
		extAABB = aabb;
	}

	if (_aabbMultiplier != KeyElementType{0}) {
		// Predict AABB movement
		const typename AABB_t::Vec_t d = _aabbMultiplier * displacement;

		for (uint i = 0; i != N; ++i) {
			if (d.point[i] < KeyElementType{0}) {
				extAABB.lb.point[i] += d.point[i];
			} else {
				extAABB.ub.point[i] += d.point[i];
			}
		}
	}

	const auto& treeAABB = _nodes[idx].aabb;
	if (treeAABB.contains(aabb)) {
		// The tree AABB still contains the object, but it might be too large.
		// Perhaps the object was moving fast but has since gone to sleep.
		// The huge AABB is larger than the new fat AABB.
		AABB_t hugeAABB;
		hugeAABB.lb = extAABB.lb - 4.0f * r;
		hugeAABB.ub = extAABB.ub + 4.0f * r;

		if (hugeAABB.contains(treeAABB)) {
			// The tree AABB contains the object AABB and the tree AABB is
			// not too large. No tree update needed.
			return;
		}

		// Otherwise the tree AABB is huge and needs to be shrunk
	}

	removeLeaf(idx);
	_nodes[idx].aabb = extAABB;
	insertLeaf(idx);
}

} // namespace biss