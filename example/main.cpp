#include <aabb_tree.hpp>

// Custom 2d vector
struct MyVec2 {
	float x;
	float y;
};

// Custom 2d AABB
struct MyAABB {
	MyVec2 lowerBound;
	MyVec2 upperBound;
};

// Specialise this function in biss namespace for using your custom vectors and AABB in AABBTree container
namespace biss {
template<>
float get(biss::uint i, const MyVec2& vec) {
	return i == 0 ? vec.x : vec.y;
}

template<>
float get_lb(uint i, const MyAABB& aabb) {
	return i == 0 ? aabb.lowerBound.x : aabb.lowerBound.y;
}

template<>
float get_ub(uint i, const MyAABB& aabb) {
	return i == 0 ? aabb.upperBound.x : aabb.upperBound.y;
}
} // namespace biss

int main() {
	using AABB = biss::AABB<2, float>;
	using Vec = biss::Vec<2, float>;
	using AABBTree = biss::AABBTree<int, 2, float>;

	AABBTree tree;

	// You can user custom AABB
	auto idx1 = tree.emplace(MyAABB{MyVec2{0, 0}, MyVec2{100, 100}}, 1);
	auto idx2 = tree.emplace(MyAABB{MyVec2{101, 101}, MyVec2{200, 200}}, 2);

	// Or default AABB
	Vec lb;
	lb.point[0] = 101.0f;
	lb.point[1] = 0.0f;
	Vec ub;
	ub.point[0] = 150.0f;
	ub.point[1] = 50.0f;

	AABB aabb{lb, ub};
	auto idx3 = tree.emplace(aabb, 3);

	// Now we can make query
	tree.query(MyAABB{{60, 60}, {150, 150}}, [&tree](const AABBTree::Iterator& it) -> bool {
		// we intersect only 2 first aabb
		assert(*it < 3);

		// continue query?
		return true;
	});

	tree.query(MyAABB{{101, 0}, {150, 50}}, [&tree](const AABBTree::Iterator& it) -> bool {
		assert(*it == 3);

		return true;
	});

	// update value
	tree[idx1] = 11;
	tree[idx2] = 12;
	tree[idx3] = 13;

	// update aabb position
	tree.update(idx1, MyAABB{{10, 10}, {11, 11}}, MyVec2{});
	tree.update(idx2, MyAABB{{10, 10}, {12, 12}}, MyVec2{});
	tree.update(idx3, MyAABB{{0, 0}, {1, 1}}, MyVec2{});

	tree.query(MyAABB{{0, 0}, {2, 2}}, [](const AABBTree::Iterator& it) -> bool {
		assert(*it == 13);

		return true;
	});

	tree.query(MyAABB{{10, 10}, {11, 11}}, [](const AABBTree::Iterator& it) -> bool {
		assert(*it < 13);

		return true;
	});

	// foreach loop
	for (auto& data : tree) {
        data += data;
	}
	
	return 0;
}