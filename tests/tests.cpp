#include <aabb_tree.hpp>
#include <catch2/catch.hpp>
#include <indexer.hpp>
#include <vector>

using namespace Catch::literals;
using namespace biss;

struct OpsCount {
	int construct = 0;
	int destruct = 0;
	int move_construct = 0;
	int copy_construct = 0;
};

class BarCopy {
  public:
	BarCopy(OpsCount& o): o(o) { ++o.construct; }
	BarCopy(const BarCopy& other): o(other.o) { ++o.copy_construct; }
	BarCopy(BarCopy&& other) = delete;
	~BarCopy() { ++o.destruct; }

  private:
	OpsCount& o;
};

class BarMove {
  public:
	explicit BarMove(OpsCount& o): o(o) { ++o.construct; }
	BarMove(const BarMove& other) = delete;
	BarMove(BarMove&& other) noexcept: o(other.o) { ++o.move_construct; }
	~BarMove() { ++o.destruct; }

  private:
	OpsCount& o;
};

TEST_CASE("Index", "[Index]") {
	SECTION("Emplace simple value into index") {
		Indexer<int> index;
		auto idx = index.emplace(11);
		REQUIRE(idx != nullindex);
		REQUIRE(index[idx] == 11);
		REQUIRE(index.count() == 1);
	}
	SECTION("Emplace into index") {
		Indexer<BarCopy> index;
		OpsCount ops;
		index.emplace(ops);
		REQUIRE(ops.construct == 1);
		REQUIRE(index.count() == 1);
	}
	SECTION("Use copy constructor if move constructor is not available when realloc") {
		Indexer<BarCopy> index(1);
		OpsCount ops;
		index.emplace(ops);
		index.emplace(ops); //
		REQUIRE(ops.construct == 2);
		REQUIRE(ops.copy_construct == 1);
		REQUIRE(index.count() == 2);
	}
	SECTION("Use move constructor if available when realloc") {
		Indexer<BarMove> index(1);
		OpsCount ops;
		index.emplace(ops);
		index.emplace(ops);

		REQUIRE(ops.construct == 2);
		REQUIRE(ops.move_construct == 1);
		REQUIRE(ops.destruct == 1);
		REQUIRE(index.count() == 2);
	}
	SECTION("Create val in index") {
		Indexer<int> index;
		auto idx = index.create();
		REQUIRE(idx != nullindex);
		index[idx] = 777;
		REQUIRE(index[idx] == 777);
		REQUIRE(index.count() == 1);
	}
	SECTION("Remove val from index") {
		Indexer<BarMove> index(1);
		OpsCount ops;
		auto id1 = index.emplace(ops);
		auto id2 = index.emplace(ops);

		REQUIRE(ops.construct == 2);
		REQUIRE(ops.move_construct == 1);
		REQUIRE(ops.destruct == 1);
		REQUIRE(index.count() == 2);

		index.remove(id1);
		REQUIRE(ops.destruct == 2);
		REQUIRE(index.count() == 1);

		index.remove(id2);
		REQUIRE(ops.destruct == 3);
		REQUIRE(index.count() == 0);
	}
	SECTION("Reuse data in index") {
		Indexer<int> index(1);

		for (int i = 0; i != 10; ++i) {
			index.emplace(i);
		}

		for (int i = index.count(); i != index.capacity(); ++i) {
			index.emplace(i);
		}

		REQUIRE(index.count() == index.capacity());

		auto beg = index.begin();
		REQUIRE(beg != index.end());
		index.remove(beg);

		REQUIRE(index.count() == index.capacity() - 1);
		const auto saveCapacity = index.capacity();
		index.emplace(777);

		REQUIRE(saveCapacity == index.capacity());
	}
	SECTION("Capacity grow") {
		Indexer<int> index(1);
		index.emplace(1);
		const auto saveCapacity = index.capacity();
		index.emplace(1);
		REQUIRE(saveCapacity < index.capacity());
	}
	SECTION("Remove with index") {
		Indexer<int> index(1);
		auto id = index.emplace(1);
		REQUIRE(index.count() == 1);
		index.remove(id);
		REQUIRE(index.count() == 0);
	}
	SECTION("Many elements") {
		Indexer<int> index(0);
		for (int i = 0; i != 1000; ++i) {
			auto idx = index.emplace(i);
			REQUIRE(index[idx] == i);
		}
		for (int i = 0; i != 1000; ++i) {
			REQUIRE(index[i] == i);
		}
	}
	SECTION("Reuse data in index") {
		Indexer<int> index(1);

		for (int i = 0; i != 10; ++i) {
			index.emplace(i);
		}

		for (int i = index.count(); i != index.capacity(); ++i) {
			index.emplace(i);
		}

		REQUIRE(index.count() == index.capacity());

		auto beg = index.begin();
		REQUIRE(beg != index.end());
		index.remove(beg);

		REQUIRE(index.count() == index.capacity() - 1);
		const auto saveCapacity = index.capacity();
		index.emplace(777);

		REQUIRE(saveCapacity == index.capacity());
	}
}

TEST_CASE("AABBTree", "[AABBTree]") {
	SECTION("Emplace simple value into tree") {
		AABBTree<int, 2, float> tree;
		tree.emplace(AABB<float, 2>{Vec<float, 2>{0.0f}, Vec<float, 2>{1.0f}}, 1);
		tree.emplace(AABB<float, 2>{Vec<float, 2>{1.0f}, Vec<float, 2>{2.0f}}, 2);
		tree.emplace(AABB<float, 2>{Vec<float, 2>{2.0f}, Vec<float, 2>{3.0f}}, 3);

		tree.query(AABB<float, 2>{Vec<float, 2>{0}, Vec<float, 2>{0.9f}}, [&tree](auto idx) {
			REQUIRE(tree[idx] == 1);
			return false;
		});
		tree.query(AABB<float, 2>{Vec<float, 2>{1.1f}, Vec<float, 2>{1.2f}}, [&tree](auto idx) {
			REQUIRE(tree[idx] == 2);
			return false;
		});
		tree.query(AABB<float, 2>{Vec<float, 2>{2.1f}, Vec<float, 2>{2.2f}}, [&tree](auto idx) {
			REQUIRE(tree[idx] == 3);
			return false;
		});
	}
	SECTION("Make query") {
		AABBTree<AABB<float, 2>, 2, float> tree(0);
		int count = 0;
		AABB<float, 2> tester{Vec<float, 2>{400}, Vec<float, 2>{500}};
		for (int i = 0; i != 1000; ++i) {
			Vec<float, 2> lb;
			lb.point[0] = rand() % 1000;
			lb.point[1] = rand() % 1000;
			Vec<float, 2> ub;
			ub.point[0] = lb.point[0] + (rand() % 1000);
			ub.point[1] = lb.point[1] + (rand() % 1000);

			tree.emplace(AABB<float, 2>{lb, ub}, AABB<float, 2>{lb, ub});

			count += tester.isIntersecting(AABB<float, 2>{lb, ub});
		}

		INFO(count);

		tree.query(tester, [&count, &tree, &tester](auto idx) {
			count -= tree[idx].isIntersecting(tester);
			return true;
		});

		REQUIRE(count == 0);
	}
	SECTION("Erase from tree") {
		AABBTree<AABB<float, 2>, 2, float> tree(0);
		std::vector<decltype(tree)::index_t> idxs;
		AABB<float, 2> tester{Vec<float, 2>{400}, Vec<float, 2>{500}};
		for (int i = 0; i != 1000; ++i) {
			Vec<float, 2> lb;
			lb.point[0] = rand() % 1000;
			lb.point[1] = rand() % 1000;
			Vec<float, 2> ub;
			ub.point[0] = lb.point[0] + (rand() % 1000);
			ub.point[1] = lb.point[1] + (rand() % 1000);

			const auto aabb = AABB<float, 2>{lb, ub};
			idxs.push_back(tree.emplace(aabb, aabb));
		}

		for (auto idx : idxs) {
			tree.remove(idx);
		}

		REQUIRE(tree.count() == 0);
	}
	SECTION("Move tree elements") {
		AABBTree<AABB<float, 2>, 2, float> tree(0);
		std::vector<decltype(tree)::index_t> idxs;
		AABB<float, 2> tester{Vec<float, 2>{400}, Vec<float, 2>{500}};
		for (int i = 0; i != 1000; ++i) {
			Vec<float, 2> lb;
			lb.point[0] = rand() % 1000;
			lb.point[1] = rand() % 1000;
			Vec<float, 2> ub;
			ub.point[0] = lb.point[0] + (rand() % 1000);
			ub.point[1] = lb.point[1] + (rand() % 1000);

			const auto aabb = AABB<float, 2>{lb, ub};
			idxs.push_back(tree.emplace(aabb, aabb));
		}

		int count = 0;
		for (auto idx : idxs) {
			Vec<float, 2> lb;
			lb.point[0] = rand() % 1000;
			lb.point[1] = rand() % 1000;
			Vec<float, 2> ub;
			ub.point[0] = lb.point[0] + (rand() % 1000);
			ub.point[1] = lb.point[1] + (rand() % 1000);

			const auto aabb = AABB<float, 2>{lb, ub};
			tree.update(idx, aabb);
			tree[idx] = aabb;

			count += tester.isIntersecting(aabb);
		}

		INFO(count);
		tree.query(tester, [&count, &tree, &tester](auto idx) {
			count -= tree[idx].isIntersecting(tester);
			return true;
		});

		REQUIRE(count == 0);
	}
}