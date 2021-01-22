#pragma once

#include <cassert>
#include <cstdlib>
#include <utility>

namespace biss {

template<class Data>
class Indexer {
  private:
	struct Node {
		static constexpr auto NEXT_BITS_COUNT = (8 * sizeof(uint)) - 1;
		uint free : 1;
		uint next: NEXT_BITS_COUNT;
		Data data;
	};

  public:
	using index_t = uint;

	Indexer(uint initialCapacity = 0);
	~Indexer();

	Indexer(const Indexer& other) = delete;
	Indexer(Indexer&& other) noexcept;

	template<class... Args>
	index_t emplace(Args&&... args);

	index_t create();

	void remove(index_t idx);

	bool contains(index_t idx) const;

	const Data& operator[](index_t idx) const;
	Data& operator[](index_t idx);

	uint capacity() const;
	uint count() const;

	class Iterator {
	  public:
		auto& operator*() const;
		auto& operator->() const;

		const auto operator++(int);
		const auto operator++();

		bool operator==(const Iterator& other) const;
		bool operator!=(const Iterator& other) const;

		index_t idx() const { return _it - _begin; }

	  private:
		friend class Indexer;
		Iterator(Node* begin, Node* it, Node* end): _begin(begin), _it(it), _end(end) {}

	  private:
		Node* _begin;
		Node* _it;
		Node* _end;
	};

	Iterator begin() const;
	Iterator end() const;

	void remove(const Iterator& iterator);

  private:
	Node* _nodes;
	uint _capacity;
	uint _count;
	uint _freeNode;
};

template<class Data>
auto& Indexer<Data>::Iterator::operator*() const {
	assert(_it && !_it->free);

	return _it->data;
}

template<class Data>
auto& Indexer<Data>::Iterator::operator->() const {
	assert(_it && !_it->free);

	return _it->data;
}

template<class Data>
const auto Indexer<Data>::Iterator::operator++(int) {
	assert(_it);

	while (++_it != _end && _it->free) {}

	return *this;
}

template<class Data>
const auto Indexer<Data>::Iterator::operator++() {
	assert(_it);

	auto copy = Iterator(_begin, _it, _end);
	while (++_it != _end && _it->free) {}

	return copy;
}

template<class Data>
bool Indexer<Data>::Iterator::operator==(const Indexer::Iterator& other) const {
	return _it == other._it && _end == other._end;
}

template<class Data>
bool Indexer<Data>::Iterator::operator!=(const Indexer::Iterator& other) const {
	return !operator==(other);
}

template<class Data>
Indexer<Data>::Indexer(uint initialCapacity) {
	_capacity = initialCapacity;
	_count = 0;

	if (_capacity) {
		_nodes = static_cast<Node*>(std::malloc(_capacity * sizeof(Node)));
		if (_nodes) {

			for (uint i = 0; i != _capacity - 1; ++i) {
				Node& node = _nodes[i];
				node.next = i + 1;
				node.free = 1;
			}
			auto& node = _nodes[_capacity - 1];
			node.next = nullindex;
			node.free = 1;
			_freeNode = 0;
		} else {
			_capacity = 0;
			_freeNode = nullindex;
		}
	} else {
		_nodes = nullptr;
		_freeNode = nullindex;
	}
}

template<class Data>
Indexer<Data>::~Indexer() {
	for (uint i = 0; i != _capacity; ++i) {
		auto& node = _nodes[i];
		if (node.free) {
			continue;
		}
		node.data.~Data();
	}

	if (_capacity) {
		free(_nodes);
	}
}

template<class Data>
void Indexer<Data>::remove(Indexer::index_t idx) {
	assert(idx < _capacity && !_nodes[idx].free);

	auto& node = _nodes[idx];
	node.data.~Data();
	node.next = _freeNode;
	node.free = 1;
	_freeNode = idx;
	--_count;
}

template<class Data>
template<class... Args>
typename Indexer<Data>::index_t Indexer<Data>::emplace(Args&&... args) {
	if (_freeNode == nullindex) {
		const auto newCapacity = (_capacity ? _capacity : 8) * 2;
		auto newNodes = static_cast<Node*>(std::malloc(newCapacity * sizeof(Node)));

		if (!newNodes) {
			return nullindex;
		}

		for (uint i = 0; i != _capacity; ++i) {
			Node& node = _nodes[i];
			Node& newNode = newNodes[i];
			newNode.free = node.free;
			newNode.next = node.next;
			if constexpr (std::is_nothrow_move_constructible_v<Data>) {
				new (&newNode.data) Data(std::move(node.data));
			} else {
				new (&newNode.data) Data(node.data);
			}
			node.data.~Data();
		}
		_nodes = newNodes;

		for (uint i = _capacity; i != newCapacity - 1; ++i) {
			auto& node = _nodes[i];
			node.next = i + 1;
			node.free = 1;
		}
		auto& node = _nodes[newCapacity - 1];
		node.next = nullindex;
		node.free = 1;

		_freeNode = _capacity;
		_capacity = newCapacity;
	}

	auto& node = _nodes[_freeNode];
	index_t newIdx = _freeNode;
	_freeNode = node.next;
	node.free = 0;
	++_count;

	new (&node.data) Data(std::forward<Args>(args)...);

	return newIdx;
}

template<class Data>
typename Indexer<Data>::index_t Indexer<Data>::create() {
	return emplace();
}

template<class Data>
const Data& Indexer<Data>::operator[](Indexer::index_t idx) const {
	assert(contains(idx));

	return _nodes[idx].data;
}

template<class Data>
Data& Indexer<Data>::operator[](Indexer::index_t idx) {
	assert(contains(idx));

	return _nodes[idx].data;
}

template<class Data>
typename Indexer<Data>::Iterator Indexer<Data>::begin() const {
	for (uint i = 0; i != _capacity; ++i) {
		if (!_nodes[i].free) {
			return Indexer::Iterator(_nodes, _nodes + i, _nodes + _capacity);
		}
	}

	return end();
}

template<class Data>
typename Indexer<Data>::Iterator Indexer<Data>::end() const {
	const auto end = _nodes + _capacity;
	return Indexer::Iterator(_nodes, end, end);
}

template<class Data>
Indexer<Data>::Indexer(Indexer&& other) noexcept:
    _capacity(other._capacity), _freeNode(other._freeNode), _nodes(other._nodes), _count(other._count) {
	other._capacity = 0;
	other._nodes = nullptr;
	other._freeNode = nullindex;
	other._count = 0;
}

template<class Data>
uint Indexer<Data>::capacity() const {
	return _capacity;
}

template<class Data>
uint Indexer<Data>::count() const {
	return _count;
}

template<class Data>
void Indexer<Data>::remove(const Indexer::Iterator& iterator) {
	assert(iterator._it);

	const auto idx = iterator._it - _nodes;
	remove(idx);
}

template<class Data>
bool Indexer<Data>::contains(Indexer::index_t idx) const {
	return idx < _capacity && !_nodes[idx].free;
}
} // namespace biss