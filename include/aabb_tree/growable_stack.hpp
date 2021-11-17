#pragma once

#include "typedefs.hpp"

#include <cstdlib>
#include <cstring>

namespace biss {

template<typename T, uint N>
class GrowableStack {
  public:
	GrowableStack() {
		_stack = _array;
		_count = 0;
		_capacity = N;
	}

	~GrowableStack() {
		if (_stack != _array) {
			free(_stack);
		}
	}

	void push(const T& element) {
		if (_count == _capacity) {
			T* old = _stack;
			_capacity *= 2;
			_stack = (T*)malloc(_capacity * sizeof(T));
			memcpy(_stack, old, _count * sizeof(T));
			if (old != _array) {
				free(old);
			}
		}

		_stack[_count] = element;
		++_count;
	}

	T pop() {
		--_count;
		return _stack[_count];
	}

	uint count() const { return _count; }

	bool isHeap() const { return _stack != _array; }

  private:
	T* _stack;
	T _array[N];
	uint _count;
	uint _capacity;
};

} // namespace biss