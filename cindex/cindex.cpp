#include "cindex.hpp"

// Constructor: Initializes the cyclic index with a given threshold.
// If the provided threshold is less than 2, it defaults to 2.
cindex::cindex(unsigned int x = 2) : _top{x < 2 ? 2 : x}, _idx{0} {}

// Helper function to wrap the index within the range [0, _top)
void cindex::wrap() {
    if(_idx >= _top) _idx %= _top;
}

// Implicit conversion to unsigned int: Returns the current index value.
cindex::operator unsigned int() const {
    return _idx;
}

// Pre-increment: Increments the index and wraps it around if necessary.
cindex& cindex::operator++() {
    ++_idx;
    wrap();
    return *this;
}

// Post-increment: Increments the index, wraps it around if necessary, 
// and returns the previous value.
unsigned int cindex::operator++(int) {
    unsigned int prev = _idx;
    ++(*this);
    return prev;
}

// Pre-decrement: Decrements the index and wraps it around if necessary.
cindex& cindex::operator--() {
    if (_idx == 0) {
        _idx = _top - 1;
    } else {
        --_idx;
    }
    return *this;
}

// Post-decrement: Decrements the index, wraps it around if necessary, 
// and returns the previous value.
unsigned int cindex::operator--(int) {
    unsigned int prev = _idx;
    --(*this);
    return prev;
}

// Overloaded += operator: Adds a value to the index and wraps it around if necessary.
cindex& cindex::operator+=(int x) {
    _idx += x;
    wrap();
    return *this;
}

// Overloaded -= operator: Subtracts a value from the index and wraps it around if necessary.
cindex& cindex::operator-=(int x) {
    if (x > 0 && _idx < static_cast<unsigned int>(x)) {
        _idx = _top - (x - _idx);
    } else {
        _idx -= x;
    }
    return *this;
}

// Overloaded = operator: Sets the index value and wraps it around if necessary.
cindex& cindex::operator=(int x) {
    if (x < 0) {
        // Convert the negative value to its positive equivalent
        // and calculate the wrap-around position
        _idx = _top - (static_cast<unsigned int>(-x) % _top);
    } else {
        _idx = static_cast<unsigned int>(x);
    }
    wrap();
    return *this;
}

// Set a new top value and ensure _idx stays within the new bounds
void cindex::set_top(unsigned int new_top) {
    _top = (new_top < 2) ? 2 : new_top; // Ensure the new top is at least 2
    wrap(); // Adjust _idx to stay within the new bounds
}