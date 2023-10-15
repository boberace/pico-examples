#ifndef CINDEX_HPP
#define CINDEX_HPP

class cindex {
private:
    unsigned int _top;        // The maximum value for the cyclic index
    unsigned int _idx;        // The current index value

    // Helper function to wrap the index within the range [0, _top)
    void wrap();

public:
    // Constructor
    cindex(unsigned int x);

    // Implicit conversion to unsigned int
    operator unsigned int() const;

    // Pre-increment
    cindex& operator++();

    // Post-increment
    unsigned int operator++(int);

    // Pre-decrement
    cindex& operator--();

    // Post-decrement
    unsigned int operator--(int);

    // Overloaded += operator
    cindex& operator+=(int x);

    // Overloaded -= operator
    cindex& operator-=(int x);

    // Overloaded = operator
    cindex& operator=(int x);

    // Set a new top value and ensure _idx stays within the new bounds
    void set_top(unsigned int new_top);

};

#endif // CINDEX_HPP
