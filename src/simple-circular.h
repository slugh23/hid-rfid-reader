#ifndef __SIMPLE_CIRCULAR_H__
#define __SIMPLE_CIRCULAR_H__

#include <stddef.h>
#include <functional>

template< typename T, unsigned int SIZE >
class SimpleCircular {
public:
  SimpleCircular() : mask((2^SIZE)-1), head(0), tail(0) {}
  bool empty() {return head == tail;}
  void push(const T& val) {
    elem[head] = val;
    inc(head);
    if(empty()) {
      inc(tail);
    }
  }
  void pop() {
    if(!empty()) {
      inc(tail);
    }
  }
  T last() { return elem[tail]; }
  void foreach( std::function<void (T)> func ) {
    while( !empty() ) {
      func(last());
      inc(tail);
    }
  }
protected:
  void inc(unsigned int& ptr) {
    ptr = (ptr + 1) & mask;
  }
protected:
  unsigned int mask;
  unsigned int head;
  unsigned int tail;
  T elem[2^SIZE];
};

#endif
