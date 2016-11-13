#ifndef __SIMPLE_CIRCULAR_H__
#define __SIMPLE_CIRCULAR_H__

#include <stddef.h>
#include <functional>

template< typename T, byte BUFF_POWER >
class SimpleCircular {
public:
  SimpleCircular() : size(1 << BUFF_POWER), mask(size-1), head(0), tail(0) {}
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
  /*
  void debug() {
    Serial.print("Size: ");
    Serial.println(size);
    Serial.print("Mask: ");
    Serial.println(mask);
    Serial.print("Mead: ");
    Serial.println(head);
    Serial.print("Tail: ");
    Serial.println(tail);
  }
  */
protected:
  void inc(unsigned int& ptr) {
    ptr = (ptr + 1) & mask;
  }
protected:
  const unsigned int size;
  const unsigned int mask;
  unsigned int head;
  unsigned int tail;
  T elem[1 << BUFF_POWER];
};

#endif
