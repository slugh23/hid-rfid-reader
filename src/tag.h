
#ifndef __TAG__
#define __TAG__

#include <Arduino.h>

class Tag
{
public:
  Tag() { clear(); }
  void clear() {
    count_ = 0;
    tag_ = 0;
  }
  bool is_valid( bool parity = false ) {
    // TODO: Add parity test
    if( count_ == 90 ) {
      if( !parity ) {
        return true;
      }
      int bits = 0;
      uint32_t pty = (uint32_t)tag_ & 0x3FFFFFF;
      int sum = pty & 0x1;
      while( ++bits < 13 ) {
        sum += (pty >>= 1) & 0x1;
      }
      if( (sum & 0x1) == 1 ) {
        //lightLEDFor_ms(10);
        while( ++bits < 27 ) {
          sum += (pty >>= 1) & 0x1;
        }
        return (sum & 0x01) == 1;
      }
    }
    return false;
  }
  bool push(bool bit) {
    if( is_valid() )
      return true;
    if( count_ & 0x01 )
    {
      if( bit == lastBit_ ) {
        return false;
      }
      tag_ = (tag_ << 1) + lastBit_;
    }
    else {
      lastBit_ = bit;
    }
    ++count_;
  }

  unsigned int value() {
    return (tag_ & 0x1FFFFFE) / 2;
  }

  String to_string() {
    //int op = tag_ & 0x1;
    String tag = "";
      //String(tag_ >> 37, HEX);
    return tag;
  }

protected:
  unsigned int count_;
  bool lastBit_;
  uint64_t tag_;
};

#endif
