
#ifndef __TAG_PARSER__
#define __TAG_PARSER__

#include "tag.h"

class TagParser
{
  enum CONSTANTS {
    SHORT_MIN = 50,
    SHORT_MAX = 70,
    LONG_MAX = 90,
    SHORT_COUNT = 6,
    LONG_COUNT = 5,
    PREAMBLE_COUNT = 15
  };

public:
  TagParser(Tag& tag) : save_(tag) {}

  void begin(uint8_t gate, uint8_t rfid, void (*rfid_fcn)(void)) {
    // Initialize interrupt;
    pinMode(rfid, INPUT);
    pinMode(gate, OUTPUT);

    digitalWrite(gate, HIGH);
    //t.begin(timerGate, 80000);
    attachInterrupt( digitalPinToInterrupt(rfid), rfid_fcn, FALLING );
  }

  void change( unsigned int now )
  {
    unsigned int delta = now - lastChange_;
    lastChange_ = now;

    if( SHORT_MIN < delta && delta < LONG_MAX ) {
      if( delta < SHORT_MAX ) {
        //  Short - 0
        if( ++count0_ == SHORT_COUNT ) {
          tag_.push(0);
          count0_ = 0;
        }
        count1_ = 0;
      }
      else {
        //  Long - 1
        if( (++count1_ % LONG_COUNT) == 0 ) {
          tag_.push(1);
        }
        count0_ = 0;
      }
    }
    else {
      count0_ = 0;
      count1_ = 0;
      tag_.clear();
    }

    if(count1_ == PREAMBLE_COUNT) {
      tag_.clear();
    }

    if(tag_.is_valid()) {
      save_ = tag_;
    }
  }

protected:
  unsigned int lastChange_;
  int count0_;
  int count1_;
  Tag tag_;
  Tag& save_;
};

#endif
