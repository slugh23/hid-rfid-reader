#include <Arduino.h>
#include <EEPROM.h>
#include "simple-circular.h"

//build_flags = -D USB_SERIAL_HID

unsigned long lightLED = 0;

void lightLEDFor_ms( unsigned int ms ) {
  lightLED = millis() + ms;
}

class Tag
{
public:
  Tag() { clear(); }
  void clear() {
    count_ = 0;
    tag_ = 0;
  }
  bool is_valid() {
    // TODO: Add parity test
    return count_ == 90;
  }
  bool push(bool bit) {
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

protected:
  unsigned int count_;
  bool lastBit_;
  uint64_t tag_;
};

class TagParser
{
public:
  TagParser(Tag& tag) : save_(tag) {}

  void begin(uint8_t gate, uint8_t rfid, void (*rfid_fcn)(void)) {
    // Initialize interrupt;
    pinMode(rfid, INPUT);
    pinMode(gate, OUTPUT);

    digitalWrite(gate, HIGH);
    //t.begin(timerGate, 250000);
    attachInterrupt( digitalPinToInterrupt(rfid), rfid_fcn, CHANGE );
  }

  void change( unsigned int now )
  {
    unsigned int delta = now - lastChange_;
    lastChange_ = now;

    if( delta < 100 ) {
      ++count_;
    }
    else
    {
      if( count_ < 15 ) {
        // push 1
        tag_.push(1);
      }
      else if( count_ < 25 ) {
        // push 1 1
        tag_.push(1);
        tag_.push(1);
      }

      if( delta < 600 ) {
        // push 0
        tag_.push(0);
      }
      else if( delta < 1000 ) {
        // push 0 0
        tag_.push(0);
        tag_.push(0);
      }
      else if( delta < 1300 ) {
        //  clear tag
        save_ = tag_;
        tag_.clear();
      }
      else if( delta < 1750 ) {
        //  clear tag
        tag_.push(0);
        save_ = tag_;
        tag_.clear();
      }
      // ignore it...
      count_ = 0;
    }
  }

protected:
  unsigned int lastChange_;
  int count_;
  Tag tag_;
  Tag& save_;
};

class TagParser2
{
public:
  TagParser2(Tag& tag) : save_(tag) {}

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

    if( 50 < delta ) {
      if( delta < 70 ) {
        //  Short - 0
        if( ++count0_ == 6 ) {
          tag_.push(0);
          count0_ = 0;
        }
        count1_ = 0;
      }
      if( delta < 100 ) {
        //  Long - 1
        if( (++count1_ % 5) == 0 ) {
          tag_.push(1);
        }
        count0_ = 0;
      }
      else {
        count0_ = 0;
        count1_ = 0;
      }
    }
    else {
      count0_ = 0;
      count1_ = 0;
    }

    if(count1_ == 15) {
      tag_.clear();
      lightLEDFor_ms(10);
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

#define RFID_SERIAL Serial2

struct Passwd {
  char str[255];
  byte len;
};

const int addrPass = 0;
const int addrTag = sizeof(Passwd);

enum MODES {
  MODE_CMD,
  MODE_OLDPASS,
  MODE_NEWPASS1,
  MODE_NEWPASS2,
  MODE_CHECKPASS,
  MODE_LEARN
};

const byte SOT = 0x02;
const byte EOT = 0x03;

SimpleCircular<unsigned int, 12> times;

Tag tag1;
Tag tag2;
TagParser2 parser(tag1);

unsigned long lastChange;
unsigned long lastTag = 0;
int readLen = 0;

unsigned lastDelta = 0;
unsigned int lastBreak = 0;
int preamble = 15;

const int TAGLEN = 12;
byte tag[TAGLEN] = { 48, 66, 48, 48, 50, 54, 48, 53, 65, 49, 56, 57 };
byte tmpTag[TAGLEN];
bool tagMatch = true;

String buff;
String cmd;
String lastCmd;
int mode = MODE_CMD;

Passwd svdPass;
String newPass1;

const int ledPin = LED_BUILTIN;
const int rfidPin = 6;
const int gatePin = 5;

bool ctrlAltDel = false;

IntervalTimer t;

void pinChanged() {
  unsigned int now = micros();
  unsigned int delta = now - lastChange;
  times.push( delta * 2 + digitalRead(rfidPin) );

  lastChange = now;
  parser.change(now);
}

void timerGate(void) {
  digitalWrite(gatePin, !digitalRead(gatePin));
}

void sendCtrlAltDel() {
  // press and hold CTRL
  Keyboard.set_modifier(MODIFIERKEY_CTRL);
  Keyboard.send_now();

  // press ALT while still holding CTRL
  Keyboard.set_modifier(MODIFIERKEY_CTRL | MODIFIERKEY_ALT);
  Keyboard.send_now();

  // press DELETE, while CLTR and ALT still held
  Keyboard.set_key1((uint8_t)KEY_DELETE);
  Keyboard.send_now();

  // release all the keys at the same instant
  Keyboard.set_modifier(0);
  Keyboard.set_key1(0);
  Keyboard.send_now();
}

bool processSerial( char newChar )
{
  switch( newChar )
  {
    case '\r':
    case '\n':
    if( buff.length() > 0 ) {
      if( buff == "!" ) {
        if( lastCmd.length() > 0 ) {
          cmd = lastCmd;
        }
      }
      else {
        cmd = buff;
      }
      buff = "";
      return true;
    }
    return false;

    case '\x08':
    if( buff.length() > 0 ) {
      buff.remove(buff.length()-1);
    }
    return false;

    default:
    buff += newChar;
    if( mode == MODE_CMD ) {
      Serial.print( newChar );
    }
    else {
      Serial.print("*");
    }
    return false;
  }
}

void processCmd()
{
  cmd.toLowerCase();
  lastCmd = cmd;
  if( cmd == "passwd" ) {
    if( svdPass.len > 0 && svdPass.len < 255 ) {
      Serial.println("Enter old password:");
      mode = MODE_OLDPASS;
    }
    else {
      Serial.println("Enter new password:");
      mode = MODE_NEWPASS1;
    }
  }
  else if( cmd == "learn" ) {
    if( svdPass.len > 0 && svdPass.len < 255 ) {
      Serial.println("Enter password:");
      mode = MODE_CHECKPASS;
    }
    else {
      Serial.println("Swipe tag:");
      mode = MODE_LEARN;
    }
  }
  else if( cmd == "dump" || cmd == "." ) {
    times.foreach(
      [](unsigned int t) {
        Serial.print(t / 2);
        Serial.print(":");
        Serial.println(t & 0x01);
      }
    );
  }
  else if( cmd == "dump8" || cmd == ".." ) {
    times.foreach(
      [](unsigned int t) {
        Serial.print(t / 16);
        Serial.print(":");
        Serial.println(t & 0x01);
      }
    );
  }
  else if( cmd == "tag" ) {
    Serial.print("TAG: ");
    for( int i = 0; i < TAGLEN; ++i ) {
      Serial.print( tag[i], HEX );
    }
    Serial.println("");
  }
  else if( cmd == "reset" ) {
    svdPass.len = 0;
    EEPROM.put(addrPass, svdPass);
    Serial.println("Password erased!");
  }
  else if( cmd == "locked" ) {
    ctrlAltDel = true;
  }
  else if( cmd == "unlocked" ) {
    ctrlAltDel = false;
  }
  else {
    Serial.println("ERROR: Unknown command!");
  }
}

void setup() {
//  pinMode(rfidPin, INPUT);
  pinMode(ledPin, OUTPUT);
  parser.begin(gatePin, rfidPin, pinChanged);
//  pinMode(gatePin, OUTPUT);

//  digitalWrite(gatePin, HIGH);
  //t.begin(timerGate, 80000);

  Serial.begin( 115200 );
  RFID_SERIAL.begin( 9600 );

  lastChange = micros();
  //attachInterrupt( digitalPinToInterrupt(rfidPin), pinChanged, CHANGE );

  EEPROM.get(addrPass, svdPass);
  EEPROM.get(addrTag, tag);

  lightLED = millis() + 1000;
}

void loop() {
  // put your main code here, to run repeatedly:
  if( millis() > lightLED ) {
    digitalWrite(ledPin, LOW);
  }
  else {
    digitalWrite(ledPin, HIGH);
  }

  if( Serial.available() > 0 ) {
    if( processSerial( Serial.read() ) ) {
      Serial.println("");

      switch( mode )
      {
        case MODE_CMD:
        processCmd();
        break;

        case MODE_OLDPASS:
        if( cmd.length() == svdPass.len ) {
          bool ok = true;
          for( unsigned i = 0; i < cmd.length(); ++i ) {
            ok |= (cmd[i] == svdPass.str[i]);
          }
          if( ok ) {
            Serial.println("Enter New Password:");
            mode = MODE_NEWPASS1;
          }
          else {
            Serial.println("ERROR: Wrong password!");
            mode = MODE_CMD;
          }
        }
        break;

        case MODE_NEWPASS1:
        newPass1 = cmd;
        mode = MODE_NEWPASS2;
        Serial.println("Confirm New Password:");
        break;

        case MODE_NEWPASS2:
        if( newPass1 != cmd ) {
          Serial.println("ERROR: Passwords do not match!");
        }
        else {
          svdPass.len = cmd.length();
          for( byte i = 0; i < svdPass.len; ++i ) {
            svdPass.str[i] = cmd[i];
          }
          EEPROM.put(addrPass, svdPass);
          Serial.println("New password saved!");
        }
        mode = MODE_CMD;
        break;

        case MODE_CHECKPASS:
        if( cmd.length() == svdPass.len ) {
          bool ok = true;
          for( unsigned i = 0; i < cmd.length(); ++i ) {
            ok |= (cmd[i] == svdPass.str[i]);
          }
          if( ok ) {
            Serial.println("Scan new tag:");
            mode = MODE_LEARN;
            break;
          }
        }
        mode = MODE_CMD;
        break;

        case MODE_LEARN:
        Serial.println("Abort learn new tag.");
        mode = MODE_CMD;
      }
    }
  }

  if(tag1.is_valid()) {
    lightLED = millis() + 100;
    Serial.print("TAG: ");
    Serial.println(tag1.value(), HEX);
    tag1.clear();
  }

  if( RFID_SERIAL.available() > 0 ) {
    byte bt = RFID_SERIAL.read();

    if( readLen < 0 ) {
      if( (tagMatch = (bt == SOT)) ) {
        ++readLen;
      }
    }
    else if( readLen < 12 ) {
      tmpTag[readLen] = bt;
      tagMatch &= (tag[readLen] == bt);
      ++readLen;
    }
    else {
      unsigned long nowTag = millis();
      if( (bt == EOT) && ((nowTag - lastTag) > 5000) ) {
        lastTag = nowTag;
        if( mode == MODE_CMD && tagMatch ) {
          lightLED = millis() + 1000;
          if( ctrlAltDel ) {
            sendCtrlAltDel();
            delay(500);
            sendCtrlAltDel();
            delay(500);
            sendCtrlAltDel();
          }
          for( int i = 0; i < svdPass.len; ++i ) {
            Keyboard.print(svdPass.str[i]);
          }
          if( ctrlAltDel ) {
            Keyboard.println("");
          }
        }
        else if( mode == MODE_LEARN ) {
          lightLED = millis() + 1000;
          EEPROM.put( addrTag, tmpTag );
          EEPROM.get( addrTag, tag );
          Serial.println("Saved new tag!");
          mode = MODE_CMD;
        }
      }
      readLen = -1;
    }
  }
}
