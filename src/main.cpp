#include <Arduino.h>
#include <EEPROM.h>
#include <functional>

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

template< typename T, unsigned int SIZE >
class SimpleCircular {
  SimpleCircular() : mask(2^SIZE-1), head(0), tail(0) {}
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
  void inc(const unsigned int& ptr) {
    ptr = (ptr + 1) & mask;
  }
protected:
  unsigned int mask;
  unsigned int head;
  unsigned int tail;
  T elem[2^SIZE];
};

unsigned int times[4096];
unsigned int first = 0;
unsigned int last = 0;
unsigned int TMASK = 0xFFF;
unsigned long lastChange;
unsigned long lastTag = 0;
unsigned long lightLED = 0;
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

void sendCtrlAltDel() {
  // press and hold CTRL
  Keyboard.set_modifier(MODIFIERKEY_CTRL);
  Keyboard.send_now();

  // press ALT while still holding CTRL
  Keyboard.set_modifier(MODIFIERKEY_CTRL | MODIFIERKEY_ALT);
  Keyboard.send_now();

  // press DELETE, while CLTR and ALT still held
  Keyboard.set_key1(KEY_DELETE);
  Keyboard.send_now();

  // release all the keys at the same instant
  Keyboard.set_modifier(0);
  Keyboard.set_key1(0);
  Keyboard.send_now();
}

void pinChanged() {
  unsigned int now = micros();
  unsigned int delta = now - lastChange;
  times[first] = delta * 2 + digitalRead(rfidPin);
  lastDelta = delta;
  lastChange = now;
  first = (first + 1) & TMASK;
  if( 1000 < delta && delta < 1800 ) {
    preamble = 29;
  }
  if( 10 < delta && delta < 80 ) {
    if( preamble ) {
      --preamble;
      if( preamble == 0 ) {
        lightLED = millis() + 100;
      }
    }
  }
}

void timerGate(void) {
  digitalWrite(gatePin, !digitalRead(gatePin));
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
    while( last != first ) {
      Serial.print(times[last] / 2);
      Serial.print(":");
      Serial.println(times[last] & 0x01);
      //Serial.println(times[last]);
      last = (last + 1) & TMASK;
    }
  }
  else if( cmd == "dump8" || cmd == ".." ) {
    while( last != first ) {
      //Serial.print(times[last] / 16);
      //Serial.print(":");
      //Serial.println(times[last] & 0x01);
      Serial.println(times[last] / 8);
      last = (last + 1) & TMASK;
    }
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
  pinMode(ledPin, OUTPUT);
  pinMode(gatePin, OUTPUT);
  pinMode(rfidPin, INPUT);
  lastChange = micros();
  Serial.begin( 115200 );
  RFID_SERIAL.begin( 9600 );
  attachInterrupt( digitalPinToInterrupt(rfidPin), pinChanged, CHANGE );
  lightLED = millis() + 1000;
  EEPROM.get(addrPass, svdPass);
  EEPROM.get(addrTag, tag);
  //t.begin(timerGate, 90000);
  digitalWrite(gatePin, HIGH);
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
