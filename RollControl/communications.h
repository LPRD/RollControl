// Contains defs for standard packet io protocol
// To read floating point numbers, you need to change your arduino configuration settings

// This just works, OK?

#include <avr/pgmspace.h>

#define BEGIN_SEND {              \
  Serial.print(F("@@@@@_time:")); \
  Serial.print(millis());
  
#define SEND_ITEM(field, value) \
  Serial.print(F(";"));         \
  Serial.print(F(#field));      \
  Serial.print(F(":"));         \
  Serial.print(value);
  
#define SEND_ITEM_NAME(field, value)\
  Serial.print(F(";"));         \
  Serial.print(F(field));       \
  Serial.print(F(":"));         \
  Serial.print(value);

#define END_SEND                \
  Serial.println(F("&&&&&"));   \
  Serial.flush();               \
}

#define SEND(field, value)      \
  BEGIN_SEND                    \
  SEND_ITEM(field, value)       \
  END_SEND

#define SEND_NAME(field, value) \
  BEGIN_SEND                    \
  SEND_ITEM_NAME(field, value)  \
  END_SEND

#define READ_BUFFER_SIZE 50
char _buffer[READ_BUFFER_SIZE];
char _data[READ_BUFFER_SIZE - 10];

#define CHECK_SERIAL_AVAIL            \
  if (!Serial.available()) {          \
    delay(100);                       \
    if (!Serial.available()) {        \
      Serial.println(F("READ timeout")); \
      goto L_ENDREAD;                 \
    }                                 \
  }                                   \

// Sorry about the gotos
#define BEGIN_READ                          \
  if (Serial.available()) {                 \
    char _c = '\0';                         \
    int _i;                                 \
    for (_i = 0; _c != '\n'; _i++) {        \
      if (_i == READ_BUFFER_SIZE) {         \
        Serial.println(F("READ buffer overflow"));\
        while (Serial.available() && Serial.read() != '\n')\
          CHECK_SERIAL_AVAIL                \
        goto L_ENDREAD;                     \
      }                                     \
      CHECK_SERIAL_AVAIL                    \
      _c = Serial.read();                   \
      _buffer[_i] = _c;                     \
      if (_c == '\r') _i--;                 \
    }                                       \
    _buffer[_i] = '\0';                     \
    if (!sscanf(_buffer, "@@@@@%[^&]&&&&&\n", _data)) {\
      Serial.println(F("READ packet error"));  \
      goto L_ENDREAD;                       \
    }                                       \
    if (0);
/*
    char *_item;                            \
    while ((_item = strsep((char**)&_data, ";")) != NULL) {\*/

#define READ_FIELD(field, spec)   \
  else if (sscanf(_data, F(#field":"spec), &field))

#define READ_FLAG(field)          \
  else if (!strcmp(_data, F(#field":")))

#define READ_DEFAULT(field_name, field) \
  else if (sscanf(_data, F("%[^:]:%s"), &field_name, &field))

#define END_READ } L_ENDREAD:;
