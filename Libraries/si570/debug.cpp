
#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include "debug.h"

void debug(char const *fmt, ... ) {
  char tmp[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(tmp, sizeof(tmp), fmt, args);
  va_end (args);
  Serial.println(tmp);
}
