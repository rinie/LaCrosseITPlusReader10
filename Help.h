// Help.h

#ifndef _HELP_h
#define _HELP_h

#ifndef ESP32	// ESP32 is not memory constrained yet
#define NOHELP
#endif

#ifndef NOHELP

#include "Arduino.h"

class Help {
 private:

 public:
   static void Show();
};

#endif

#endif

