#ifndef DENGFOC_LIBRARY_CURRENT_CONTROL_H
#define DENGFOC_LIBRARY_CURRENT_CONTROL_H

#include <Arduino.h>
#include <Wire.h>

class DengFOCLibraryCurrentController {
public:
  void begin(Stream &serialPort, TwoWire &motor1Wire, TwoWire &motor2Wire);
  void update();
};

#endif
