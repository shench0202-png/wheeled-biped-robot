#include "DengFOC_V4_Torque_Current_Control.h"


DengFOCTorqueCurrentController controller;
TwoWire motor2Wire = TwoWire(1);

void setup() {
  Serial.begin(115200);
  Wire.begin(19, 18, 400000UL);
  motor2Wire.begin(23, 5, 400000UL);
  controller.begin(Serial, Wire, motor2Wire);
}

void loop() {
  controller.update();
}
