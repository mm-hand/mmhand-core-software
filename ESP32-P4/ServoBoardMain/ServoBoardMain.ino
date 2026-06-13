#include <Arduino.h>
#include "src/system/SystemTask.h"


// ServoBoardMain entry point.
void setup() {
    System_Init();
}



void loop() {
    System_Loop();
}
