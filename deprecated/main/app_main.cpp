#include <Arduino.h>
#include "SystemTask.h"

extern "C" void app_main(void)
{
    initArduino();
    System_Init();

    while (true) {
        System_Loop();
    }
}
