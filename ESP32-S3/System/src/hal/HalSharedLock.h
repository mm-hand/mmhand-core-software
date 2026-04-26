#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Shared mutex for MUX line arbitration between encoder and tactile paths.
SemaphoreHandle_t getSharedMuxLock();

