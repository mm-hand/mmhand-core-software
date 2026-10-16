#pragma once

// Called only by Task_Tactile between complete SPI acquisition rounds.
// The task is the sole owner of the tactile SPI bus and selector GPIOs.
bool pollTactileMaintenance();
