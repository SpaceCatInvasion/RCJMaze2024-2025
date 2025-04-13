#pragma once
#include <vl53l7cx_class.h>
#include "Wire.h"

VL53L7CX sensor_vl53l7cx_top(&Wire1, 0);


void lidarInit();
double getOffsetDueToObject();