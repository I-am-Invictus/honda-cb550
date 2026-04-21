#pragma once
#include <Arduino.h>
#include "MotorController_DbcTypes.h"

namespace mcdbc {
bool decode(uint32_t can_id, const uint8_t *buf, uint8_t len, AnyMessage &out);
}
