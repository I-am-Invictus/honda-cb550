#pragma once
#include <stdint.h>
#include "DbcTypes.h"

namespace dbc {

// Returns true if decoded into `out`, false if the ID isn’t recognized or DLC is invalid.
bool decode(uint32_t can_id, const uint8_t data[8], uint8_t dlc, AnyMessage &out);

} // namespace dbc
