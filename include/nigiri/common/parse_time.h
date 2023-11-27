#pragma once

#include "date/date.h"
#include "nigiri/types.h"

namespace nigiri {

 unixtime_t parse_time(std::string_view s, char const* format);

}
