#include "nigiri/common/parse_time.h"
#include "nigiri/types.h"

namespace nigiri {

  unixtime_t parse_time(std::string_view s, char const* format) {
    std::stringstream in;
    in << s;

    date::local_seconds ls;
    std::string tz;
    in >> date::parse(format, ls, tz);

    return std::chrono::time_point_cast<unixtime_t::duration>(
        date::make_zoned(tz, ls).get_sys_time());
  }

}
