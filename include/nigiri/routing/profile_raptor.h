#pragma once

#include "nigiri/routing/query.h"
#include "nigiri/routing/routing_time.h"
#include "nigiri/routing/raptor.h"

namespace nigiri {
struct timetable;
}

namespace nigiri::routing {

struct profile_search_state;

struct profile_raptor {
  profile_raptor(timetable const& tt, profile_search_state& state, query q);
  void init_starts();
  void route();



  day_idx_t start_day_offset() const;
  day_idx_t number_of_days_in_search_interval() const;

  timetable const& tt_;
  std::uint16_t n_tt_days_;
  query q_;
  interval<unixtime_t> search_interval_;
  profile_search_state& state_;
  stats stats_;
};

}