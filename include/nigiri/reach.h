#pragma once

#include "nigiri/timetable.h"
#include "nigiri/types.h"
#include "nigiri/reach_store.h"
#include "nigiri/routing/journey.h"

using namespace nigiri;
using namespace nigiri::routing;

namespace nigiri {

  reach_t get_location_reach(reach_store const& rs, location_idx_t loc);

  reach_t get_route_reach(reach_store const& rs,
                          stop_idx_t s,
                          route_idx_t r);

  void update_location_reach(reach_store& rs,
                             location_idx_t loc,
                             reach_t const& reach);


  void update_route_reach(reach_store& rs,
                          stop_idx_t s,
                          route_idx_t r,
                          reach_t const& reach);

  void update_transport_reach(reach_store& rs,
                              timetable const& tt,
                              stop_idx_t s,
                              transport_idx_t t,
                              reach_t const& reach);

  void compute_reach(reach_store& rs,
                     timetable const& tt,
                     const journey& j);

  void compute_reach(reach_store& rs,
                     timetable const& tt,
                     const pareto_set<journey>& journeys);

  void init_reach_store(reach_store& rs, timetable const& tt);

  void add_reach_store_for(interval<nigiri::unixtime_t> const& time_range, timetable& tt);

}