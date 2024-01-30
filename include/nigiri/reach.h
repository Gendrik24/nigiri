#pragma once

#include "nigiri/timetable.h"
#include "nigiri/types.h"
#include "nigiri/reach_store.h"
#include "nigiri/routing/journey.h"

namespace nigiri {

  void update_location_reach(reach_store& rs,
                             location_idx_t loc,
                             reach_t const& reach);

  void update_route_reach(reach_store& rs,
                          timetable const& tt,
                          stop_idx_t s,
                          transport_idx_t t,
                          reach_t const& reach);

  void update_transport_reach(reach_store& rs,
                              timetable const& tt,
                              stop_idx_t s,
                              transport_idx_t t,
                              reach_t const& reach);

  void compute_reach(reach_store& rs,
                     timetable const& tt,
                     const nigiri::routing::journey& j);

  void compute_reach(reach_store& rs,
                     timetable const& tt,
                     const pareto_set<nigiri::routing::journey>& journeys);

  void compute_reach(reach_store& rs,
                     timetable const& tt,
                     interval<unixtime_t> const& time_range,
                     unsigned int num_threads);

  void init_reach_store(reach_store& rs, timetable const& tt);

  void add_reach_store_for(interval<nigiri::unixtime_t> const& time_range,
                           timetable& tt,
                           unsigned int num_threads);

  void extend_reach_store_by(reach_store_idx_t rs_idx,
                             duration_t const duration,
                             timetable& tt,
                             unsigned int num_threads);

}