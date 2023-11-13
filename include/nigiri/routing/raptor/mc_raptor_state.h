#pragma once

#include <vector>

#include "date/date.h"

#include "cista/containers/flat_matrix.h"

#include "nigiri/common/delta_t.h"
#include "nigiri/routing/pareto_set.h"
#include "nigiri/routing/journey.h"
#include "nigiri/routing/routing_time.h"

namespace nigiri {
struct timetable;
}

namespace nigiri::routing {

struct mc_raptor_label {

  mc_raptor_label()
      : arrival_(routing_time::min()),
        departure_(routing_time::max())
  {}

  mc_raptor_label(routing_time arr, routing_time dep) :
    arrival_(arr),
    departure_(dep) {}

  inline bool equals(const mc_raptor_label& other) const noexcept {
        return  arrival_ == other.arrival_ &&
                departure_ == other.departure_;
  }

  inline bool dominates(const mc_raptor_label& other) const noexcept {
        return  arrival_ <= other.arrival_ &&
                departure_ >= other.departure_;
  }
    routing_time arrival_;
    routing_time departure_;
};

struct mc_raptor_route_label {
    mc_raptor_route_label()
        : transport_(transport{transport_idx_t::invalid(), day_idx_t::invalid()}),
          departure_(routing_time::max())
    {}

    mc_raptor_route_label(transport transport,
                          routing_time departure)
        : transport_(transport),
          departure_(departure)
    {}

    inline bool equals(const mc_raptor_route_label& other) const noexcept {
        return  departure_ == other.departure_ &&
               transport_.t_idx_ == other.transport_.t_idx_ &&
               transport_.day_ == other.transport_.day_;
    }

    inline bool dominates(const mc_raptor_route_label& other) const noexcept {
        return  departure_ >= other.departure_ &&
               (transport_.day_ < other.transport_.day_ || (transport_.day_ == other.transport_.day_ && transport_.t_idx_ <= other.transport_.t_idx_));
    }

    transport transport_;
    routing_time departure_;
};

struct mc_raptor_state {
  mc_raptor_state() = default;
  mc_raptor_state(mc_raptor_state const&) = delete;
  mc_raptor_state& operator=(mc_raptor_state const&) = delete;
  mc_raptor_state(mc_raptor_state&&) = default;
  mc_raptor_state& operator=(mc_raptor_state&&) = default;
  ~mc_raptor_state() = default;

  void resize(unsigned n_locations,
              unsigned n_routes);

  std::vector<pareto_set<mc_raptor_label>> best_;
  cista::raw::flat_matrix<pareto_set<mc_raptor_label>> round_bags_;
  std::vector<bool> station_mark_;
  std::vector<bool> prev_station_mark_;
  std::vector<bool> route_mark_;
  std::vector<pareto_set<journey>> results_;
};

}  // namespace nigiri::routing