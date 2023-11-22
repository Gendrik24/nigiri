#pragma once

#include <vector>
#include <optional>
#include <variant>

#include "date/date.h"

#include "cista/containers/flat_matrix.h"

#include "nigiri/common/delta_t.h"
#include "nigiri/routing/pareto_set.h"
#include "nigiri/routing/journey.h"
#include "nigiri/routing/routing_time.h"
#include "nigiri/footpath.h"
#include "nigiri/types.h"

namespace nigiri {
struct timetable;
}

namespace nigiri::routing {

struct mc_raptor_label {
  struct transport_leg {
    transport_leg(location_idx_t enter,
                  transport via,
                  location_idx_t exit)
        : enter_(enter),
          via_(via),
          exit_(exit) {}

    location_idx_t enter_;
    transport via_;
    location_idx_t exit_;
  };

  struct footpath_leg {
    footpath_leg(duration_t duration,
                  location_idx_t target)
        : duration_(duration),
          target_(target) {}

    duration_t duration_;
    location_idx_t target_;
  };

  mc_raptor_label()
    : arrival_(routing_time::min()),
      departure_(routing_time::max()) {}

  mc_raptor_label(routing_time arrival, routing_time dep)
    : arrival_(arrival),
      departure_(dep) {}

  mc_raptor_label(routing_time arrival, routing_time dep, pareto_set<mc_raptor_label>::const_iterator prev)
      : arrival_(arrival),
        departure_(dep),
        prev_(prev) {}

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

  pareto_set<mc_raptor_label>::const_iterator prev_;

  std::optional<transport_leg> with_;
  std::optional<footpath_leg> transfer_;
};

struct mc_raptor_route_label {
  mc_raptor_route_label()
      : transport_(transport{transport_idx_t::invalid(), day_idx_t::invalid()}),
        departure_(routing_time::max()) {}

  mc_raptor_route_label(transport transport,
                        location_idx_t entered,
                        routing_time departure,
                        pareto_set<mc_raptor_label>::const_iterator prev)
      : transport_(transport),
        entered_(entered),
        departure_(departure),
        prev_(prev) {}

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
  location_idx_t entered_;
  routing_time departure_;

  pareto_set<mc_raptor_label>::const_iterator prev_;
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