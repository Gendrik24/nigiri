#include "nigiri/routing/bmc_raptor_reconstructor.h"

#include <set>

#include "nigiri/routing/limits.h"
#include "nigiri/routing/query.h"
#include "nigiri/types.h"
#include "nigiri/routing/bmc_raptor_search_state.h"
#include "nigiri/routing/for_each_meta.h"
#include "utl/helpers/algorithm.h"
#include "nigiri/special_stations.h"


namespace nigiri::routing {

bmc_raptor_reconstructor::bmc_raptor_reconstructor(const timetable& tt,
                                                   const query& q,
                                                   bmc_raptor_search_state& state,
                                                   interval<unixtime_t> search_interval)
      : tt_{tt},
        q_{q},
        state_{state},
        n_rounds_{std::min(kMaxTransfers, q_.max_transfers_) + 1U},
        search_interval_(search_interval),
        round_times_(make_flat_matrix<uncompressed_round_times_t>(n_rounds_, tt_.n_locations())),
        round_time_iters(make_flat_matrix<uncompressed_round_time_iterator>(n_rounds_, tt_.n_locations())) {}

void bmc_raptor_reconstructor::uncompress_round_bags() {
  const auto first_day_offset =
      tt_.day_idx_mam(search_interval_.from_).first;

  for (auto r = 0U; r < n_rounds_; ++r) {
    for (auto l = 0U; l < tt_.n_locations(); ++l) {
      const auto& bag = state_.round_bags_[r][l];
      auto& ert = round_times_[r][l];

      const auto& uncompressed_labels = bag.uncompress();
      ert.reserve(uncompressed_labels.size());
      for (const auto& arr_dep_l : uncompressed_labels) {
        ert.push_back(arr_dep_l.add_day_offset(to_idx(first_day_offset)));
      }
      std::sort(ert.begin(), ert.end(), departure_arrival_comparator());
      round_time_iters[r][l] = ert.begin();
    }
  }
}

void bmc_raptor_reconstructor::get_departure_events() {
  for (auto l = 0U; l < tt_.n_locations(); ++l) {
    const auto& dep_arr_times = round_times_[0][l];
    for (const auto& dat : dep_arr_times) {
      departure_events_.insert(dat.departure_);
    }
  }
}

std::pair<routing_time, std::optional<reconstruction_leg>> bmc_raptor_reconstructor::get_routing_information(const unsigned long k,
                                                                                                             const location_idx_t loc,
                                                                                                             const long_minutes_after_midnight_t departure) {
  auto& iter = round_time_iters[k][to_idx(loc)];
  const auto& dep_arr_times = round_times_[k][to_idx(loc)];

  routing_time rt = kInvalidTime<direction::kForward>;
  while (iter != dep_arr_times.end() && iter->departure_ > departure) {
    iter++;
  }
  if (iter != dep_arr_times.end() && iter->departure_ == departure) {
    rt = routing_time{iter->arrival_.count()};
    return {rt, iter->leg_};
  }
  return {rt, std::nullopt};
}

std::optional<journey::leg> bmc_raptor_reconstructor::find_start_footpath(journey const& j,
                                                                          const long_minutes_after_midnight_t departure) {
  constexpr auto const kFwd = true;

  auto const is_better_or_eq = [](auto a, auto b) {
    return kFwd ? a <= b : a >= b;
  };

  auto const is_journey_start = [&](location_idx_t const candidate_l) {
    return utl::any_of(q_.start_, [&](offset const& o) {
      return matches(tt_, q_.start_match_mode_, o.target_, candidate_l);
    });
  };

  auto const leg_start_location =
      kFwd ? j.legs_.back().from_ : j.legs_.back().to_;
  auto const leg_start_time =
      kFwd ? j.legs_.back().dep_time_ : j.legs_.back().arr_time_;

  if (q_.start_match_mode_ != location_match_mode::kIntermodal &&
      is_journey_start(leg_start_location) &&
      is_better_or_eq(j.start_time_, leg_start_time)) {
    return std::nullopt;
  }

  auto const& footpaths =
      kFwd ? tt_.locations_.footpaths_in_[leg_start_location]
           : tt_.locations_.footpaths_out_[leg_start_location];
  auto const j_start_time = routing_time{tt_, j.start_time_};
  auto const fp_target_time = get_routing_information(0, leg_start_location, departure).first;

  if (q_.start_match_mode_ == location_match_mode::kIntermodal) {
    for (auto const& o : q_.start_) {
      if (matches(tt_, q_.start_match_mode_, o.target_, leg_start_location) &&
          is_better_or_eq(j.start_time_,
                          leg_start_time - (kFwd ? 1 : -1) * o.duration_)) {
        return journey::leg{direction::kForward,
                            get_special_station(special_station::kStart),
                            leg_start_location,
                            j.start_time_,
                            j.start_time_ + (kFwd ? 1 : -1) * o.duration_,
                            o};
      }

      for (auto const& fp : footpaths) {
        if (matches(tt_, q_.start_match_mode_, o.target_, fp.target_) &&
            is_better_or_eq(
                j.start_time_,
                leg_start_time -
                    (kFwd ? 1 : -1) * (o.duration_ + fp.duration_))) {
          return journey::leg{direction::kForward,
                              get_special_station(special_station::kStart),
                              leg_start_location,
                              j.start_time_,
                              j.start_time_ + (kFwd ? 1 : -1) * o.duration_,
                              o};
        }
      }
    }
  } else {
    for (auto const& fp : footpaths) {
      if (is_journey_start(fp.target_) &&
          fp_target_time != kInvalidTime<direction::kForward> &&
          std::abs((j_start_time - fp_target_time).count()) ==
              fp.duration_.count()) {
        return journey::leg{direction::kForward,
                            fp.target_,
                            leg_start_location,
                            j.start_time_,
                            fp_target_time.to_unixtime(tt_),
                            fp};
      }
    }
  }

  throw utl::fail("no valid journey start found");
}

std::pair<journey::leg, journey::leg> bmc_raptor_reconstructor::get_legs(unsigned const k,
                                                                         location_idx_t const l,
                                                                         long_minutes_after_midnight_t departure,
                                                                         journey& j) {

  auto [arrive_at_l, optional_reconstruction_information] = get_routing_information(k, l, departure);
  if (!optional_reconstruction_information.has_value()) {
    utl::fail("ERROR! No reconstruction info is given.");
  }
  const auto& rc_info = optional_reconstruction_information.value();
  auto const& stop_seq = tt_.route_location_seq_[tt_.transport_route_[rc_info.uses_.t_idx_]];

   unsigned int transport_to_stop_idx = std::numeric_limits<unsigned int>::max();
   auto const n_stops = stop_seq.size() - rc_info.transport_from_stop_idx_;
   for (auto i = 1U; i != n_stops; ++i) {
    auto const stop_idx =
        static_cast<unsigned>(rc_info.transport_from_stop_idx_ + i);
    auto const stop = timetable::stop{stop_seq[stop_idx]};
    if (stop.location_idx() == rc_info.transport_to_) {
      transport_to_stop_idx = stop_idx;
      break;
    }
   }

   if (to_idx(rc_info.uses_.day_) < 0) {
    utl::fail("ERROR! transport is not valid!");
   }

   const auto transport_day_idx = day_idx_t{static_cast<uint16_t>(to_idx(rc_info.uses_.day_))};
   auto const dep_event_from = routing_time{transport_day_idx,
                                            tt_.event_mam(rc_info.uses_.t_idx_, rc_info.transport_from_stop_idx_, event_type::kDep)};

   auto const arr_event_to = routing_time{transport_day_idx,
                                          tt_.event_mam(rc_info.uses_.t_idx_, transport_to_stop_idx, event_type::kArr)};

   const auto transport_leg = journey::leg{
        direction::kForward,
        rc_info.transport_from_,
        rc_info.transport_to_,
        dep_event_from.to_unixtime(tt_),
        arr_event_to.to_unixtime(tt_),
        journey::transport_enter_exit{
          transport{rc_info.uses_.t_idx_, transport_day_idx}, rc_info.transport_from_stop_idx_, transport_to_stop_idx}
   };

   footpath fp;
   fp.target_ = l;
   fp.duration_ = arrive_at_l - arr_event_to;

   if (l == rc_info.transport_to_) {
    fp.target_ = l;
    fp.duration_ = (k == j.transfers_ + 1U)
                       ? 0_minutes
                       : tt_.locations_.transfer_time_[l];

    arrive_at_l = arr_event_to + fp.duration_;
   }

   const auto footpath_leg = journey::leg{
       direction::kForward,
       rc_info.transport_to_,
       l,
       arr_event_to.to_unixtime(tt_),
       arrive_at_l.to_unixtime(tt_),
       fp
   };


   if (k == j.transfers_ + 1U) {
    j.dest_time_ = footpath_leg.arr_time_;
   }


   return {footpath_leg, transport_leg};
}

void bmc_raptor_reconstructor::reconstruct_journey(journey& j,
                                                   const long_minutes_after_midnight_t departure_time) {
  auto l = j.dest_;
  for (auto i = 0U; i <= j.transfers_; ++i) {
    auto const k = j.transfers_ + 1 - i;
    auto [fp_leg, transport_leg] = get_legs(k, l, departure_time, j);
    l = transport_leg.from_;
    j.add(std::move(fp_leg));
    j.add(std::move(transport_leg));
  }

  auto init_fp = find_start_footpath(j, departure_time);
  if (init_fp.has_value()) {
    j.add(std::move(*init_fp));
  }

  std::reverse(begin(j.legs_), end(j.legs_));
}

void bmc_raptor_reconstructor::reconstruct_for_destination(std::size_t dest_idx,
                                                           location_idx_t dest,
                                                           const long_minutes_after_midnight_t departure_time) {
  for (auto k = 1U; k != n_rounds_; ++k) {
    auto& dest_iter = round_time_iters[k][to_idx(dest)];
    const auto& dest_dep_arr_events = round_times_[k][to_idx(dest)];

    routing_time dest_time = kInvalidTime<direction::kForward>;
    while (dest_iter != dest_dep_arr_events.end() && dest_iter->departure_ > departure_time) {
      dest_iter++;
    }
    if (dest_iter != dest_dep_arr_events.end() && dest_iter->departure_ == departure_time) {
      dest_time = routing_time{dest_iter->arrival_.count()};
    }

    if (dest_time == kInvalidTime<direction::kForward>) {
      continue;
    }

    auto const [optimal, it] = state_.results_[dest_idx].add(journey{
        .legs_ = {},
        .start_time_ = routing_time(departure_time.count()).to_unixtime(tt_),
        .dest_time_ = dest_time.to_unixtime(tt_),
        .dest_ = dest,
        .transfers_ = static_cast<std::uint8_t>(k - 1)});
    if (optimal) {
      auto const outside_interval =
          !search_interval_.contains(it->start_time_);
      if (!outside_interval) {
        try {
          reconstruct_journey(*it, departure_time);
        } catch (std::exception const& e) {
          state_.results_[dest_idx].erase(it);
          log(log_lvl::error, "routing", "reconstruction failed: {}", e.what());
        }
      }
    }
  }
}

void bmc_raptor_reconstructor::reconstruct() {
  uncompress_round_bags();
  get_departure_events();

  for (const auto& dep_time : departure_events_) {

    for (auto const [i, t] : utl::enumerate(q_.destinations_)) {
      for (auto const dest : state_.destinations_[i]) {
        reconstruct_for_destination(i, dest, dep_time);
      }
    }
  }

}

}
