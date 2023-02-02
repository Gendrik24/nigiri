#include "nigiri/routing/profile_raptor.h"

#include <string>
#include <algorithm>
#include <set>
#include <chrono>

#include "nigiri/routing/search_state.h"
#include "nigiri/routing/start_times.h"
#include "nigiri/types.h"
#include "nigiri/location.h"
#include "nigiri/timetable.h"
#include "nigiri/dynamic_bitfield.h"
#include "nigiri/tracer.h"
#include "nigiri/special_stations.h"
#include "nigiri/routing/reconstruct.h"
#include "nigiri/location.h"

#include "utl/overloaded.h"
#include "utl/enumerate.h"
#include "utl/erase_if.h"

#include "fmt/core.h"

namespace nigiri::routing {

profile_raptor::profile_raptor(const timetable& tt,
                               profile_search_state& state,
                               query q)
    : tt_{tt},
      n_tt_days_{static_cast<std::uint16_t>(tt_.date_range_.size().count())},
      q_(q),
      search_interval_(q_.start_time_.apply(utl::overloaded{
          [](interval<unixtime_t> const& start_interval) { return start_interval; },
          [this](unixtime_t const start_time) {
            return interval<unixtime_t>{start_time, tt_.end()};
          }})),
      state_{state}{};

day_idx_t profile_raptor::start_day_offset() const {
  return tt_.day_idx_mam(this->search_interval_.from_).first;
}

day_idx_t profile_raptor::number_of_days_in_search_interval() const {
  return tt_.day_idx_mam(this->search_interval_.to_).first
         - tt_.day_idx_mam(this->search_interval_.from_).first + 1;
}

bool profile_raptor::is_dominated_by_best_bags(const raptor_label& l) {
  for (auto l_idx = location_idx_t{0U}; l_idx != tt_.n_locations(); ++l_idx) {
    if (!state_.is_destination_[to_idx(l_idx)]) {
      continue;
    }

    if (!state_.best_bag_[to_idx(l_idx)].dominates(l)) {
      return false;
    }
  }

  return true;
}

unsigned profile_raptor::end_k() const {
  return std::min(kMaxTransfers, q_.max_transfers_) + 1U;
}

void profile_raptor::init_location_with_offset(timetable const& tt,
                                               minutes_after_midnight_t time_to_arrive,
                                               location_idx_t location,
                                               day_idx_t n_days_in_search_interval,
                                               matrix<raptor_bag>& round_bags_,
                                               std::vector<bool>& station_mark,
                                               std::vector<raptor_bag>& best_bags) {
  for (auto const& r : tt.location_routes_.at(location)) {

    auto const location_seq = tt.route_location_seq_.at(r);
    for (auto const [i, s] : utl::enumerate(location_seq)) {
      if (timetable::stop{s}.location_idx() != location) {
        continue;
      }

      auto const& transport_range = tt.route_transport_ranges_[r];
      for (auto transport_idx = transport_range.from_;
           transport_idx != transport_range.to_; ++transport_idx) {

        auto const stop_time =
            tt.event_mam(transport_idx, i,
                         event_type::kDep);

        auto const stop_time_day_offset = stop_time.count() / 1440;
        auto const stop_time_mam = stop_time % 1440;

        auto start_time = stop_time_mam - time_to_arrive;
        if (start_time < minutes_after_midnight_t::zero()) {
          start_time = (1_days - minutes_after_midnight_t{std::abs(start_time.count()) % 1440});
        }
        const auto at_stop = start_time + time_to_arrive;
        const int trip_tdb_bias_shift = start_day_offset().v_
                                        - stop_time_day_offset
                                        + (at_stop.count() / 1440);

        auto trip_tdb = tt_.bitfields_[tt_.transport_traffic_days_[transport_idx]];
        if (trip_tdb_bias_shift >= 0) {
          trip_tdb >>= trip_tdb_bias_shift;
        } else {
          trip_tdb <<= std::abs(trip_tdb_bias_shift);
        }

        dynamic_bitfield traffic_day_bitfield{
            trip_tdb,
            this->number_of_days_in_search_interval().v_
        };

        auto mask = std::string(n_days_in_search_interval.v_, '1');
        const auto dep_lower_limit = search_interval_.from_.time_since_epoch();
        if (start_time < minutes_after_midnight_t{dep_lower_limit.count() % 1440}) {
          mask.back() = '0';
        }

        const auto dep_upper_limit = search_interval_.to_.time_since_epoch();
        if (start_time >= minutes_after_midnight_t{dep_upper_limit.count() % 1440}) {
          mask.front() = '0';
        }
        traffic_day_bitfield &= dynamic_bitfield{mask, n_days_in_search_interval.v_};

        if (traffic_day_bitfield.any() && (at_stop-start_time) <= minutes_after_midnight_t{kMaxTravelTime}) {
          bool merged = round_bags_[0U][to_idx(location)]
                            .add(raptor_label{
                                at_stop,
                                start_time,
                                traffic_day_bitfield
                            }).first;

          station_mark[to_idx(location)] = merged || station_mark[to_idx(location)];
          if (merged) {
            best_bags[to_idx(location)].add(raptor_label{
                at_stop,
                start_time,
                traffic_day_bitfield
            });
          }
        }
      }
    }
    auto const first_out_of_interval = search_interval_.to_.time_since_epoch() % 1440;

    dynamic_bitfield traffic_day_bitfield{
        std::string(n_days_in_search_interval.v_, '0').replace(0U, 1U, "1"),
        this->number_of_days_in_search_interval().v_
    };
    bool merged = round_bags_[0U][to_idx(location)]
                      .add(raptor_label{
                          first_out_of_interval + time_to_arrive,
                          first_out_of_interval,
                          traffic_day_bitfield
                      }).first;

    station_mark[to_idx(location)] = merged || station_mark[to_idx(location)];
    if (merged) {
      best_bags[to_idx(location)].add(raptor_label{
          first_out_of_interval + time_to_arrive,
          first_out_of_interval,
          traffic_day_bitfield
      });
    }
  }

}

void profile_raptor::init_starts() {
  for (const auto& o : q_.start_) {
    location_idx_t loc = o.target_;
    init_location_with_offset(tt_,
                              o.duration_,
                              loc,
                              number_of_days_in_search_interval(),
                              state_.round_bags_,
                              state_.station_mark_,
                              state_.best_bag_);

    // It is possible to transfer to another location and start from there
    if (q_.use_start_footpaths_) {

      // First get all outgoing footpaths
      auto const footpaths_out = tt_.locations_.footpaths_out_[loc];
      for (const auto& fp : footpaths_out) {

        init_location_with_offset(tt_,
                                  o.duration_ + fp.duration_,
                                  fp.target_,
                                  number_of_days_in_search_interval(),
                                  state_.round_bags_,
                                  state_.station_mark_,
                                  state_.best_bag_);
      }
    }
  }
}


void profile_raptor::route() {
  state_.reset(tt_);
  state_.search_interval_ = search_interval_;

  collect_destinations(tt_, q_.destinations_, q_.dest_match_mode_,
                       state_.destinations_, state_.is_destination_);
  init_starts();
  rounds();
  reconstruct();
  for (auto& r : state_.results_) {
    utl::erase_if(r, [&](journey const& j) {
      return !search_interval_.contains(
          j.start_time_);
    });
  }
}

void profile_raptor::rounds() {
  print_state();
  for (auto k = 1U; k != end_k(); ++k) {
    trace_always("┊ round k={}\n", k);
    // Round k
    auto any_marked = false;
    for (auto l_idx = location_idx_t{0U};
         l_idx != static_cast<cista::base_t<location_idx_t>>(
                      state_.station_mark_.size()); ++l_idx) {
      
      if (state_.station_mark_[to_idx(l_idx)]) {
        any_marked = true;
        for (auto const& r : tt_.location_routes_[l_idx]) {
          state_.route_mark_[to_idx(r)] = true;
        }
      }
    }
    
    std::swap(state_.prev_station_mark_, state_.station_mark_);
    std::fill(begin(state_.station_mark_), end(state_.station_mark_), false);
    
    if (!any_marked) {
      trace_always("┊ ╰ no routes marked, exit\n\n");
      return;
    }

    any_marked = false;
    for (auto r_id = 0U; r_id != tt_.n_routes(); ++r_id) {
      if (!state_.route_mark_[r_id]) {
        continue;
      }
      trace("┊ ├ updating route {}\n", r_id);
      any_marked |= update_route(k, route_idx_t{r_id});
    }

    std::fill(begin(state_.route_mark_), end(state_.route_mark_), false);
    if (!any_marked) {
      trace_always("┊ ╰ no stations marked, exit\n\n");
      return;
    }
    update_footpaths(k);
    trace_always("┊ ╰ round {} done\n", k);
    print_state();
  }
}

bool profile_raptor::update_route(unsigned const k, route_idx_t route_idx) {

  auto any_marked = false;
  auto const stop_sequence = tt_.route_location_seq_[route_idx];

  raptor_bag r_b{};
  for (auto i = 0U; i != stop_sequence.size(); ++i) {
    auto const stop_idx =
        static_cast<unsigned>(i);
    auto const stop = timetable::stop{stop_sequence[stop_idx]};
    auto const l_idx = cista::to_idx(stop.location_idx());

    trace(
        "┊ │  stop_idx={}, location=(name={}, id={}, idx={})\n",
        stop_idx, tt_.locations_.names_[location_idx_t{l_idx}].view(),
        tt_.locations_.ids_[location_idx_t{l_idx}].view(), l_idx);

    // 1. Iterate over Labels in Route Bag and update arrival times according to assigned transport
    trace(
        "┊ │    Update arrival times of labels in route bag:\n");
    for (auto it = r_b.begin(); it != r_b.end(); ++it) {
      auto& active_label = *it;

      if (active_label.t_.t_idx_ == transport_idx_t::invalid()) {
        trace(
            "┊ │    │ current label={} has no active transport -> SKIP!\n",
            active_label.to_string());
        continue;
      }
      const auto new_arr = tt_.event_mam(route_idx,
                                         active_label.t_.t_idx_,
                                         stop_idx,
                                         event_type::kArr) + minutes_after_midnight_t{active_label.t_.day_.v_ * 1440};

      trace(
          "┊ │    │ current label={}, active transport={} -> updated arrival time={}, transfer time={}\n",
          active_label.to_string(), active_label.t_, new_arr, new_arr_with_transfer-new_arr);
      active_label.arrival_ = new_arr;
    }


    // 2. Merge Route Bag into round bag
    trace(
        "┊ │    Merge route bag into round bag:\n");
    if (stop.out_allowed()) {
      auto const transfer_time_offset = tt_.locations_.transfer_time_[location_idx_t{l_idx}];
      auto const is_destination = state_.is_destination_[l_idx];
      for (const auto& rl : r_b) {
        /*
        if (!state_.is_destination_[l_idx] && is_dominated_by_best_bags(rl)) {
          trace("┊ │    │ label={}, dominated by best target bags -> SKIP!\n", rl.to_string());
          continue;
        }
        */
        raptor_bag& best_bag_of_stop = state_.best_bag_[l_idx];
        if (state_.round_bags_[k][cista::to_idx(l_idx)].add(raptor_label{
                                                          rl.arrival_ + (is_destination ? minutes_after_midnight_t::zero() : transfer_time_offset),
                                                          rl.departure_,
                                                          rl.traffic_day_bitfield_
                                                            }).first) {
          trace("┊ │    │ label={}, not dominated -> new best bag label!\n", rl.to_string());
          state_.station_mark_[l_idx] = true;
          any_marked = true;
        } else {
          trace("┊ │    │ label={}, dominated by best local bag -> SKIP!\n", rl.to_string());
        }
      }
    }

    if (i == stop_sequence.size()-1) {
      return any_marked;
    }


    // 3. Assign Trips to labels from previous round and merge them into route bag
    if (stop.in_allowed()) {
      const raptor_bag& prev_round = state_.round_bags_[k-1][cista::to_idx(l_idx)];
      trace(
          "┊ │    Assign trips to labels from prev round:\n");
      for (const auto& l : prev_round) {
        trace("┊ │    ├ search for transports serving label={}\n", l.to_string());
        get_earliest_sufficient_transports(l, route_idx, stop_idx, r_b);
      }
    }
  }
  return any_marked;
}


void profile_raptor::update_footpaths(unsigned const k) {
  trace("┊ ├ FOOTPATHS\n");
  std::vector<std::vector<raptor_label>> buffered_labels{tt_.n_locations(), std::vector<raptor_label>()};
  for (auto l_idx = location_idx_t{0U}; l_idx != tt_.n_locations(); ++l_idx) {
    trace("┊ ├ updating footpaths of {} ({} of {})\n", location{tt_, l_idx}, l_idx, tt_.n_locations());
    if (!state_.station_mark_[to_idx(l_idx)]) {
      trace("┊ │    ├ Not marked -> SKIP!\n");
      continue;
    }
    const auto& round_bag = state_.round_bags_[k][to_idx(l_idx)];
    auto const fps = tt_.locations_.footpaths_out_[l_idx];
    for (auto const& fp : fps) {
      auto const target = to_idx(fp.target_);
      auto const fp_offset = fp.duration_ - ((state_.is_destination_[to_idx(l_idx)]) ? minutes_after_midnight_t::zero() : tt_.locations_.transfer_time_[l_idx]);
      trace("┊ │    ├ Footpath of duration {} to target {} merging {} labels\n", fp_offset, location{tt_, fp.target_}, round_bag.size());
      for (auto it = round_bag.begin(); it != round_bag.end(); ++it) {
        const raptor_label new_rl{
          it->arrival_ + fp_offset,
              it->departure_,
              it->traffic_day_bitfield_};

        /*
        if (!state_.is_destination_[target] && is_dominated_by_best_bags(new_rl)) {
          continue;
        }
         */
        buffered_labels[target].push_back(new_rl);

      }
    }
  }

  for (auto l_idx = location_idx_t{0U}; l_idx != tt_.n_locations(); ++l_idx) {
    for (const auto& l : buffered_labels[to_idx(l_idx)]) {
      if (state_.round_bags_[k][to_idx(l_idx)].add(l).first) {
        state_.station_mark_[to_idx(l_idx)] = true;
      }
    }
  }

}

void profile_raptor::get_earliest_sufficient_transports(const raptor_label label,
                                                        route_idx_t const r,
                                                        unsigned const stop_idx,
                                                        pareto_set<raptor_label>& bag) {
  const auto lbl_dep_time = label.departure_;
  auto lbl_arr_time = label.arrival_;
  auto lbl_tdb = label.traffic_day_bitfield_;

  auto const n_days_to_iterate =
      std::min(kMaxTravelTime / 1440U + 1,
               static_cast<unsigned int>(n_tt_days_ - to_idx(start_day_offset())));

  auto const stop_event_times = tt_.event_times_at_stop(
      r, stop_idx, event_type::kDep);

  for (auto i = day_idx_t::value_t{0U}; i != n_days_to_iterate; ++i) {
    const auto day = (lbl_arr_time.count() / 1440U) + i;
    if (!lbl_tdb.any()) {
      return;
    }

    auto const time_range_to_scan = it_range{
        i == 0U ? std::lower_bound(
                      stop_event_times.begin(),
                      stop_event_times.end(), lbl_arr_time,
                      [&](auto&& a, auto&& b) { return (a % 1440U) < (b % 1440U); }) : stop_event_times.begin(),
        stop_event_times.end()};

    if (time_range_to_scan.empty()) {
      continue;
    }
    auto const base =
        static_cast<unsigned>(&*time_range_to_scan.begin_ - stop_event_times.data());

    for (auto const [t_offset, ev] : utl::enumerate(time_range_to_scan)) {
      auto const ev_mam = minutes_after_midnight_t{
          ev.count() < 1440 ? ev.count() : ev.count() % 1440};

      auto const ev_day_offset = static_cast<day_idx_t::value_t>(
          ev.count() < 1440
              ? 0
              : static_cast<cista::base_t<day_idx_t>>(ev.count() / 1440));

      const auto travel_time_lb = ev_mam + minutes_after_midnight_t{day * 1440U} - lbl_dep_time;
      if (travel_time_lb.count() > kMaxTravelTime || !lbl_tdb.any()) {
        return;
      }

      auto const t = tt_.route_transport_ranges_[r][base + t_offset];

      const int trip_tdb_bias_shift = start_day_offset().v_
                                      - ev_day_offset
                                      + day;
      auto trip_traffic_day_bitfield = dynamic_bitfield{
          (trip_tdb_bias_shift >= 0) ?
                                     tt_.bitfields_[tt_.transport_traffic_days_[t]] >> trip_tdb_bias_shift :
                                     tt_.bitfields_[tt_.transport_traffic_days_[t]] << std::abs(trip_tdb_bias_shift),
          this->number_of_days_in_search_interval().v_
      };


      if (!trip_traffic_day_bitfield.any()) continue;

      const auto new_td_bitfield = lbl_tdb & (~trip_traffic_day_bitfield);
      if ((new_td_bitfield ^ lbl_tdb).any()) {
        const auto ins = raptor_label{
          label.arrival_,
              label.departure_,
              lbl_tdb & trip_traffic_day_bitfield,
              relative_transport{t, relative_day_idx_t{day - ev_day_offset}}
        };
        bag.add(ins);
        lbl_tdb = new_td_bitfield;
      }
    }
  }
}


void profile_raptor::reconstruct() {
  state_.results_.resize(
      std::max(state_.results_.size(), state_.destinations_.size()));

  const auto r_max = kMaxTransfers + 1;
  matrix<std::vector<dep_arr_t>> expanded_round_times =
      make_flat_matrix<std::vector<dep_arr_t>>(r_max, tt_.n_locations());

  matrix<std::vector<dep_arr_t>::iterator> curr_dep_time_iter =
      make_flat_matrix<std::vector<dep_arr_t>::iterator>(r_max, tt_.n_locations());

  const auto dep_arr_cmp = [](const dep_arr_t& el1, const dep_arr_t& el2) {
    // First sort from latest departure to earliest departure
    if (el1.first > el2.first) {
      return true;
    } else if (el1.first < el2.first) {
      return false;
    }

    // If same departure sort from earliest arrival to latest
    if (el1.second < el2.second) {
      return true;
    }

    return false;
  };

  const auto dep_cmp = [](const routing_time& rt1, const routing_time& rt2) {
    if (rt1 > rt2) {
      return true;
    }

    return false;
  };

  for (auto r = 0U; r < r_max; ++r) {
    for (auto l = 0U; l < tt_.n_locations(); ++l) {
      const auto& bag = state_.round_bags_[r][l];
      auto& ert = expanded_round_times[r][l];

      for (const auto& label : bag) {
        const auto& resolved = label.resolve_label(to_idx(start_day_offset()));
        ert.insert(ert.end(), resolved.begin(), resolved.end());
      }

      //1.sort round times here
      std::sort(ert.begin(), ert.end(), dep_arr_cmp);
      //2.Initialize iterator to first element
      curr_dep_time_iter[r][l] = ert.begin();
    }
  }

  std::set<routing_time, decltype(dep_cmp)> dep_times(dep_cmp);
  for (auto l = 0U; l < tt_.n_locations(); ++l) {
    const auto& dep_arr_times = expanded_round_times[0][l];
    for (const auto& dat : dep_arr_times) {
      dep_times.insert(dat.first);
    }
  }

  std::vector<routing_time> best_times(tt_.n_locations(), kInvalidTime<direction::kForward>);

  matrix<routing_time> round_times =
      make_flat_matrix<routing_time>(
          kMaxTransfers + 1U,
          tt_.n_locations(),
          kInvalidTime<direction::kForward>);

  for (const auto& dep_time : dep_times) {

    for (auto r = 0U; r < r_max; ++r) {
      for (auto l = 0U; l < tt_.n_locations(); ++l) {
        auto& iter = curr_dep_time_iter[r][l];

        while (iter != expanded_round_times[r][l].end() && iter->first > dep_time) {
          iter++;
        }

        if (iter != expanded_round_times[r][l].end() && iter->first == dep_time) {
          round_times[r][l] = iter->second;
        }
      }
    }
    for (auto const [i, t] : utl::enumerate(q_.destinations_)) {
      for (auto const dest : state_.destinations_[i]) {
        reconstruct_for_destination(
            i,
            dest,
            best_times,
            round_times,
            dep_time.to_unixtime(tt_),
            state_.results_);
      }
    }

    std::fill(begin(best_times), end(best_times), kInvalidTime<direction::kForward>);
    round_times.reset(kInvalidTime<direction::kForward>);
  }
}


void profile_raptor::reconstruct_for_destination(std::size_t dest_idx,
                                                 location_idx_t dest,
                                                 std::vector<routing_time> const& best_times,
                                                 matrix<routing_time> const& round_times,
                                                 const unixtime_t start_at_start,
                                                 std::vector<pareto_set<journey>>& results) {
  for (auto k = 1U; k != end_k(); ++k) {
    if (round_times[k][to_idx(dest)] == kInvalidTime<direction::kForward>) {
      continue;
    }
    auto const [optimal, it] = results[dest_idx].add(journey{
        .legs_ = {},
        .start_time_ = start_at_start,
        .dest_time_ = round_times[k][to_idx(dest)].to_unixtime(tt_),
        .dest_ = dest,
        .transfers_ = static_cast<std::uint8_t>(k - 1)});
    if (optimal) {
      auto const outside_interval =
          !search_interval_.contains(it->start_time_);
          if (!outside_interval) {
            try {
              reconstruct_journey<direction::kForward>(tt_, q_, *it, best_times, round_times);
            } catch (std::exception const& e) {
              results[dest_idx].erase(it);
              log(log_lvl::error, "routing", "reconstruction failed: {}", e.what());
              print_state("RECONSTRUCT FAILED");
              fmt::print("Start at {}, Dest time {}, destination {}, transfers {}\n", start_at_start, round_times[k][to_idx(dest)].to_unixtime(tt_), dest, k-1);
            }
          }
    }
  }
}

#ifdef NIGIRI_RAPTOR_TRACING
void profile_raptor::print_state(char const* comment) {
  force_print_state(comment);
}
#else
void profile_raptor::print_state(char const*) {}
#endif

}
