#include "nigiri/routing/profile_raptor.h"

#include <string>
#include <algorithm>
#include <set>

#include "nigiri/routing/search_state.h"
#include "nigiri/routing/start_times.h"
#include "nigiri/types.h"
#include "nigiri/timetable.h"
#include "nigiri/dynamic_bitfield.h"
#include "nigiri/tracer.h"
#include "nigiri/special_stations.h"
#include "nigiri/routing/reconstruct.h"

#include "utl/overloaded.h"
#include "utl/enumerate.h"

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

void init_location_with_offset(timetable const& tt,
                               minutes_after_midnight_t time_to_arrive,
                               location_idx_t location,
                               day_idx_t n_days_in_search_interval,
                               cista::raw::matrix<raptor_bag>& round_bags_,
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

        auto preliminary_start_time = stop_time - time_to_arrive;
        auto day_offset = preliminary_start_time < duration_t::zero()
            ? -(preliminary_start_time - 1_days).count() / 1440 : 0;

        const auto start_time = preliminary_start_time + duration_t {day_offset * 60 * 24};
        const auto at_stop = start_time + time_to_arrive;

        //Initialize traffic day bitfield and shift by number of days offset
        dynamic_bitfield traffic_day_bitfield{std::string(n_days_in_search_interval.v_, '1'), n_days_in_search_interval.v_};
        traffic_day_bitfield >>= day_offset;


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

      auto const transfer_time_offset = tt_.locations_.transfer_time_[location_idx_t{l_idx}];
      auto const is_destination = state_.is_destination_[l_idx];
      auto const new_arr_with_transfer = new_arr + (is_destination ? minutes_after_midnight_t::zero() : transfer_time_offset);

      trace(
          "┊ │    │ current label={}, active transport={} -> updated arrival time={}, transfer time={}\n",
          active_label.to_string(), active_label.t_, new_arr, new_arr_with_transfer-new_arr);
      active_label.arrival_ = new_arr_with_transfer;
    }

    // 2. Merge Route Bag into round bag
    trace(
        "┊ │    Merge route bag into round bag:\n");
    for (const auto& rl : r_b) {
      if (!state_.is_destination_[l_idx] && is_dominated_by_best_bags(rl)) {
        trace("┊ │    │ label={}, dominated by best target bags -> SKIP!\n", rl.to_string());
        continue;
      }

      raptor_bag& best_bag_of_stop = state_.best_bag_[l_idx];
      if (best_bag_of_stop.add(rl).first) {
        trace("┊ │    │ label={}, not dominated -> new best bag label!\n", rl.to_string());
        state_.round_bags_[k][cista::to_idx(l_idx)].add(rl);
        any_marked = true;
      } else {
        trace("┊ │    │ label={}, dominated by best local bag -> SKIP!\n", rl.to_string());
      }
    }
    if (any_marked) {
      state_.station_mark_[l_idx] = true;
    }

    // 3. Assign Trips to labels from previous round and merge them into route bag
    const raptor_bag& prev_round = state_.round_bags_[k-1][cista::to_idx(l_idx)];
    trace(
        "┊ │    Assign trips to labels from prev round:\n");
    for (const auto& l : prev_round) {
      trace("┊ │    ├ search for transports serving label={}\n", l.to_string());
      std::vector<transport> e_transports;
      get_earliest_sufficient_transports(l, route_idx, stop_idx, e_transports);
      dynamic_bitfield to_serve = l.traffic_day_bitfield_;
      auto alr_shifted = size_t{0};
      for (const auto& transport : e_transports) {
        to_serve <<= transport.day_.v_ - alr_shifted;
        alr_shifted += transport.day_.v_ - alr_shifted;

        auto trip_traffic_day_bitfield = dynamic_bitfield{
            tt_.bitfields_[tt_.transport_traffic_days_[transport.t_idx_]] >> this->start_day_offset().v_,
            this->number_of_days_in_search_interval().v_
        };

        r_b.add(raptor_label{
            l.arrival_,
            l.departure_,
            (to_serve & trip_traffic_day_bitfield) >> alr_shifted,
            transport
        });

        to_serve &= ~trip_traffic_day_bitfield;
        trace("┊ │    │  │ FOUND! transport={}, serving_on={} -> remaining={}\n", transport, trip_traffic_day_bitfield.to_string(), to_serve.to_string());
      }
    }
  }

  return any_marked;
}


void profile_raptor::update_footpaths(unsigned const k) {

  for (auto l_idx = location_idx_t{0U}; l_idx != tt_.n_locations(); ++l_idx) {
    if (!state_.station_mark_[to_idx(l_idx)]) {
      continue;
    }

    auto round_bag_copy = state_.round_bags_[k][to_idx(l_idx)];

    auto const fps = tt_.locations_.footpaths_out_[l_idx];

    for (auto const& fp : fps) {
      auto const target = to_idx(fp.target_);
      auto const fp_offset = fp.duration_ - tt_.locations_.transfer_time_[l_idx];

      for (auto it = round_bag_copy.begin(); it != round_bag_copy.end(); ++it) {
        (*it).arrival_ += fp_offset;
      }
      const auto res = state_.round_bags_[k][target].merge(round_bag_copy);
      if (std::any_of(res.begin(), res.end(), [](const auto e){return e.first;})) {
        state_.station_mark_[target] = true;
      }

    }
  }

}

void profile_raptor::get_earliest_sufficient_transports(const raptor_label label,
                                                        route_idx_t const r,
                                                        unsigned const stop_idx,
                                                        std::vector<transport>& transports) {
  auto arr_time = label.arrival_;
  auto td_bitfield = label.traffic_day_bitfield_;
  day_idx_t day_offset{0};

  auto dep_upper_limit = std::min(
      label.departure_ + minutes_after_midnight_t{kMaxTravelTime},
      1_days * static_cast<int16_t>(number_of_days_in_search_interval().v_)
      );

  auto const stop_event_times = tt_.event_times_at_stop(
      r, stop_idx, event_type::kDep);

  bool lower_bound_found = false;
  while (dep_upper_limit >= minutes_after_midnight_t::zero() && td_bitfield.any()) {

    auto const time_range_to_scan = it_range{
        lower_bound_found ? stop_event_times.begin() : std::lower_bound(
                                                         stop_event_times.begin(),
                                                         stop_event_times.end(), arr_time,
                                                         [&](auto&& a, auto&& b) { return a < b; }),
        stop_event_times.end()};


    if (!time_range_to_scan.empty()) {
      auto const base =
          static_cast<unsigned>(&*time_range_to_scan.begin_ - stop_event_times.data());
      for (auto const [t_offset, ev] : utl::enumerate(time_range_to_scan)) {

        if (ev > dep_upper_limit || !td_bitfield.any()) {
          return;
        }

        auto const t = tt_.route_transport_ranges_[r][base + t_offset];
        auto trip_traffic_day_bitfield = dynamic_bitfield{
          tt_.bitfields_[tt_.transport_traffic_days_[t]] >> this->start_day_offset().v_,
          this->number_of_days_in_search_interval().v_
        };


        if (!trip_traffic_day_bitfield.any()) continue;

        const auto new_td_bitfield = td_bitfield & (~trip_traffic_day_bitfield);
        if ((new_td_bitfield ^ td_bitfield).any()) {
          td_bitfield = new_td_bitfield;
          transports.push_back({
              t,
              day_offset
          });
        }
      }

      lower_bound_found = true;
    }

    dep_upper_limit = dep_upper_limit - 1_days;
    arr_time = arr_time - 1_days;
    td_bitfield <<= 1U;
    day_offset++;
  }
}

void profile_raptor::force_print_state(const char* comment) {
  auto const empty_rounds = [&](std::uint32_t const l) {
    for (auto k = 0U; k != end_k(); ++k) {
      if (state_.round_bags_[k][l].size() != 0) {
        return false;
      }
    }
    return true;
  };

  fmt::print("PROFILE RAPTOR STATE INFO: {}\n", comment);

  for (auto l = 0U; l != tt_.n_locations(); ++l) {
    if (!is_special(location_idx_t{l}) && !state_.is_destination_[l] &&
        state_.best_bag_[l].size() == 0 && empty_rounds(l)) {
      continue;
    }

    std::string_view name, track;
    auto const type = tt_.locations_.types_.at(location_idx_t{l});
    if (type == location_type::kTrack) {
      name =
          tt_.locations_.names_.at(tt_.locations_.parents_[location_idx_t{l}])
              .view();
      track = tt_.locations_.names_.at(location_idx_t{l}).view();
    } else {
      name = tt_.locations_.names_.at(location_idx_t{l}).view();
      track = "---";
    }
    tt_.locations_.names_[location_idx_t{l}].view();
    auto const id = tt_.locations_.ids_[location_idx_t{l}].view();
    fmt::print(
        "[{}] {:8} [name={:48}, track={:10}, id={:16}]: ",
        state_.is_destination_[l] ? "X" : "_", l, name, track,
        id.substr(0, std::min(std::string_view ::size_type{16U}, id.size())));
    auto const b = state_.best_bag_[l];
    if (b.size() == 0) {
      fmt::print("best=_________, round_times: ");
    } else {
      fmt::print("best={:9}, round_times: ", b.size());
    }
    for (auto i = 0U; i != kMaxTransfers + 1U; ++i) {
      auto const rb = state_.round_bags_[i][l];
      if (rb.size() > 0) {
        fmt::print("{:9} ", rb.size());
      } else {
        fmt::print("_________ ");
      }
    }
    fmt::print("\n");
  }
}


void profile_raptor::reconstruct() {
  state_.results_.resize(
      std::max(state_.results_.size(), state_.destinations_.size()));

  const auto r_max = kMaxTransfers + 1;
  matrix<std::vector<dep_arr_t>> expanded_round_times =
      cista::raw::make_matrix<std::vector<dep_arr_t>>(r_max, tt_.n_locations());

  matrix<std::vector<dep_arr_t>::iterator> round_time_iter =
      cista::raw::make_matrix<std::vector<dep_arr_t>::iterator>(r_max, tt_.n_locations());

  const auto dep_arr_cmp = [](const dep_arr_t& el1, const dep_arr_t& el2) {
    if (el1.first > el2.first) {
      return true;
    } else if (el1.first < el2.first) {
      return false;
    }

    if (el1.second < el2.second) {
      return true;
    }

    return false;
  };

  for (auto r = 0U; r < r_max; ++r) {
    for (auto l = 0U; l < tt_.n_locations(); ++l) {
      const auto& bag = state_.round_bags_[r][l];
      auto& ert = expanded_round_times[r][l];

      for (auto it = bag.begin(); it != bag.end(); it++) {
        const auto& resolved = it->resolve_label(to_idx(start_day_offset()));
        ert.insert(ert.end(), resolved.begin(), resolved.end());
      }

      //1.sort round times here
      std::sort(ert.begin(), ert.end(), dep_arr_cmp);
      //2.Initialize iterator to first element
      round_time_iter[r][l] = ert.begin();
    }
  }

  const auto dep_time_cmp = [](const routing_time& rt1, const routing_time& rt2) {
    if (rt1 > rt2) {
      return true;
    }

    return false;
  };

  std::set<routing_time, decltype(dep_time_cmp)> dep_times(dep_time_cmp);
  for (auto l = 0U; l < tt_.n_locations(); ++l) {
    const auto& dep_arr_times = expanded_round_times[0][l];
    for (const auto& dat : dep_arr_times) {
      dep_times.insert(dat.first);
    }
  }

  for (const auto& dep_time : dep_times) {
    search_state ss;
    ss.reset(tt_, kInvalidTime<direction::kForward>);

    for (auto r = 0U; r < r_max; ++r) {
      for (auto l = 0U; l < tt_.n_locations(); ++l) {
        auto& iter = round_time_iter[r][l];

        while (iter != expanded_round_times[r][l].end() && iter->first > dep_time) {
          iter++;
        }

        if (iter != expanded_round_times[r][l].end() && iter->first == dep_time) {
          ss.round_times_[r][l] = iter->second;
        }
      }
    }
    ss.results_.resize(
        std::max(state_.results_.size(), state_.destinations_.size()));
    for (auto const [i, t] : utl::enumerate(q_.destinations_)) {
      for (auto const dest : state_.destinations_[i]) {
        reconstruct_for_destination( i, dest, ss, dep_time.to_unixtime(tt_));
      }

      state_.results_[i].merge(ss.results_[i]);
    }
  }
}


void profile_raptor::reconstruct_for_destination(std::size_t dest_idx,
                                                 location_idx_t dest,
                                                 search_state& state,
                                                 const unixtime_t start_at_start) {
  for (auto k = 1U; k != end_k(); ++k) {
    if (state.round_times_[k][to_idx(dest)] == kInvalidTime<direction::kForward>) {
      continue;
    }
    auto const [optimal, it] = state.results_[dest_idx].add(journey{
        .legs_ = {},
        .start_time_ = start_at_start,
        .dest_time_ = state.round_times_[k][to_idx(dest)].to_unixtime(tt_),
        .dest_ = dest,
        .transfers_ = static_cast<std::uint8_t>(k - 1)});
    if (optimal) {
        try {
          reconstruct_journey<direction::kForward>(tt_, q_, state, *it);
        } catch (std::exception const& e) {
          state.results_[dest_idx].erase(it);
          log(log_lvl::error, "routing", "reconstruction failed: {}", e.what());
          print_state("RECONSTRUCT FAILED");
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
