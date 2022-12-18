#include "nigiri/routing/profile_raptor.h"

#include <string>
#include <algorithm>

#include "nigiri/routing/search_state.h"
#include "nigiri/routing/start_times.h"
#include "nigiri/types.h"
#include "nigiri/timetable.h"
#include "nigiri/dynamic_bitfield.h"

#include "utl/overloaded.h"
#include "utl/enumerate.h"

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
  for (auto k = 1U; k != end_k(); ++k) {
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
      return;
    }

    any_marked = false;
    for (auto r_id = 0U; r_id != tt_.n_routes(); ++r_id) {
      if (!state_.route_mark_[r_id]) {
        continue;
      }
      any_marked |= update_route(k, route_idx_t{r_id});
    }

    std::fill(begin(state_.route_mark_), end(state_.route_mark_), false);
    if (!any_marked) {
      return;
    }

    update_footpaths(k);
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

    // 1. Iterate over Labels in Route Bag and update arrival times according to assigned transport
    for (auto it = r_b.begin(); it != r_b.end(); ++it) {
      auto& active_label = *it;

      if (active_label.t_.t_idx_ == transport_idx_t::invalid()) {
        continue;
      }

      const auto new_arr = tt_.event_mam(route_idx,
                                         active_label.t_.t_idx_,
                                         stop_idx,
                                         event_type::kArr) + minutes_after_midnight_t{active_label.t_.day_.v_ * 1440};

      active_label.arrival_ = new_arr;
    }

    // 2. Merge Route Bag into round bag
    const auto resp = state_.round_bags_[k][cista::to_idx(l_idx)].merge(r_b);
    any_marked = std::any_of(resp.begin(),
                             resp.end(),
                             [](const auto e){return e.first;});

    // 3. Assign Trips to labels from previous round and merge them into route bag
    const raptor_bag& prev_round = state_.round_bags_[k-1][cista::to_idx(l_idx)];
    for (const auto& l : prev_round) {
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
      }
    }
  }

  return any_marked;
}


void profile_raptor::update_footpaths(unsigned const k) {

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

}
