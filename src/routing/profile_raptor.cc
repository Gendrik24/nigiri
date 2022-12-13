#include "nigiri/routing/profile_raptor.h"

#include <string>

#include "nigiri/routing/search_state.h"
#include "nigiri/routing/start_times.h"
#include "nigiri/types.h"
#include "nigiri/timetable.h"

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

void init_location_with_offset(timetable const& tt,
                               minutes_after_midnight_t time_to_arrive,
                               location_idx_t location,
                               day_idx_t n_days_in_search_interval,
                               cista::raw::matrix<raptor_bag>& round_bags_) {
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
        bitfield traffic_day_bitfield{std::string(n_days_in_search_interval.v_, '1')};
        traffic_day_bitfield >>= day_offset;

        round_bags_[0U][to_idx(location)]
          .add(raptor_label{
                at_stop,
                start_time,
                traffic_day_bitfield
            });
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
                              state_.round_bags_);

    // It is possible to transfer to another location and start from there
    if (q_.use_start_footpaths_) {

      // First get all outgoing footpaths
      auto const footpaths_out = tt_.locations_.footpaths_out_[loc];
      for (const auto& fp : footpaths_out) {

        init_location_with_offset(tt_,
                                  o.duration_ + fp.duration_,
                                  fp.target_,
                                  number_of_days_in_search_interval(),
                                  state_.round_bags_);
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

}

}
