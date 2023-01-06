#include "doctest/doctest.h"

#include "nigiri/loader/hrd/load_timetable.h"
#include "nigiri/routing/profile_raptor.h"
#include "nigiri/routing/raptor_label.h"
#include "nigiri/routing/search_state.h"
#include "nigiri/routing/routing_time.h"
#include "nigiri/routing/sorted_pareto_set.h"
#include "nigiri/routing/raptor_label.h"

#include "nigiri/timetable.h"
#include "nigiri/dynamic_bitfield.h"

#include <chrono>

#include "../loader/hrd/hrd_timetable.h"

using namespace nigiri;
using namespace nigiri::routing;
using namespace nigiri::test_data::hrd_timetable;

constexpr auto const fwd_journeys = R"(
[2020-03-30 05:30, 2020-03-30 07:45]
TRANSFERS: 1
     FROM: (A, 0000001) [2020-03-30 05:30]
       TO: (C, 0000003) [2020-03-30 07:45]
leg 0: (A, 0000001) [2020-03-30 05:30] -> (B, 0000002) [2020-03-30 06:30]
   0: 0000001 A...............................................                               d: 30.03 05:30 [30.03 07:30]  [{name=RE 1337, day=2020-03-30, id=1337/0000001/330/0000002/390/, src=0}]
   1: 0000002 B............................................... a: 30.03 06:30 [30.03 08:30]
leg 1: (B, 0000002) [2020-03-30 06:30] -> (B, 0000002) [2020-03-30 06:32]
  FOOTPATH (duration=2)
leg 2: (B, 0000002) [2020-03-30 06:45] -> (C, 0000003) [2020-03-30 07:45]
   0: 0000002 B...............................................                               d: 30.03 06:45 [30.03 08:45]  [{name=RE 7331, day=2020-03-30, id=7331/0000002/405/0000003/465/, src=0}]
   1: 0000003 C............................................... a: 30.03 07:45 [30.03 09:45]
leg 3: (C, 0000003) [2020-03-30 07:45] -> (C, 0000003) [2020-03-30 07:45]
  FOOTPATH (duration=0)


[2020-03-30 05:00, 2020-03-30 07:15]
TRANSFERS: 1
     FROM: (A, 0000001) [2020-03-30 05:00]
       TO: (C, 0000003) [2020-03-30 07:15]
leg 0: (A, 0000001) [2020-03-30 05:00] -> (B, 0000002) [2020-03-30 06:00]
   0: 0000001 A...............................................                               d: 30.03 05:00 [30.03 07:00]  [{name=RE 1337, day=2020-03-30, id=1337/0000001/300/0000002/360/, src=0}]
   1: 0000002 B............................................... a: 30.03 06:00 [30.03 08:00]
leg 1: (B, 0000002) [2020-03-30 06:00] -> (B, 0000002) [2020-03-30 06:02]
  FOOTPATH (duration=2)
leg 2: (B, 0000002) [2020-03-30 06:15] -> (C, 0000003) [2020-03-30 07:15]
   0: 0000002 B...............................................                               d: 30.03 06:15 [30.03 08:15]  [{name=RE 7331, day=2020-03-30, id=7331/0000002/375/0000003/435/, src=0}]
   1: 0000003 C............................................... a: 30.03 07:15 [30.03 09:15]
leg 3: (C, 0000003) [2020-03-30 07:15] -> (C, 0000003) [2020-03-30 07:15]
  FOOTPATH (duration=0)


)";

TEST_CASE("profile-raptor") {
  using namespace date;
  timetable tt;
  auto const src = source_idx_t{0U};
  load_timetable(src, loader::hrd::hrd_5_20_26, files_abc(), tt);
  auto profile_state = routing::profile_search_state{};
  auto state = routing::search_state{};

  routing::query q{
      .start_time_ =
          interval<unixtime_t>{
              unixtime_t{sys_days{2020_y / March / 28}} + 5_hours,
              unixtime_t{sys_days{2020_y / April / 1}} + 6_hours},
      .start_match_mode_ = nigiri::routing::location_match_mode::kExact,
      .dest_match_mode_ = nigiri::routing::location_match_mode::kExact,
      .use_start_footpaths_ = true,
      .start_ = {nigiri::routing::offset{
          tt.locations_.location_id_to_idx_.at(
              {.id_ = "0000001", .src_ = src}),
          0_minutes, 0U}},
      .destinations_ = {{nigiri::routing::offset{
          tt.locations_.location_id_to_idx_.at(
              {.id_ = "0000003", .src_ = src}),
          0_minutes, 0U}}},
      .via_destinations_ = {},
      .allowed_classes_ = bitset<kNumClasses>::max(),
      .max_transfers_ = 6U,
      .min_connection_count_ = 0U,
      .extend_interval_earlier_ = false,
      .extend_interval_later_ = false};

  auto fwdp_r = routing::profile_raptor{
      tt, profile_state, q
  };
  fwdp_r.route();

  std::stringstream ss_profile;
  ss_profile << "\n";
  for (auto const& x : fwdp_r.state_.results_.at(0)) {
    x.print(ss_profile, tt);
    ss_profile << "\n\n";
  }

  auto fwd_r = routing::raptor<direction::kForward, false>{
      tt, state, q
  };
  fwd_r.route();

  std::stringstream ss;
  ss << "\n";
  for (auto const& x : state.results_.at(0)) {
    x.print(ss, tt);
    ss << "\n\n";
  }

  CHECK_EQ(ss.str(), ss_profile.str());
};