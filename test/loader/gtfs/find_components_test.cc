#include "gtest/gtest.h"

#include "date/date.h"

#include "nigiri/loader/gtfs/agency.h"
#include "nigiri/loader/gtfs/files.h"
#include "nigiri/loader/gtfs/load_timetable.h"
#include "nigiri/loader/build_footpaths.h"
#include "nigiri/location.h"
#include "nigiri/routing/dijkstra.h"
#include "nigiri/routing/query.h"
#include "nigiri/timetable.h"
#include "nigiri/types.h"
#include "./test_data.h"
#include "nigiri/loader/init_finish.h"
#include "./test_data.h"

#include "fmt/core.h"

using namespace date;
using namespace nigiri;
using namespace nigiri::routing;
using namespace nigiri::loader;
using namespace nigiri::loader::gtfs;

TEST(gtfs, find_components_test_1) {
    const auto files = example_files();

    auto tt = timetable{};
    tt.date_range_ = {2006_y / 7 / 1, 2006_y / 8 / 1};
    load_timetable({}, source_idx_t{0U}, files, tt);
    finalize(tt);

    auto q = query{
      .destination_ = {
          {location_idx_t{7U}, 0_minutes, 0U}}};

    std::vector<lower_bound> lb;
    dijkstra(tt, q, tt.transfers_lb_graph_, lb);

    ASSERT_EQ(
        tt.locations_.next_component_idx_,
        component_idx_t{6}
    );

    ASSERT_EQ(
        tt.locations_.components_[location_idx_t{5}],
        tt.locations_.components_[location_idx_t{6}]
    );

        ASSERT_EQ(
        tt.locations_.components_[location_idx_t{6}],
        tt.locations_.components_[location_idx_t{7}]
    );

    for (auto l = location_idx_t{0}; l < tt.n_locations(); ++l) {
        const auto lower_bound = lb[to_idx(l)];
        if (tt.locations_.components_[location_idx_t{7}] == tt.locations_.components_[l]) {
            ASSERT_EQ(
                lower_bound.transports_,
                0U
            );
        } else {
            ASSERT_EQ(
                lower_bound.transports_,
                1U
            );
        }
    }
}

TEST(gtfs, find_components_test_2) {
    const auto files = example_2_files();

    auto tt = timetable{};
    tt.date_range_ = {2006_y / 7 / 1, 2006_y / 8 / 1};
    load_timetable({}, source_idx_t{0U}, files, tt);
    finalize(tt);

    auto q = query{
      .destination_ = {
          {location_idx_t{9U}, 0_minutes, 0U}}};


    std::vector<lower_bound> lb;
    dijkstra(tt, q, tt.transfers_lb_graph_, lb);

    ASSERT_EQ(
        tt.locations_.next_component_idx_,
        component_idx_t{8}
    );

    const vector<lower_bound::value_type> expected_transports = {
        1U, 2U, 2U, 2U, 2U, 2U, 2U, 2U, 1U, 0U 
    };

    for (auto i = 0U; i<tt.n_locations(); ++i) {
        ASSERT_EQ(
            lb[to_idx(location_idx_t{i})].transports_,
            expected_transports[i]
        );
    }
}

TEST(gtfs, find_components_test_3) {
    const auto files = components_files();

    auto tt = timetable{};
    tt.date_range_ = {2006_y / 7 / 1, 2006_y / 8 / 1};
    load_timetable({}, source_idx_t{0U}, files, tt);
    finalize(tt);

    auto q = query{
      .destination_ = {
          {location_idx_t{9U}, 0_minutes, 0U}}};


    std::vector<lower_bound> lb;
    dijkstra(tt, q, tt.transfers_lb_graph_, lb);

    ASSERT_EQ(
        tt.locations_.next_component_idx_,
        component_idx_t{8}
    );

        const vector<component_idx_t::value_t> expected_component = {
        0U, 1U, 2U, 3U, 4U, 5U, 5U, 5U, 6U, 6U , 7U, 7U, 7U
    };

    for (auto i = 0U; i<tt.n_locations(); ++i) {
        ASSERT_EQ(
            tt.locations_.components_[location_idx_t{i}],
            component_idx_t{expected_component[i]}
        );
    }
}