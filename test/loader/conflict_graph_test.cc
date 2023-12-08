#include <cstddef>
#include "nigiri/loader/dir.h"
#include "gtest/gtest.h"

#include "date/date.h"
#include "nigiri/loader/gtfs/load_timetable.h"
#include "nigiri/loader/init_finish.h"
#include "gtfs/test_data.h"
#include "nigiri/types.h"
#include "nigiri/timetable.h"

using namespace date;
using namespace nigiri;
using namespace nigiri::loader;
using namespace nigiri::loader::gtfs;

bool is_disjunct(const route_idx_t r1,
                 const route_idx_t r2,
                 const timetable& tt) {
  
  const auto& r1_stop_seq = tt.route_location_seq_[r1];
  const auto& r2_stop_seq = tt.route_location_seq_[r2];
  for (auto i = 0U; i != r1_stop_seq.size(); ++i) {
    const auto stp_1 = stop{r1_stop_seq[i]};
    for (auto j = 0U; j != r2_stop_seq.size(); ++j) {
      const auto stp_2 = stop{r2_stop_seq[j]};
      if (stp_1.location_idx() == stp_2.location_idx()) {
        return false;
      }
    }
  }

  return true;
}

bool conflict_groups_disjunct(const timetable& tt) {
  for (std::size_t i = 0U; i < tt.conflict_groups_last_index_.size(); ++i) {
    for (std::size_t r1 = (i == 0U) ? 0U : tt.conflict_groups_last_index_[i-1];
         r1 < tt.conflict_groups_last_index_[i]; ++r1) {
      for (std::size_t r2 = r1 + 1; r2 < tt.conflict_groups_last_index_[i]; ++r2) {
        if (!is_disjunct(tt.routes_conflict_ordered_[r1],
                                tt.routes_conflict_ordered_[r2],
                                tt)) {
          return false;
        }
      }
    } 
  }
  return true;
}

TEST(loader, conflict_graph_test_1) {
  const auto files = example_files();

  auto tt = timetable{};
  tt.date_range_ = {2006_y / 7 / 1, 2006_y / 8 / 1};
  load_timetable({},source_idx_t{0U}, files, tt);
  finalize(tt);
  
  EXPECT_TRUE(conflict_groups_disjunct(tt));
}


TEST(loader, conflict_graph_test_2) {
  const auto files = fs_dir{"test/test_data/transit-10-15"};

  auto tt = timetable{};
  tt.date_range_ = {2023_y / 10 / 1, 2023_y / 11 / 1};
  load_timetable({},source_idx_t{0U}, files, tt);
  finalize(tt);
  
  EXPECT_TRUE(conflict_groups_disjunct(tt));
}
