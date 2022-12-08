#include "doctest/doctest.h"

#include "nigiri/routing/raptor_label.h"
#include "nigiri/routing/raptor_bag.h"
#include "nigiri/types.h"

using namespace nigiri;
using namespace nigiri::routing;
using namespace std;

bool verify_not_dominating(const raptor_bag& bag) {
  for (size_t i=0; i<bag.size(); ++i) {
    for (size_t j=0; j<bag.size(); ++j) {
      if (bag[i].dominates(bag[j]) || bag[j].dominates(bag[i])) {
        return false;
      }
    }
  }

  return true;
}

TEST_CASE("raptor-bag") {
  labels_t undominated_labels = {
      raptor_label(5_hours, 4_hours, bitfield{"1100000"}),
      raptor_label(5_hours + 30_minutes, 4_hours + 10_minutes, bitfield{"1111111"}),
      raptor_label(4_hours + 30_minutes, 3_hours, bitfield{"1111111"})
  };

  raptor_bag bag(undominated_labels);
  CHECK_EQ(bag.size(), 3UL);
  CHECK(verify_not_dominating(bag));

  bag.merge(raptor_label(5_hours, 4_hours, bitfield{"0011111"}));
  CHECK_EQ(bag.size(), 4UL);
  CHECK(verify_not_dominating(bag));

  bag.merge(raptor_label(4_hours + 30_minutes, 4_hours, bitfield{"1111111"}));
  CHECK_EQ(bag.size(), 2UL);
  CHECK(verify_not_dominating(bag));

  bag.clear();
  CHECK_EQ(bag.size(), 0UL);

  bag.merge(raptor_label(36_hours + 30_minutes, 18_hours, bitfield{"1101111"}));
  CHECK_EQ(bag.size(), 1UL);
  CHECK(verify_not_dominating(bag));

  bag.merge(raptor_label(35_hours, 17_hours, bitfield{"0010000"}));
  CHECK_EQ(bag.size(), 2UL);
  CHECK(verify_not_dominating(bag));

  bag.merge(raptor_label(35_hours, 17_hours, bitfield{"0110000"}));
  CHECK_EQ(bag.size(), 2UL);
  CHECK(verify_not_dominating(bag));
};