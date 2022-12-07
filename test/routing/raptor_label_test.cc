#include "doctest/doctest.h"

#include "nigiri/routing/raptor_label.h"
#include "nigiri/types.h"

using namespace nigiri;
using namespace nigiri::routing;

TEST_CASE("raptor-label") {
    raptor_label label_1(5_hours, 4_hours, bitfield{"1100000"});
    raptor_label label_2(4_hours + 30_minutes, 4_hours + 10_minutes, bitfield{"1111111"});
    raptor_label label_3(4_hours + 30_minutes, 4_hours, bitfield{"1111111"});
    raptor_label label_4(5_hours, 4_hours, bitfield{"0011111"});

    raptor_label label_5(30_hours, 16_hours, bitfield{"0011111"});
    raptor_label label_6(30_hours, 16_hours, bitfield{"0000110"});
    CHECK_EQ(label_1.dominates(label_2), false);
    CHECK_EQ(label_2.dominates(label_1), true);
    CHECK_EQ(label_3.dominates(label_1), true);
    CHECK_EQ(label_1.dominates(label_1), false);
    CHECK_EQ(label_1.dominates(label_4), false);
    CHECK_EQ(label_5.dominates(label_6), true);
    CHECK_EQ(label_6.dominates(label_5), false);
};
