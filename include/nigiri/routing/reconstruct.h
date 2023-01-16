#pragma once

#include "nigiri/types.h"
#include "nigiri/routing/routing_time.h"
#include "nigiri/types.h"

namespace nigiri {
struct timetable;
}

namespace nigiri::routing {

struct query;
struct search_state;
struct journey;

template <direction SearchDir>
void reconstruct_journey(timetable const&,
                         query const&,
                         journey&,
                         std::vector<routing_time> const&,
                         matrix<routing_time> const&);

}  // namespace nigiri::routing
