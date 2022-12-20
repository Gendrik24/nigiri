#pragma once

// #define NIGIRI_RAPTOR_TRACING
// #define NIGIRI_RAPTOR_TRACING_ONLY_UPDATES

#ifdef NIGIRI_RAPTOR_TRACING

#ifdef NIGIRI_RAPTOR_TRACING_ONLY_UPDATES
#define trace(...)
#else
#define trace(...) fmt::print(__VA_ARGS__)
#endif

#define trace_always(...) fmt::print(__VA_ARGS__)
#define trace_upd(...) fmt::print(__VA_ARGS__)
#else
#define trace(...)
#define trace_always(...)
#define trace_upd(...)
#endif