#pragma once

#include <cstdio>

// Lightweight logging shim to keep the solver self contained.
#define ELITE_LOG_ERROR(fmt, ...) \
  std::fprintf(stderr, "[elite_kinematics][error] " fmt "\n", ##__VA_ARGS__)

