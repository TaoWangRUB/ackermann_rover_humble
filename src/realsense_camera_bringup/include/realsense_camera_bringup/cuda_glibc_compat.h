#pragma once

// Jetson + Ubuntu 24.04 + CUDA 11.4 hits a glibc/libstdc++ interaction during
// nvcc compiler detection: bits/math-vector.h exposes AArch64 vector typedefs
// that cicc cannot parse. Pull in features.h first, then neutralize the glibc
// version test so those declarations stay disabled for CUDA compilation.
#if defined(__aarch64__)
#include <features.h>

#ifdef __GNUC_PREREQ
#undef __GNUC_PREREQ
#define __GNUC_PREREQ(maj, min) 0
#endif
#endif
