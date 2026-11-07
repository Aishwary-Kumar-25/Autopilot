#pragma once
/**
 * Minimal test helpers. Unlike assert(), CHECK is NOT compiled out by -DNDEBUG,
 * so tests still test something in Release builds. A failing CHECK prints the
 * location and makes the test exit non-zero, but keeps running so one run
 * shows every failure.
 */
#include <cmath>
#include <cstdio>

namespace test {
inline int& failures() { static int f = 0; return f; }
inline int& passes() { static int p = 0; return p; }
inline int finish(const char* name) {
    std::printf("%s: %d checks passed, %d failed\n", name, passes(), failures());
    return failures() == 0 ? 0 : 1;
}
} // namespace test

#define CHECK(cond)                                                                       \
    do {                                                                                  \
        if (cond) { ++test::passes(); }                                                   \
        else { ++test::failures(); std::printf("CHECK FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond); } \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                             \
    do {                                                                                  \
        const double a_ = (a), b_ = (b);                                                  \
        if (std::fabs(a_ - b_) <= (tol)) { ++test::passes(); }                            \
        else { ++test::failures(); std::printf("CHECK FAILED %s:%d: %s = %g, expected %g (tol %g)\n", \
                                               __FILE__, __LINE__, #a, a_, b_, (double)(tol)); } \
    } while (0)
