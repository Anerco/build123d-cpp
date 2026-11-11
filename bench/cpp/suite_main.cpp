// Times the bd_warehouse comparison cases (tests/compare/cases_bdw_*.cpp, linked as they are)
// one by one, in place of compare_cases' main. The twin of py/suite.py.
//
//   suite_bdw <prefix> <warmup> <runs>
//
// For every case whose name starts with <prefix>: <warmup> untimed builds, then <runs> timed
// ones; prints one JSON line per case. What is timed is the case's function alone (the shape
// or the values it returns), not the metrics compare.py compares.
#include "case_registry.hpp"

#include <Standard_Failure.hxx>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    const std::string prefix = argc > 1 ? argv[1] : "bdw_";
    const int warmup = argc > 2 ? std::atoi(argv[2]) : 1;
    const int runs = argc > 3 ? std::atoi(argv[3]) : 1;
    using clk = std::chrono::steady_clock;
    for (const auto& e : compare::registry()) {
        if (e.name.rfind(prefix, 0) != 0) continue;
        std::vector<double> times;
        std::string error;
        double volume = 0;
        for (int i = 0; i < warmup + runs; ++i) {
            build123d::Shape shape;
            compare::V value;
            const auto t0 = clk::now();
            try {
                if (e.shape) {
                    shape = e.shape();
                } else {
                    value = e.value();
                }
            } catch (const Standard_Failure& ex) {
                error = ex.what();
            } catch (const std::exception& ex) {
                error = ex.what();
            }
            const auto t1 = clk::now();
            if (i >= warmup) times.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
            if (e.shape && error.empty() && i + 1 == warmup + runs) volume = shape.volume();
        }
        std::string t;
        for (std::size_t i = 0; i < times.size(); ++i) {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%s%.6g", i ? ", " : "", times[i]);
            t += buf;
        }
        char vol[32];
        std::snprintf(vol, sizeof(vol), "%.12g", volume);
        std::cout << "{\"name\": " << compare::V::quote(e.name) << ", \"kind\": \"" << (e.shape ? "shape" : "value")
                  << "\", \"times_ms\": [" << t << "], \"volume\": " << vol
                  << ", \"error\": " << (error.empty() ? "null" : compare::V::quote(error)) << "}" << std::endl;
    }
    return 0;
}
