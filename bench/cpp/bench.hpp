// Timing harness shared by every C++ benchmark program (the Python twin is py/pybench.py).
//
//   prog time <warmup> <min_runs> <max_runs> <target_s>   build in-process, print one JSON line
//   prog run <out.step>                                    build once, export one STEP, print JSON
//
// A model is a function returning the named shapes it builds. Its result is destroyed outside
// the timed region, as the Python harness does.
#pragma once

#include <build123d/build123d.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace bench {

using Shapes = std::vector<std::pair<std::string, build123d::Shape>>;
using clk = std::chrono::steady_clock;

inline double ms(clk::time_point a, clk::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
}

inline std::string num(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.12g", v);
    return buf;
}

inline std::string metrics_json(const Shapes& shapes) {
    std::string out = "[";
    for (std::size_t i = 0; i < shapes.size(); ++i) {
        const auto& [name, s] = shapes[i];
        out += (i ? ", " : "");
        out += "{\"name\": \"" + name + "\", \"volume\": " + num(s.volume()) + ", \"area\": " + num(s.area()) +
               ", \"faces\": " + std::to_string(s.faces().size()) + "}";
    }
    return out + "]";
}

/// Times `build` in-process: `warmup` untimed runs, then runs until both `min_runs` runs and
/// `target_s` seconds are reached, or `max_runs` runs. The same rule as pybench.py.
/// `prepare` runs before each build, outside the timed region.
inline int time_mode(int argc, char** argv, const std::function<Shapes()>& build,
                     const std::function<void()>& prepare = {}) {
    const int warmup = argc > 2 ? std::atoi(argv[2]) : 2;
    const int min_runs = argc > 3 ? std::atoi(argv[3]) : 10;
    const int max_runs = argc > 4 ? std::atoi(argv[4]) : 200;
    const double target_ms = 1000.0 * (argc > 5 ? std::atof(argv[5]) : 2.0);
    Shapes last;
    std::vector<double> warm_times;
    for (int i = 0; i < warmup; ++i) {
        if (prepare) prepare();
        const auto t0 = clk::now();
        Shapes r = build();
        const auto t1 = clk::now();
        warm_times.push_back(ms(t0, t1));
        last = std::move(r);
    }
    std::vector<double> times;
    double total = 0;
    while (static_cast<int>(times.size()) < max_runs && (static_cast<int>(times.size()) < min_runs || total < target_ms)) {
        if (prepare) prepare();
        Shapes r;
        const auto t0 = clk::now();
        r = build();
        const auto t1 = clk::now();
        times.push_back(ms(t0, t1));
        total += times.back();
        last = std::move(r);  // the previous result is destroyed here, outside the timed region
    }
    std::string t = "[";
    for (std::size_t i = 0; i < times.size(); ++i) t += (i ? ", " : "") + num(times[i]);
    t += "]";
    std::string w = "[";
    for (std::size_t i = 0; i < warm_times.size(); ++i) w += (i ? ", " : "") + num(warm_times[i]);
    w += "]";
    std::cout << "{\"lang\": \"cpp\", \"warmup_ms\": " << w << ", \"times_ms\": " << t
              << ", \"shapes\": " << metrics_json(last) << "}" << std::endl;
    return 0;
}

/// Builds once and exports all the shapes as one STEP file. Nothing else: no metrics, as in
/// pybench.py's run mode (the whole run is the program's start, the build and the export).
inline int run_mode(int argc, char** argv, const std::function<Shapes()>& build) {
    const std::string out = argc > 2 ? argv[2] : "out.step";
    const Shapes shapes = build();
    std::vector<build123d::Shape> all;
    for (const auto& [name, s] : shapes) all.push_back(s);
    const build123d::Shape to_export = all.size() == 1 ? all[0] : build123d::Shape(build123d::Compound(all));
    build123d::export_step(to_export, out);
    std::cout << "{\"lang\": \"cpp\"}" << std::endl;
    return 0;
}

inline int main_for(int argc, char** argv, const std::function<Shapes()>& build,
                    const std::function<void()>& prepare = {}) {
    const std::string mode = argc > 1 ? argv[1] : "run";
    try {
        if (mode == "time") return time_mode(argc, argv, build, prepare);
        if (mode == "run") return run_mode(argc, argv, build);
        if (mode == "start") return 0;  // the program's start alone
    } catch (const std::exception& e) {
        std::cout << "{\"lang\": \"cpp\", \"error\": " << '"' << e.what() << '"' << "}" << std::endl;
        return 1;
    }
    std::cerr << "unknown mode " << mode << "\n";
    return 2;
}

}  // namespace bench

/// A program whose model is `fn`, a function returning one Shape.
#define BENCH_MODEL(name_literal, fn)                                                                    \
    int main(int argc, char** argv) {                                                                 \
        return bench::main_for(argc, argv,                                                            \
                               [] { return bench::Shapes{{name_literal, build123d::Shape(fn())}}; }); \
    }
