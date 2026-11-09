// A minimal self-registering test framework shared by tests/test_*.cpp (main() is in
// test_main.cpp). Run: build123d_tests [test-name ...]
#pragma once

#include <cmath>
#include <exception>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace testing {

struct TestCase {
    const char* name;
    void (*body)();
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

struct Register {
    Register(const char* name, void (*body)()) { registry().push_back({name, body}); }
};

struct Failure : std::exception {
    std::string message;
    explicit Failure(std::string m) : message(std::move(m)) {}
    const char* what() const noexcept override { return message.c_str(); }
};

inline std::string where(const char* file, int line) {
    return std::string(file) + ":" + std::to_string(line) + ": ";
}

}  // namespace testing

#define TEST(name)                                                       \
    static void test_##name();                                           \
    static const testing::Register register_##name(#name, test_##name); \
    static void test_##name()

#define CHECK(cond)                                                                                     \
    do {                                                                                                \
        if (!(cond)) throw testing::Failure(testing::where(__FILE__, __LINE__) + "CHECK(" #cond ") failed"); \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                          \
    do {                                                                                               \
        const double check_a = (a), check_b = (b);                                                     \
        if (!(std::abs(check_a - check_b) <= (tol))) {                                                 \
            std::ostringstream check_os;                                                               \
            check_os.precision(17);                                                                    \
            check_os << testing::where(__FILE__, __LINE__) << #a " = " << check_a << ", expected " << check_b; \
            throw testing::Failure(check_os.str());                                                    \
        }                                                                                              \
    } while (0)

#define CHECK_THROWS(expr, type)                                                                    \
    do {                                                                                            \
        bool check_thrown = false;                                                                  \
        try {                                                                                       \
            (void)(expr);                                                                           \
        } catch (const type&) {                                                                     \
            check_thrown = true;                                                                    \
        }                                                                                           \
        if (!check_thrown) {                                                                        \
            throw testing::Failure(testing::where(__FILE__, __LINE__) + #expr " did not throw " #type); \
        }                                                                                           \
    } while (0)
