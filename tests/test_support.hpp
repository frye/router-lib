#pragma once

#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace sailroute::test {

struct TestCase {
    std::string name;
    std::function<void()> function;
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

class Registrar {
public:
    Registrar(std::string name, std::function<void()> function) {
        registry().push_back(TestCase{std::move(name), std::move(function)});
    }
};

inline void require(bool condition, const char* expression, const char* file, int line) {
    if (!condition) {
        throw std::runtime_error(
            std::string{file} + ":" + std::to_string(line) + ": requirement failed: " +
            expression);
    }
}

inline void require_near(
    double actual,
    double expected,
    double tolerance,
    const char* file,
    int line) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(
            std::string{file} + ":" + std::to_string(line) + ": expected " +
            std::to_string(actual) + " near " + std::to_string(expected));
    }
}

}  // namespace sailroute::test

#define SAILROUTE_TEST_CONCAT_INNER(a, b) a##b
#define SAILROUTE_TEST_CONCAT(a, b) SAILROUTE_TEST_CONCAT_INNER(a, b)
#define TEST_CASE(name) \
    static void SAILROUTE_TEST_CONCAT(test_function_, __LINE__)(); \
    static ::sailroute::test::Registrar SAILROUTE_TEST_CONCAT(test_registrar_, __LINE__)( \
        name, SAILROUTE_TEST_CONCAT(test_function_, __LINE__)); \
    static void SAILROUTE_TEST_CONCAT(test_function_, __LINE__)()
#define REQUIRE(expression) \
    ::sailroute::test::require((expression), #expression, __FILE__, __LINE__)
#define REQUIRE_NEAR(actual, expected, tolerance) \
    ::sailroute::test::require_near((actual), (expected), (tolerance), __FILE__, __LINE__)
