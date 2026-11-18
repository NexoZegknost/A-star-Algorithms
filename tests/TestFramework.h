#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace test {

typedef void (*TestFn)();

struct TestCase {
    std::string name;
    TestFn fn;
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> cases;
    return cases;
}

inline int& failures() {
    static int count = 0;
    return count;
}

struct Registrar {
    Registrar(const std::string& name, TestFn fn) {
        TestCase testCase = {name, fn};
        registry().push_back(testCase);
    }
};

inline void report(bool ok, const char* expr, const char* file, int line) {
    if (!ok) {
        ++failures();
        std::cerr << "  FAILED " << expr << " at " << file << ":" << line << "\n";
    }
}

}

#define TEST_CASE(name)                                         \
    static void name();                                         \
    static test::Registrar name##_registrar(#name, &name);      \
    static void name()

#define CHECK(expr) test::report((expr), #expr, __FILE__, __LINE__)
#define CHECK_NEAR(a, b) test::report(std::fabs((a) - (b)) < 1e-6, #a " ~= " #b, __FILE__, __LINE__)

#endif
