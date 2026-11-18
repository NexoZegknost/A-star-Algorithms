#include "TestFramework.h"

int main() {
    const std::vector<test::TestCase>& cases = test::registry();
    for (size_t i = 0; i < cases.size(); ++i) {
        int before = test::failures();
        cases[i].fn();
        std::cout << (test::failures() == before ? "[PASS] " : "[FAIL] ") << cases[i].name << "\n";
    }
    std::cout << cases.size() << " tests, " << test::failures() << " failed checks\n";
    return test::failures() == 0 ? 0 : 1;
}
