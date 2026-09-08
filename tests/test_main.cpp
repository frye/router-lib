#include "test_support.hpp"

int main(int argc, char** argv) {
    int failures = 0;
    int selected = 0;
    for (const auto& test : sailroute::test::registry()) {
        bool matches = argc <= 1;
        for (int index = 1; index < argc; ++index) {
            matches = matches || test.name.find(argv[index]) != std::string::npos;
        }
        if (!matches) continue;
        ++selected;
        try {
            test.function();
            std::cout << "[pass] " << test.name << '\n';
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "[fail] " << test.name << ": " << error.what() << '\n';
        }
    }
    if (selected == 0) {
        std::cerr << "no test cases matched\n";
        return 2;
    }
    return failures == 0 ? 0 : 1;
}
