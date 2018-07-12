#include <iostream>

#include <data_ranges.hpp> 

using namespace std;
using namespace gng::data_ranges;

void test1() {
    array<double, 2> mins { 0, 0 };
    array<double, 2> maxs { 1, 1 };
    DataRange<2> dataRange2D(mins, maxs, [](auto p){ return true; });
    auto point = dataRange2D.genRandom();
    cout << point << '\n';
}

void test2() {
    array<double, 2> mins { -5, -5 };
    array<double, 2> maxs { -3, -3 };
    DataRange<2> dataRange2D(mins, maxs, [](auto p){ return true; });
    auto point = dataRange2D.genRandom();
    cout << point << '\n';
}

int main() {
    test2();
}
