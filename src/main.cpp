#include <memory>
#include <gng_sam.hpp>

using namespace std;
using namespace gng;
using namespace gng::random;

void test3D() {
    auto growingNeuralGas = make_unique<GNG<3>>(-0.5);
    DataRange<3> dataRange; 
    dataRange.setMins(array<double, 3>({0,0,0}));
    dataRange.setMaxs(array<double, 3>({2,2,2}));
    dataRange.setBoundFunction([](auto p) -> bool { return true; });
    growingNeuralGas->addDataRange(dataRange);

    dataRange.setMins(array<double, 3>({1,1,1}));
    dataRange.setMaxs(array<double, 3>({5,5,5}));
    dataRange.setBoundFunction([](auto p) -> bool { return true; });
    growingNeuralGas->addDataRange(dataRange);

    growingNeuralGas->start(1000, 5, 0.1, 20);
}

void test2D() {
    auto gng = make_unique<GNG<2>>(0.2);
    DataRange<2> dataRange;
    dataRange.setMins(array<double, 2>({ -1 , -1 }));
    dataRange.setMaxs(array<double, 2>({  1 ,  1 }));
    dataRange.setBoundFunction([](auto& p) -> bool { 
        double x = p[X];
        bool neg = random::random() > 0.5;

        p[Y] = sqrt(1 - pow(x,2));
        p[Y] = neg ? -p[Y] : p[Y];
        return true;
    });
    gng->addDataRange(dataRange);

    dataRange.setMins(array<double, 2>({ -5 , -5 }));
    dataRange.setMaxs(array<double, 2>({ -3 , -3 }));
    dataRange.setBoundFunction([](auto& p) -> bool { 
        double x = p[X];
        bool neg = random::random() > 0.5;
        double d = sqrt(1 - pow(x + 4,2));

        p[Y] = neg? - d - 4: d - 4; 
        return true;
    });
    gng->addDataRange(dataRange);


    gng->start(1000, 2, 0.1, 5);
}

int main() {
    test2D();
}
