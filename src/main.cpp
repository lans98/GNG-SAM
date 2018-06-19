#include <memory>
#include <gng_sam.hpp>

using namespace std;
using namespace gng;

int main() {
    auto growingNeuralGas = make_unique<GNG<3>>(-0.5);
    DataRange<3> dataRange; 
    dataRange.setMins({ 0, 0 });
    dataRange.setMaxs({ 1, 1 });
    dataRange.setBoundFunction([](auto p) -> bool { return true; });

    growingNeuralGas->addDataRange(dataRange);
    growingNeuralGas->start(1000, 5, 0.1, 20);
}
