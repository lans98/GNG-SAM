#include <memory>
#include <gng_sam.hpp>

using namespace std;
using namespace gng;

int main() {
  auto growingNeuralGas = make_unique<GNG<3>>(-0.5);
  growingNeuralGas->start(1000, 5, 0.1, 20);
}
