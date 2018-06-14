#include <memory>
#include <gng_sam.hpp>

using namespace std;
using namespace gng;

int main() {
  auto growingNeuralGas = make_unique<GNG<2>>(-0.5);
  growingNeuralGas->start(10, 5, 0.1, 20);
}
