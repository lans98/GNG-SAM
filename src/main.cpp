#include <memory>
#include <gng_sam.hpp>

using namespace std;
using namespace gng;

int main() {
  auto growing_neural_gas = make_unique<GNG<2>>();
  growing_neural_gas->start(10, 5);
}
