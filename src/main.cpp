#include <memory>
#include <gng_sam.hpp>

using namespace std;
using namespace gng;

int main() {
  auto growing_neural_gas = make_unique<GNG<2>>(-0.5);
  growing_neural_gas->set_maximum_edge_age(10);
  growing_neural_gas->start(10, 5, 0.1);
}
