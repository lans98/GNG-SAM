#ifndef GNG_SAM_RANDOM_HPP
#define GNG_SAM_RANDOM_HPP

#include <random>

namespace gng {
namespace random {

  using namespace std;

  // generate a random number in the closed interval [beg, end]
  double randomIn(double beg, double end) {
    random_device rand_dev;
    mt19937_64 engine(rand_dev());
    uniform_real_distribution<> distribution(beg, end);
    return distribution(engine);
  }

  // generate a random number in [0,1]
  double random() { return randomIn(0.0, 1.0); }

}
}

#endif
