#ifndef GNG_SAM_RANDOM_HPP
#define GNG_SAM_RANDOM_HPP

#include <random>

namespace gng::random {

  using namespace std;

  // generate a random number in the closed interval [beg, end]
  double random(double beg = 0.0, double end = 1.0) {
    random_device rand_dev;
    mt19937_64 engine(rand_dev());
    uniform_real_distribution<> distribution(beg, end);
    return distribution(engine);
  }
}

#endif
