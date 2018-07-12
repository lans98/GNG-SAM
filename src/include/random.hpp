#ifndef GNG_SAM_RANDOM_HPP
#define GNG_SAM_RANDOM_HPP

#include <random>

namespace gng {
namespace random {

    using namespace std;

    // generate a random double in the interval [beg, end] (inclusive)
    double randomIn(double beg, double end) {
        static random_device randDev;
        static mt19937_64    engine(randDev());
        uniform_real_distribution<> distribution(beg, end);
        return distribution(engine);
    }

    // generate a random double in [0,1]
    double random() { return randomIn(0.0, 1.0); }

    // generate a random integer in the interval [beg, end] (inclusive)
    int randomIn(int beg, int end) {
        static random_device randDev;
        static mt19937_64        engine(randDev());
        uniform_int_distribution<> distribution(beg, end);
        return distribution(engine);
    }

}
}

#endif
