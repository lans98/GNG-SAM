#ifndef GNG_SAM_HPP
#define GNG_SAM_HPP

// Std dependencies
#include <vector>

// Crate dependencies
#include <point.hpp>

namespace gng {

  using namespace std;

  template <size_t N>
  class GNG {
  private:

    struct Node {
      PointN<N>     point;
      vector<Node*> relatives;
    };

  public:
    GNG() = default;
    GNG(const GNG&) = delete;

  };

}

#endif
