#ifndef GNG_SAM_HPP
#define GNG_SAM_HPP

// Std dependencies
#include <vector>
#include <numeric>

// Crate dependencies
#include <point.hpp>
#include <random.hpp>

namespace gng {

  using namespace std;
  using namespace gng::point;

  constexpr double MIN_DOUBLE = numeric_limits<double>::min();
  constexpr double MAX_DOUBLE = numeric_limits<double>::max();

  template <size_t N>
  class GNG {
  private:
    // Related abstractions
    struct Node {
      double        error;
      PointN<N>     point;
      vector<Node*> relatives;
    };

    // Private fields
    vector<Node*> nodes;

  public:
    GNG() = default;
    GNG(const GNG&) = delete;

    // stop criterion, net size
    void start(size_t desired_netsize) {
      Node* tmp_node;

      tmp_node = new Node();
      tmp_node->point = PointN<N>::random_in(MIN_DOUBLE, MAX_DOUBLE);
      nodes.push_back(tmp_node);

      tmp_node = new Node();
      tmp_node->point = PointN<N>::random_in(MIN_DOUBLE, MAX_DOUBLE);
      nodes.push_back(tmp_node);

      nodes[0]->relatives.push_back(nodes[1]);
      nodes[1]->relatives.push_back(nodes[0]);

      while (nodes.size() < desired_netsize) {
        PointN<N> signal = gen_random_signal();

      }
    }
//nose que weas

  private:

    PointN<N> gen_random_signal() {
      return PointN<N>::random_in(MIN_DOUBLE, MAX_DOUBLE);
    }

  };

}

#endif
