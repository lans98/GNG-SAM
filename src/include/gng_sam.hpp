#ifndef GNG_SAM_HPP
#define GNG_SAM_HPP

// Std dependencies
#include <set>
#include <utility>
#include <vector>
#include <numeric>

// Crate dependencies
#include <point.hpp>
#include <random.hpp>

namespace gng {

  using namespace std;
  using namespace gng::point;

  using Age = unsigned long;

  constexpr double MIN_DOUBLE = numeric_limits<double>::min();
  constexpr double MAX_DOUBLE = numeric_limits<double>::max();

  template <size_t N>
  class GNG {
  private:
    // Related abstractions
    struct Node;

    struct Edge {
      Age   age;
      Node* relative;

      bool operator<(const Edge& r) { return relative < r.relative; }
      bool operator>(const Edge& r) { return relative > r.relative; }
      bool operator==(const Edge& r) { return relative == r.relative; }
    };

    struct Node {
      double     error;
      PointN<N>  point;
      set<Edge>  relatives;
    };

    // Private fields
    vector<Node*> nodes; // the net

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

      while (nodes.size() < desired_netsize) {
        PointN<N> signal = gen_random_signal();
        auto nearest = two_nearest_nodes(signal);
        auto u = nearest.first;
        auto v = nearest.second;

        for (auto& n : u->relatives)
          n.age += 1;

        u->error += pow(u->point.norma2() - signal.point.norma2(), 2);
        // u->point +=


      }
    }

  private:

    PointN<N> gen_random_signal() {
      return PointN<N>::random_in(MIN_DOUBLE, MAX_DOUBLE);
    }

    /*
    pair<Node*, Node*> two_nearest_nodes(const PointN<N>& point) {

    }
    */
  };

}

#endif
