#ifndef GNG_SAM_HPP
#define GNG_SAM_HPP

// Std dependencies
#include <set>
#include <queue>
#include <utility>
#include <vector>
#include <numeric>
#include <unordered_set>

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
      Age   age = 0;
      Node* node_a;
      Node* node_b;
    };

    struct Node {
      double     error   = 0.0;
      PointN<N>  point;
      unordered_set<Node*> relatives;
    };

    // Private fields
    set<Node*>   nodes;
    vector<Edge> edges;
    Age          maximum_age;

  public:
    GNG() = default;
    GNG(const GNG&) = delete;

    void set_maximum_edge_age(Age new_age) { maximum_age = new_age; }
    Age  get_maximum_edge_age() { return maximum_age; }

    // stop criterion, net size
    void start(size_t desired_netsize) {
      Node* tmp_node;

      tmp_node = new Node();
      tmp_node->point = PointN<N>::random_in(MIN_DOUBLE, MAX_DOUBLE);
      nodes.insert(tmp_node);

      tmp_node = new Node();
      tmp_node->point = PointN<N>::random_in(MIN_DOUBLE, MAX_DOUBLE);
      nodes.insert(tmp_node);

      while (nodes.size() < desired_netsize) {
        PointN<N> signal = gen_random_signal();
        auto nearest = two_nearest_nodes(signal);
        auto u = nearest.first;
        auto v = nearest.second;

        for (auto& n : u->relatives)
          n.age += 1;

        u->error += pow(u->point.norma2() - signal.point.norma2(), 2);
        // TODO: u->point +=...
        // TODO: foreach ...

        // If there isn't a edge between u and v, create one
        if (u->relatives.find(v) == u->relatives.end()) {
          u->relatives.insert(v);
          v->relatives.insert(u);
          edges.insert(Edge{ .age = 0, .node_a = u, .node_b = v });
        }

        update_edges();
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

    void update_edges() {


    }

    void reset_visited_marks() {
      for (auto& n : nodes)
        n->visited = false;
    }
  };

}

#endif
