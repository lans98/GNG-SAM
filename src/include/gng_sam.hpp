#ifndef GNG_SAM_HPP
#define GNG_SAM_HPP

// Std dependencies
#include <utility>
#include <vector>
#include <numeric>
#include <algorithm>
#include <unordered_set>

// Crate dependencies
#include <point.hpp>
#include <random.hpp>

namespace gng {

  using namespace std;
  using namespace point;

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
      double     error = 0.0;
      PointN<N>  point;
      unordered_set<Node*> relatives;
      unordered_set<Edge*> relative_edges;
    };

    // Private fields
    unordered_set<Node*> nodes;
    vector<Edge*>        edges;
    Age                  maximum_age;

  public:
    GNG() = default;
    GNG(const GNG&) = delete;

    void set_maximum_edge_age(Age new_age) { maximum_age = new_age; }
    Age  get_maximum_edge_age() { return maximum_age; }

    // stop criterion, net size
    void start(size_t desired_netsize, unsigned no_steps, double beta) {
      Node* tmp_node;

      tmp_node = new Node();
      tmp_node->point = PointN<N>::random_in(MIN_DOUBLE, MAX_DOUBLE);
      nodes.insert(tmp_node);

      tmp_node = new Node();
      tmp_node->point = PointN<N>::random_in(MIN_DOUBLE, MAX_DOUBLE);
      nodes.insert(tmp_node);

      unsigned step_counter = 0;
      unsigned cycle_counter = 0;
      while (nodes.size() < desired_netsize) {
        step_counter += 1;
        PointN<N> signal = gen_random_signal();
        auto nearest = two_nearest_nodes(signal);
        auto v = nearest.first;
        auto u = nearest.second;

        for (auto& n : v->relative_edges)
          n->age += 1;

        v->error += pow(v->point.norma2() - signal.norma2(), 2);
        
        // TODO: u->point +=... line 10 in paper
        // TODO: foreach ... line 11 in paper

        // If there isn't a edge between u and v, create one
        if (v->relatives.find(u) == v->relatives.end()) {
          Edge* e = new Edge { .age = 0, .node_a = u, .node_b = v };
          u->relatives.insert(v);
          v->relatives.insert(u);
          u->relative_edges.insert(e);
          v->relative_edges.insert(e);
          edges.push_back(e);
        }

        // set age of edge (u <-> v) to 0
        auto  vu_edge_iterator = v->relative_edges.find(u);
        Edge& vu_edge = *(*vu_edge_iterator);
        vu_edge.age = 0;

        // remove oldest edges
        for (auto it = edges.begin(); it != edges.end(); ++it) {
          Edge& edge = *(*it); // just an alias

          Node* a = edge.node_a;
          Node* b = edge.node_b;

          // check if edge is young enough
          if (edge.age <= maximum_age)
            continue;

          // delete this edge because it's too old
          edges.erase(it);

          // we deleted its last edge
          if (a->relatives.size() == 1) {
            auto a_it = nodes.find(a);
            nodes.erase(a_it);
            delete a;
          }

          // we deleted its last edge
          if (b->relatives.size() == 1) {
            auto b_it = nodes.find(b);
            nodes.erase(b_it);
            delete b;
          }
        }

        if (step_counter == no_steps) {
          create_new_node();
          step_counter = 0;
          cycle_counter += 1;
        }

        // decrease all error by some 'beta' constant
        for (auto& node : nodes)
          node.error += beta * node.error;
      }
    }

  private:

    PointN<N> gen_random_signal() {
      return PointN<N>::random_in(MIN_DOUBLE, MAX_DOUBLE);
    }

    double euclidian_distance(const PointN<N>& a, const PointN<N>& b){
      double distance = 0;
      for(int i = 0; i < N; i++)
        distance += pow(a[i] - b[i], 2);
      return sqrt(distance);
    }

    auto two_nearest_nodes(const PointN<N>& point) {
      Node* a;
      Node* b;
      double distancea = -1, distanceb = -1;

      for (auto& n : nodes){
        if (distancea == -1) {
          distancea = euclidian_distance(n->point, point);
          a = n;
        }
        else if(distanceb == -1) {
          distanceb = euclidian_distance(n->point, point);
          b = n;
        }
        else{
          double d = euclidian_distance(n->point, point);
          if( d < distancea ){
            distancea = d;
            a = n;
          } else if(d < distanceb) {
            distanceb = d;
            b = n;
          }
        }
      }

      return make_pair(a, b);
    }

    // TODO: implement
    void create_new_node() {

    }
  };

}

#endif
