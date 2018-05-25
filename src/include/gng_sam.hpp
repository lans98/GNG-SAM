#ifndef GNG_SAM_HPP
#define GNG_SAM_HPP

// Std dependencies
#include <utility>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iostream>
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
    double               alfa;

  public:
    GNG() = default;
    GNG(const GNG&) = delete;
    GNG(double alfa): alfa(alfa) {}

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

      unsigned step_counter  = 0;
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
          Edge* e = new Edge { .age = 0, .node_a = v, .node_b = u };
          v->relatives.insert(u);
          u->relatives.insert(v);
          v->relative_edges.insert(e);
          u->relative_edges.insert(e);
          edges.push_back(e);
        }

        // set age of edge (u <-> v) to 0
        auto vu_edge_it = find_if(v->relative_edges.begin(), v->relative_edges.end(), [&v,&u](auto& edge){
          return edge->node_a == u || edge->node_b == u;
        });

        Edge& vu_edge = *(*vu_edge_it);
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
          node->error += beta * node->error;
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
        } else if(distanceb == -1) {
          distanceb = euclidian_distance(n->point, point);
          b = n;
        } else{
          double d = euclidian_distance(n->point, point);

          if (d < distancea) {
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

    auto largest_error() {
      double error = -1;

      Node* node_with_max_error;
      Node* neighbor_with_max_error;

      for(auto& n: nodes) {
        if(error <= n->error) {
          error = n->error;
          node_with_max_error = n;
        }
      }

      error = -1;
      for(auto& n: node_with_max_error->relatives){
        if(error <= n->error){
          error = n->error;
          neighbor_with_max_error = n;
        }
      }

      return make_pair(node_with_max_error, neighbor_with_max_error);
    }

    void dec_error_by_alpha(Node* node) {
      node->error = alfa * (node->error);
    }

    void create_new_node() {
      auto qf = largest_error();
      Node* q = qf.first;
      Node* f = qf.second;

      Node* new_node  = new Node();
      new_node->point = (q->point + f->point) / 2;

      // buscar arista que conecta ambos nodos en uno de los nodos
      for (auto it = q->relative_edges.begin(); it != q->relative_edges.end(); ++it) {
        Edge& edge = *(*it);

        if (edge.node_a == f || edge.node_b == f) {
          auto edge_it = find(edges.begin(), edges.end(), &edge);
          edges.erase(edge_it);
          q->relative_edges.erase(it);
          f->relative_edges.erase(it);
          break;
        }
      }

      // delete f from q's relatives
      auto fit = q->relatives.find(f);
      q->relatives.erase(fit);

      // delete q from f's relatives
      auto qit = f->relatives.find(q);
      f->relatives.erase(qit);

      Edge* e1 = new Edge { .age = 0, .node_a = new_node, .node_b = q };
      Edge* e2 = new Edge { .age = 0, .node_a = new_node, .node_b = f };

      q->relatives.insert(new_node);
      f->relatives.insert(new_node);
      q->relative_edges.insert(e1);
      new_node->relative_edges.insert(e1);
      f->relative_edges.insert(e2);
      new_node->relative_edges.insert(e2);
      edges.push_back(e1);
      edges.push_back(e2);
      dec_error_by_alpha(q);
      dec_error_by_alpha(f);
      new_node->error = (q->error + f->error)/2;
      nodes.insert(new_node);
    }
  };

}

#endif
