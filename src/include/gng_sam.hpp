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

  constexpr double MIN_DOUBLE = 0;
  constexpr double MAX_DOUBLE = 1;

  template <size_t N>
  class GNG {
  private:
    // Related abstractions
    struct Node;

    struct Edge {
      Age   age = 0;
      Node* nodeA;
      Node* nodeB;
    };

    struct Node {
      double     error = 0.0;
      PointN<N>  point;
      unordered_set<Node*> relatives;
      unordered_set<Edge*> relativeEdges;
    };

    // Private fields
    unordered_set<Node*> nodes;
    vector<Edge*>        edges;
    Age                  maximunAge;
    double               alfa;

  public:
    GNG() = default;
    GNG(const GNG&) = delete;
    GNG(double alfa): alfa(alfa) {}

    void setMaximumEdgeAge(Age newAge) { maximunAge = newAge; }
    Age  getMaximumEdgeAge() { return maximunAge; }

    // stop criterion, net size
    void start(size_t desiredNetsize, unsigned numberSteps, double beta, Age maximunAge) {
      setMaximumEdgeAge(maximunAge);
      Node* tmpNode;

      tmpNode = new Node();
      tmpNode->point = PointN<N>::randomIn(MIN_DOUBLE, MAX_DOUBLE);
      nodes.insert(tmpNode);

      tmpNode = new Node();
      tmpNode->point = PointN<N>::randomIn(MIN_DOUBLE, MAX_DOUBLE);
      nodes.insert(tmpNode);

      unsigned stepCounter  = 0;
      unsigned cycleCounter = 0;
      while (nodes.size() < desiredNetsize) {
        stepCounter += 1;

        PointN<N> signal = genRandomSignal(MIN_DOUBLE, MAX_DOUBLE);
        auto nearest = twoNearestNodes(signal);
        auto v = nearest.first;
        auto u = nearest.second;

        for (auto& n : v->relativeEdges)
          n->age += 1;

        v->error += pow(v->point.norma2() - signal.norma2(), 2);

        double constanteDesconocida = 0.5;
        double constanteDesconocidaPequenha = 0.2;
        v->point = v->point + (signal - v->point) * constanteDesconocida;
        for(auto& n: v->relatives)
          n->point = n->point + (signal - n->point) * constanteDesconocidaPequenha;

        // If there isn't a edge between u and v, create one
        if (v->relatives.find(u) == v->relatives.end()) {
          Edge* e = new Edge { .age = 0, .nodeA = v, .nodeB = u };
          v->relatives.insert(u);
          u->relatives.insert(v);
          v->relativeEdges.insert(e);
          u->relativeEdges.insert(e);
          edges.push_back(e);
        }

        // set age of edge (u <-> v) to 0
        auto vuEdgeIt = find_if(v->relativeEdges.begin(), v->relativeEdges.end(),
        [&u](auto& edge){
          return edge->nodeA == u || edge->nodeB == u;
        });

        Edge& vuEdge = *(*vuEdgeIt);
        vuEdge.age = 0;

        // remove oldest edges
        for (auto it = edges.begin(); it != edges.end(); ++it) {
          Edge* edge = *it; // just an alias

          Node* a = edge->nodeA;
          Node* b = edge->nodeB;

          // check if edge is young enough
          if (edge->age <= maximunAge)
            continue;

          // delete this edge because it's too old
          edges.erase(it);
          a->relativeEdges.erase(a->relativeEdges.find(edge));
          b->relativeEdges.erase(b->relativeEdges.find(edge));
          a->relatives.erase(a->relatives.find(b));
          b->relatives.erase(b->relatives.find(a));

          // we deleted its last edge
          if (a->relatives.empty()) {
            nodes.erase(nodes.find(a));
            delete a;
          }

          // we deleted its last edge
          if (b->relatives.empty()) {
            nodes.erase(nodes.find(b));
            delete b;
          }

          delete edge;
        }

        if (stepCounter == numberSteps) {
          createNewNode();
          stepCounter = 0;
          cycleCounter += 1;
        }

        // decrease all error by some 'beta' constant
        for (auto& node : nodes)
          node->error += beta * node->error;
      }
    }

  private:

    PointN<N> genRandomSignal(double beg, double end) {
      return PointN<N>::randomIn(beg, end);
    }

    double euclidianDistance(const PointN<N>& a, const PointN<N>& b){
      double distance = 0;
      for(int i = 0; i < N; i++)
        distance += pow(a[i] - b[i], 2);
      return sqrt(distance);
    }

    auto twoNearestNodes(const PointN<N>& point) {
      Node* a = nullptr;
      Node* b = nullptr;
      double distancea = -1, distanceb = -1;

      for (auto& n : nodes){
        if (distancea == -1) {
          distancea = euclidianDistance(n->point, point);
          a = n;
        } else if(distanceb == -1) {
          distanceb = euclidianDistance(n->point, point);
          b = n;
        } else{
          double d = euclidianDistance(n->point, point);

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

    auto largestError() {
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

    void decErrorByAlpha(Node* node) {
      node->error = alfa * (node->error);
    }

    void createNewNode() {
      auto qf = largestError();
      Node* q = qf.first;
      Node* f = qf.second;

      Node* newNode  = new Node();
      newNode->point = (q->point + f->point) / 2;

      // buscar arista que conecta ambos nodos en uno de los nodos
      for (auto it = q->relativeEdges.begin(); it != q->relativeEdges.end(); ++it) {
        Edge& edge = *(*it);
        if (edge.nodeA == f || edge.nodeB == f) {
          auto edge_it = find(edges.begin(), edges.end(), &edge);
          edges.erase(edge_it);
          q->relativeEdges.erase(it);
          break;
        }
      }

      for (auto it = f->relativeEdges.begin(); it != f->relativeEdges.end(); ++it) {
        Edge& edge = *(*it);
        if (edge.nodeA == q || edge.nodeB == q) {
          f->relativeEdges.erase(it);
          break;
        }
      }

      // delete f from q's relatives
      auto fit = q->relatives.find(f);
      q->relatives.erase(fit);

      // delete q from f's relatives
      auto qit = f->relatives.find(q);
      f->relatives.erase(qit);

      Edge* e1 = new Edge { .age = 0, .nodeA = newNode, .nodeB = q };
      Edge* e2 = new Edge { .age = 0, .nodeA = newNode, .nodeB = f };

      q->relatives.insert(newNode);
      f->relatives.insert(newNode);
      q->relativeEdges.insert(e1);
      newNode->relativeEdges.insert(e1);
      f->relativeEdges.insert(e2);
      newNode->relativeEdges.insert(e2);
      newNode->relatives.insert(q);
      newNode->relatives.insert(f);
      edges.push_back(e1);
      edges.push_back(e2);
      decErrorByAlpha(q);
      decErrorByAlpha(f);
      newNode->error = (q->error + f->error)/2;
      nodes.insert(newNode);
    }
  };

}

#endif
