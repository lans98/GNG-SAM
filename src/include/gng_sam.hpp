#ifndef GNG_SAM_HPP
#define GNG_SAM_HPP

// Std dependencies
#include <utility>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <unordered_set>

// pcl dependencies
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

// Crate dependencies
#include <point.hpp>
#include <random.hpp>

namespace gng {

  using namespace std;
  using namespace point;

  using Age = unsigned long;

  constexpr double MIN_DOUBLE = 0;
  constexpr double MAX_DOUBLE = 1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr gng_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Viewer"));        
  int i = 0;

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
    void start(size_t desired_netsize, unsigned no_steps, double beta, Age maximum_age) {
    
      viewer->setBackgroundColor (0, 0, 0);
      viewer->addPointCloud<pcl::PointXYZ> (gng_cloud, "GNG");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "GNG");
      viewer->addCoordinateSystem (1.0);
      viewer->initCameraParameters ();

      set_maximum_edge_age(maximum_age);
      Node* tmp_node;

      tmp_node = new Node();
      tmp_node->point = PointN<N>::random_in(MIN_DOUBLE, MAX_DOUBLE);
      nodes.insert(tmp_node);
    
      tmp_node = new Node();
      tmp_node->point = PointN<N>::random_in(MIN_DOUBLE, MAX_DOUBLE);
      nodes.insert(tmp_node);
    
      unsigned step_counter  = 0;
      unsigned cycle_counter = 0;
      
      while(!viewer->wasStopped()){
        viewer->spinOnce (100, true);
      //  while (nodes.size() < desired_netsize) {
          updateCloud();
          gng_cloud->width = (int) gng_cloud->points.size ();
          gng_cloud->height = 1;

          if(!viewer->updatePointCloud(gng_cloud, "new cloud")){
              viewer->addPointCloud<pcl::PointXYZ> (gng_cloud, "new cloud");
              viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "new cloud");
          }

          step_counter += 1;

          PointN<N> signal = gen_random_signal(MIN_DOUBLE, MAX_DOUBLE);
          auto nearest = two_nearest_nodes(signal);
          auto v = nearest.first;
          auto u = nearest.second;

          for (auto& n : v->relative_edges)
            n->age += 1;

          v->error += pow(v->point.norma2() - signal.norma2(), 2);

          double constanteDesconocida = 1;
          v->point = v->point + (signal - v->point) * constanteDesconocida;
          for(auto& n: v->relatives)
            n->point = n->point + (signal - n->point) * constanteDesconocida;

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
          auto vu_edge_it = find_if(v->relative_edges.begin(), v->relative_edges.end(),
          [&u](auto& edge){
            return edge->node_a == u || edge->node_b == u;
          });

          Edge& vu_edge = *(*vu_edge_it);
          vu_edge.age = 0;

          // remove oldest edges
          for (auto it = edges.begin(); it != edges.end(); ++it) {
            Edge* edge = *it; // just an alias

            Node* a = edge->node_a;
            Node* b = edge->node_b;

            // check if edge is young enough
            if (edge->age <= maximum_age)
              continue;

            // delete this edge because it's too old
            edges.erase(it);
            a->relative_edges.erase(a->relative_edges.find(edge));
            b->relative_edges.erase(b->relative_edges.find(edge));
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

          if (step_counter == no_steps) {
            create_new_node();
            step_counter = 0;
            cycle_counter += 1;
          }

          // decrease all error by some 'beta' constant
          for (auto& node : nodes)
            node->error += beta * node->error;
        //}
        boost::this_thread::sleep (boost::posix_time::microseconds (1000));

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

    void updateCloud(){
      gng_cloud->points.clear();
      viewer->removeAllShapes();
      for(auto& n: nodes) {
        addPointToCloud(n);
      }
      for (auto it = edges.begin(); it != edges.end(); ++it) {
            Edge* edge = *it; 

            Node* a = edge->node_a;
            Node* b = edge->node_b;

            pcl::PointXYZ init = convertToPointXYZ(a);
            pcl::PointXYZ end = convertToPointXYZ(b);

            std::ostringstream stm;
            stm<<i;
            viewer->addLine<pcl::PointXYZ> (init, end, stm.str());
            i++;
      }
      cout<<nodes.size()<<endl;
    }

    void addPointToCloud(Node * node){
      pcl::PointXYZ basic_point = convertToPointXYZ(node);
      gng_cloud->points.push_back(basic_point);
    }

    pcl::PointXYZ convertToPointXYZ(Node * node){
      pcl::PointXYZ basic_point;
      basic_point.x = node->point[0];
      basic_point.y = node->point[1];
      basic_point.z = 0;
      return basic_point;
    }

  };

}

#endif
