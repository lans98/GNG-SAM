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
#include <tools.hpp>
#include <point.hpp>
#include <random.hpp>
#include <data_ranges.hpp>

namespace gng {

    // Needed namespaces
    using namespace std;
    using namespace pcl;
    using namespace point;
    using namespace tools;
    using namespace data_ranges;
 
    // Type aliases
    using Age       = unsigned long;
    using ViewerPtr = shared_ptr<visualization::PCLVisualizer>;

    // PCL things
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Viewer"));                
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

        // Internal graph
        unordered_set<Node*> nodes;
        vector<Edge*>        edges;

        // Learning rate constants
        Age    maximumAge;
        double alfa;

        // Fields needed to draw
        ViewerPtr                 viewer;
        PointCloud<PointXYZ>::Ptr cloud;

        // Needed to generate random signals
        vector<DataRange<N>> dataRanges;

    public:
        GNG(): cloud(new PointCloud<PointXYZ>), viewer(new visualization::PCLVisualizer) { 
            static_assert(N == 2 || N == 3, "Only accepted 2D and 3D");
        }

        GNG(double alfa): cloud(new PointCloud<PointXYZ>), viewer(new visualization::PCLVisualizer), alfa(alfa) {
            static_assert(N == 2 || N == 3, "Only accepted 2D and 3D");
        }

        // delete copy constructor
        GNG(const GNG&) = delete;
 
        // get and set the maximumAge
        void setMaximumEdgeAge(Age newAge) { maximumAge = newAge; }
        Age  getMaximumEdgeAge() { return maximumAge; }

        // Setup data ranges
        void addDataRange(const DataRange<N>& dataRange) {
            dataRanges.push_back(dataRange);
        }

        // stop criterion, net size
        void start(size_t desiredNetsize, unsigned numberSteps, double beta, Age maximumAge) {
            expect(!dataRanges.empty(), "There is no way to generate random signals");

            viewer->setBackgroundColor(0, 0, 0);
            viewer->addPointCloud<PointXYZ>(cloud, "GNG");
            viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, "GNG");
            viewer->addCoordinateSystem(1.0);
            viewer->initCameraParameters();

            setMaximumEdgeAge(maximumAge);
            Node* tmpNode;

            tmpNode = new Node();
            tmpNode->point = genRandom();
            nodes.insert(tmpNode);

            tmpNode = new Node();
            tmpNode->point = genRandom();
            nodes.insert(tmpNode);

            unsigned stepCounter  = 0;
            unsigned cycleCounter = 0;
            while(!viewer->wasStopped()){

                if(nodes.size() >= desiredNetsize) return;
                if(nodes.size()%10 == 0){
                    viewer->spinOnce(100, true);
                
                    updateCloud();
                    cloud->width  = cloud->points.size();
                    cloud->height = 1;

                    if (!viewer->updatePointCloud(cloud, "new cloud")) {
                        viewer->addPointCloud<pcl::PointXYZ>(cloud, "new cloud");
                        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "new cloud");
                    }
                }
                

                stepCounter += 1;

                PointN<N> signal = genRandom();
                auto nearest = twoNearestNodes(signal);
                auto v = nearest.first;
                auto u = nearest.second;

                v->error += pow(v->point.norma2() - signal.norma2(), 2);

                double constanteDesconocida = 0.5;
                double constanteDesconocidaPequenha = 0.1;
                v->point = v->point + (signal - v->point) * constanteDesconocida;

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
                    if(a == v){

                        b->point = b->point + (signal - b->point) * constanteDesconocidaPequenha;
                        edge->age += 1;
                    }

                    // check if edge is young enough
                    if (edge->age <= maximumAge)
                        continue;

                    // delete this edge because it's too old
                    it = edges.erase(it);
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

                //boost::this_thread::sleep (boost::posix_time::microseconds (1000));
            }
        }

    private:

        PointN<N> genRandom() {
            expect(!dataRanges.empty(), "There is no way to generate random signals");

            int choosenRange = randomIn(0, dataRanges.size() - 1);
            return dataRanges[choosenRange].genRandom();
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
            cloud->points.clear();
            viewer->removeAllShapes();
            for(auto& n: nodes)
                addPointToCloud(n);
            
            for (auto it = edges.begin(); it != edges.end(); ++it, ++i) {
                Edge* edge = *it; 

                Node* a = edge->nodeA;
                Node* b = edge->nodeB;

                PointXYZ init = convertToPointXYZ(a);
                PointXYZ end    = convertToPointXYZ(b);
                viewer->addLine<pcl::PointXYZ> (init, end, to_string(i));
            }

            cout << nodes.size() << endl;
        }

        void addPointToCloud(Node* node){
            PointXYZ basic_point = convertToPointXYZ(node);
            cloud->points.push_back(basic_point);
        }

        PointXYZ convertToPointXYZ(Node* node){
            PointXYZ basic_point;

            basic_point.x = node->point[0];
            basic_point.y = node->point[1];

            size_t dim = node->point.getDimension();
            switch (dim) {
                case 2:
                    basic_point.z = 0;
                    break;
                case 3:
                    basic_point.z = node->point[2];
                    break;
                default:
                    throw invalid_argument("Node is greater than 3D");
            }
            
            return basic_point;
        }
    };
}

#endif
