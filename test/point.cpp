#include <iostream>
#include <point.hpp>

using namespace std;
using namespace gng::point;

int main() {
  PointN<3> point3d1 { 2, 1, 3 };
  PointN<3> point3d2 { 2, 3, 1 };

  cout << point3d1 + point3d2 << '\n';
  cout << point3d1 - point3d2 << '\n';
  cout << (point3d1 + point3d2)/2 << '\n';
  cout << (point3d1 - point3d2)/2 << '\n';
  cout << (point3d1 + point3d2)*2 << '\n';
  cout << (point3d1 - point3d2)*2 << '\n';
}
