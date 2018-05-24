#ifndef GNG_SAM_POINT_HPP
#define GNG_SAM_POINT_HPP

#include <array>
#include <cmath>
#include <type_traits>
#include <initializer_list>

#include <random.hpp>

namespace gng {
namespace point {

  using namespace std;
  using namespace random;

  template <size_t N>
  class PointN {
  private:
    array<double, N> position;

  public:
    PointN() = default;
    PointN(const PointN&) = default;
    PointN(const initializer_list<double>& list) {
      if (list.size() != N)
        throw std::runtime_error("Dimension differs!");

      auto it = list.begin();
      for (size_t i = 0; i < list.size(); ++i, ++it)
        position[i] = *it;
    }

    PointN(array<double,N> pos): position(std::move(pos)) {}

    double& operator[](size_t dim) {
      if (dim >= N)
        throw std::runtime_error("Dimension differs!");

      return position[dim];
    }

    const double& operator[](size_t dim) const {
      if (dim >= N)
        throw std::runtime_error("Dimension differs!");

      return position[dim];
    }

    double norma2() {
      double sum = 0;
      for (size_t i = 0; i < N; ++i)
        sum += position[i] * position[i];

      return sqrt(sum);
    }

    size_t get_dimension() { return N; }

    // static methods
    static PointN random_in(double beg, double end) {
      array<double, N> pos;
      for (size_t i = 0; i < N; ++i)
        pos[i] = random::random_in(beg, end);

      return PointN(pos);
    }
  };

}
}

#endif
