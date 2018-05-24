#ifndef GNG_SAM_POINT_HPP
#define GNG_SAM_POINT_HPP

#include <array>
#include <type_traits>
#include <initializer_list>

#include <random.hpp>

namespace gng::point {

  using namespace std;
  using namespace gng::random;

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

    // More point methods if necessary later

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

#endif
