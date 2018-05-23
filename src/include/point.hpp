#ifndef GNG_SAM_POINT_HPP
#define GNG_SAM_POINT_HPP

#include <array>
#include <type_traits>
#include <initializer_list>

namespace gng {

  using namespace std;

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
  };

}

#endif
