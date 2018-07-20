#ifndef GNG_SAM_POINT_HPP
#define GNG_SAM_POINT_HPP

#include <array>
#include <cmath>
#include <type_traits>
#include <initializer_list>
#include <iostream>

#include <tools.hpp>
#include <random.hpp>

namespace gng {
namespace point {

    using namespace std;
    using namespace tools;
    using namespace random;

    enum Component : int { X, Y, Z };

    template <size_t N>
    class PointN {
    private:
        array<double, N> position;

    public:
        PointN() = default;
        PointN(const PointN&) = default;
        PointN(const initializer_list<double>& list) {
            expect(list.size() == N, "Dimension differs!");

            auto it = list.begin();
            for (size_t i = 0; i < list.size(); ++i, ++it)
                position[i] = *it;
        }

        PointN(array<double,N> pos): position(std::move(pos)) {}

        double& operator[](size_t dim) {
            expect(dim < N, "Dimension differs");
            return position[dim];
        }

        const double& operator[](size_t dim) const {
            expect(dim < N, "Dimesion differs");
            return position[dim];
        }

        double norma2() {
            double sum = 0;
            for (size_t i = 0; i < N; ++i)
                sum += position[i] * position[i];

            return sqrt(sum);
        }

        size_t getDimension() { return N; }

        PointN operator+(const PointN& r) {
            PointN result;
            for (size_t i = 0; i < N; ++i)
                result[i] = (this->position[i] + r.position[i]);
            return result;
        }

        PointN operator-(const PointN& r) {
            PointN result;
            for (size_t i = 0; i < N; ++i)
                result[i] = (this->position[i] - r.position[i]);
            return result;
        }

        PointN operator*(double m){
            PointN result;
            for (size_t i = 0; i < N; ++i)
                result[i] = this->position[i] * m;
            return result;
        }

        PointN operator/(double d) {
            PointN result;
            for (size_t i = 0; i < N; ++i)
                result[i] = this->position[i] / d;
            return result;
        }

        friend ostream& operator<<(ostream& out, const PointN& point) {
            if (N == 0) return out;

            out << "[ ";
            for (size_t i = 0; i < N - 1; ++i)
                out << point[i] << ", ";
            out << point[N - 1] << " ]";

            return out;
        }

        /** generate a random point in [beg, end] */
        static PointN randomIn(double beg, double end) {
            array<double, N> pos;
            for (size_t i = 0; i < N; ++i)
                pos[i] = random::randomIn(beg, end);

            return PointN(pos);
        }
    };

}
}

#endif
