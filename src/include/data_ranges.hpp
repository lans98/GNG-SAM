#ifndef GNG_SAM_DATA_RANGES_HPP
#define GNG_SAM_DATA_RANGES_HPP

// std crates
#include <functional>

// project crates
#include <point.hpp>

namespace gng {
namespace data_ranges {

    using namespace std;
    using namespace point;

    /**
     * Used in the GNG to bound limits, one way is to use
     * a fixed set of points, the other way is to determine 
     * a range for <x,y> and a BoundFunction that can be a
     * mathematical function or a logic function checking 
     * if the generated point is on the set or not.
     */
    template <size_t N>
    class ContinousDataRange {
    public:
        using BoundFunction = function<bool (PointN<N>&)>;

    private:
        array<double, N> mins;
        array<double, N> maxs;
        BoundFunction checkBound;

    public:
        ContinousDataRange() = default;
        ContinousDataRange(const ContinousDataRange&) = default;
        ContinousDataRange(array<double, N> mins, array<double, N> maxs, BoundFunction fn): mins(move(mins)), maxs(move(maxs)), checkBound(fn) {}

        PointN<N> genRandom() { throw runtime_error("Only 2D or 3D"); }

        void setMins(array<double, N> mins) { this->mins = move(mins); }
        void setMaxs(array<double, N> maxs) { this->maxs = move(maxs); }
        void setBoundFunction(const BoundFunction& cbf) { checkBound = cbf; }

        double& atMins(size_t index) { return mins[index]; }
        double& atMaxs(size_t index) { return maxs[index]; }
    };

    /** Specialization for 2D ContinousDataRange */
    template <>
    PointN<2> ContinousDataRange<2>::genRandom() {
        PointN<2> point;

        do {
            point[X] = randomIn(mins[X], maxs[X]);
            point[Y] = randomIn(mins[Y], maxs[Y]);
        } while(!checkBound(point));

        return point;
    }

    /** Specialization for 3D ContinousDataRange */
    template <>
    PointN<3> ContinousDataRange<3>::genRandom() {
        PointN<3> point;

        do {
            point[X] = randomIn(mins[X], maxs[X]);
            point[Y] = randomIn(mins[Y], maxs[Y]);
            point[Z] = randomIn(mins[Z], maxs[Z]);
        } while(!checkBound(point));

        return point;
    }

    template <size_t N>
    class DiscreteDataRange {
    private:
        vector<PointN<N>> point_set;

    public:
        DiscreteDataRange() = default;
        DiscreteDataRange(const DiscreteDataRange&) = default;
        DiscreteDataRange(vector<PointN<N>> points): point_set(points) {}
    
        PointN<N> genRandom() { throw runtime_error("Only 2D or 3D"); }

        void push_point(const PointN<N>& point) {
            point_set.push_back(point);
        }
    };

    /** Specialization for 2D DiscreteDataRange */
    template <>
    PointN<2> DiscreteDataRange<2>::genRandom() {
        PointN<2> point;

        int pos = randomIn(int(0), int(point_set.size()));
        return point_set[pos];
    }

    /** Specialization for 3D DiscreteDataRange */
    template <>
    PointN<3> DiscreteDataRange<3>::genRandom() {
        PointN<3> point;

        int pos = randomIn(int(0), int(point_set.size()));
        return point_set[pos];
    }

}
}

#endif
