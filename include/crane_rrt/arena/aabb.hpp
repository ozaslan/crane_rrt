#ifndef ARENA_AABB_HPP
#define ARENA_AABB_HPP

#include <Eigen/Dense>
#include <limits>
#include <algorithm>
#include <iostream>

namespace arena {

template<typename Scalar = double>
class AABB {
public:
    using Self  = AABB<Scalar>;
    using Point = Eigen::Matrix<Scalar, 3, 1>;

    AABB() { reset(); }

    AABB(const Point& min_corner, const Point& max_corner)
        : mMin(min_corner), mMax(max_corner) {}

    Self& setMin(const Point& min_corner) {
        mMin = min_corner;
        return *this;
    }

    Self& setMax(const Point& max_corner) {
        mMax = max_corner;
        return *this;
    }

    const Point& min() const { return mMin; }
    const Point& max() const { return mMax; }

    Self& reset() {
        mMin = Point::Constant(std::numeric_limits<Scalar>::infinity());
        mMax = Point::Constant(-std::numeric_limits<Scalar>::infinity());
        return *this;
    }

    bool isValid() const {
        return (mMin.array() < mMax.array()).all();
    }

    bool checkCollision(const Self& other) const {
        for (int i = 0; i < 3; ++i)
            if (mMax[i] < other.mMin[i] || mMin[i] > other.mMax[i])
                return false;
        return true;
    }

    bool checkCollision(const Point& p1, const Point& p2) const {
        constexpr Scalar eps = std::numeric_limits<Scalar>::epsilon() * 100;
        Scalar tmin = 0.0;
        Scalar tmax = 1.0;
        Point d = p2 - p1;

        for (int i = 0; i < 3; ++i) {
            if (std::abs(d[i]) < eps) {
                if (p1[i] < mMin[i] || p1[i] > mMax[i])
                    return false;
            } else {
                Scalar invD = Scalar(1.0) / d[i];
                Scalar t1 = (mMin[i] - p1[i]) * invD;
                Scalar t2 = (mMax[i] - p1[i]) * invD;

                if (t1 > t2) std::swap(t1, t2);

                tmin = std::max(tmin, t1);
                tmax = std::min(tmax, t2);

                if (tmin > tmax)
                    return false;
            }
        }
        return true;
    }

    bool checkCollision(const Point& v0, const Point& v1, const Point& v2) const {
        const Point boxCenter = center();
        const Point boxHalfSize = scale() * Scalar(0.5);

        Point tv0 = v0 - boxCenter;
        Point tv1 = v1 - boxCenter;
        Point tv2 = v2 - boxCenter;

        Point e0 = tv1 - tv0;
        Point e1 = tv2 - tv1;
        Point e2 = tv0 - tv2;

        Point normal = e0.cross(e1);
        if (normal.norm() < Scalar(1e-12))
            return contains(v0) || contains(v1) || contains(v2);

        for (int i = 0; i < 3; ++i) {
            Scalar minTri = std::min({tv0[i], tv1[i], tv2[i]});
            Scalar maxTri = std::max({tv0[i], tv1[i], tv2[i]});
            if (minTri > boxHalfSize[i] || maxTri < -boxHalfSize[i])
                return false;
        }

        if (!overlapOnAxis(tv0, tv1, tv2, normal, boxHalfSize))
            return false;

        const Point axes[3] = { Point::UnitX(), Point::UnitY(), Point::UnitZ() };
        for (int i = 0; i < 3; ++i) {
            const Point& edge = (i == 0 ? e0 : (i == 1 ? e1 : e2));
            for (int j = 0; j < 3; ++j) {
                Point axis = edge.cross(axes[j]);
                if (!axis.isZero(1e-8)) {
                    if (!overlapOnAxis(tv0, tv1, tv2, axis, boxHalfSize))
                        return false;
                }
            }
        }

        return true;
    }

    bool contains(const Point& p) const {
        for (int i = 0; i < 3; ++i)
            if (p[i] < mMin[i] || p[i] > mMax[i])
                return false;
        return true;
    }

    bool contains(const Self& other) const {
        return contains(other.min()) && contains(other.max());
    }

    bool contains(const Point& p1, const Point& p2) const {
        return contains(p1) && contains(p2);
    }

    Point center() const {
        return (mMin + mMax) / Scalar(2);
    }

    Point scale() const {
        return mMax - mMin;
    }

    Scalar volume() const {
        if (!isValid()) return Scalar(0);
        Point s = scale();
        return s.x() * s.y() * s.z();
    }

    Self& expandToInclude(const Point& p) {
        for (int i = 0; i < 3; ++i) {
            mMin[i] = std::min(mMin[i], p[i]);
            mMax[i] = std::max(mMax[i], p[i]);
        }
        return *this;
    }

    Self& expandToInclude(const Self& other) {
        if (!other.isValid()) return *this;
        if (!this->isValid()) {
            *this = other;
            return *this;
        }
        mMin = mMin.cwiseMin(other.mMin);
        mMax = mMax.cwiseMax(other.mMax);
        return *this;
    }

    Self unionWith(const Self& other) const {
        if (!this->isValid()) return other;
        if (!other.isValid()) return *this;
        return Self(mMin.cwiseMin(other.mMin), mMax.cwiseMax(other.mMax));
    }

    Self intersectWith(const Self& other) const {
        if (!this->isValid() || !other.isValid()) return Self();
        Self result(mMin.cwiseMax(other.mMin), mMax.cwiseMin(other.mMax));
        return result.isValid() ? result : Self();
    }

    bool operator==(const Self& other) const {
        return mMin == other.mMin && mMax == other.mMax;
    }

    bool operator!=(const Self& other) const {
        return !(*this == other);
    }

    void print(const std::string& prefix = "") const {
        std::cout << prefix << (*this) << std::endl;
    }

    Self& expandUniform(Scalar margin) {
        mMin.array() -= margin;
        mMax.array() += margin;
        return *this;
    }

    Self& shrinkUniform(Scalar margin) {
        mMin.array() += margin;
        mMax.array() -= margin;
        if (!isValid()) reset();
        return *this;
    }

    template<typename Scalar_>
    friend std::ostream& operator<<(std::ostream& os, const AABB<Scalar_>& aabb) {
        os << "AABB[min: " << aabb.min().transpose()
           << ", max: " << aabb.max().transpose() << "]";
        return os;
    }

private:
    Point mMin;
    Point mMax;

    bool overlapOnAxis(const Point& v0, const Point& v1, const Point& v2,
                       const Point& axis, const Point& boxHalfSize) const {
        Scalar p0 = v0.dot(axis);
        Scalar p1 = v1.dot(axis);
        Scalar p2 = v2.dot(axis);
        Scalar minTri = std::min({p0, p1, p2});
        Scalar maxTri = std::max({p0, p1, p2});
        Scalar r = boxHalfSize.x() * std::abs(axis.x()) +
                   boxHalfSize.y() * std::abs(axis.y()) +
                   boxHalfSize.z() * std::abs(axis.z());
        return !(minTri > r || maxTri < -r);
    }
};

} // namespace arena

#endif // ARENA_AABB_HPP
