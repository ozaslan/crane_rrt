#ifndef ARENA_SPHERE_HPP
#define ARENA_SPHERE_HPP

#include "primitive.hpp"
#include <cmath>

namespace arena {

template<typename Scalar = double>
class Sphere : public Primitive<Scalar> {
public:
    using Base   = Primitive<Scalar>;
    using Self   = Sphere<Scalar>;
    using Point  = typename Base::Point;
    using AABB   = typename Base::AABB;
    using Ptr    = typename Base::Ptr;
    using Mat3   = typename Base::Mat3;

    AABB getAABB() const override {
        const Point center = this->getPosition();
        const Mat3 R = this->getOrientation();
        const Point semi_axes = this->getScale() / Scalar(2);

        if (isUniformScale(semi_axes)) {
            const Point extents = Point::Constant(semi_axes.x());
            return AABB(center - extents, center + extents);
        }

        // Exact AABB half-extents for a rotated ellipsoid with semi-axes (a, b, c)
        const Point a2 = semi_axes.cwiseProduct(semi_axes);
        const Mat3 R2 = R.cwiseProduct(R);
        const Point extents = (R2 * a2).cwiseSqrt();
        return AABB(center - extents, center + extents);
    }

    Ptr clone() const override {
        return std::make_shared<Sphere>(*this);
    }

    bool checkCollision(const Base& other) const override {
        if (auto other_sphere = dynamic_cast<const Sphere*>(&other)) {
            return checkSphereSphereCollision(*other_sphere);
        }
        // Delegate to composite types (e.g., PrimitiveBundle)
        return other.checkCollision(*this);
    }

private:
    bool checkSphereSphereCollision(const Self& other) const {
        const Point pa = this->getPosition();
        const Point pb = other.getPosition();
        // Scale stores full diameters; convert to radii / semi-axes
        const Point half_a = this->getScale() * Scalar(0.5);
        const Point half_b = other.getScale() * Scalar(0.5);

        const bool a_uniform = isUniformScale(half_a);
        const bool b_uniform = isUniformScale(half_b);

        if (a_uniform && b_uniform) {
            // Case 1: both uniform spheres — direct squared-distance test is faster than AABB
            const Scalar r_sum = half_a.x() + half_b.x();
            return (pa - pb).squaredNorm() <= r_sum * r_sum;
        }

        if (!Base::checkRoughCollision(other)) {
            return false;
        }

        const Mat3 Ra = this->getOrientation();
        const Mat3 Rb = other.getOrientation();

        if (a_uniform) {
            // Case 2: sphere A vs ellipsoid B
            return checkSphereVsEllipsoid(pa, half_a.x(), pb, Rb, half_b);
        }

        if (b_uniform) {
            // Case 2 reversed: sphere B vs ellipsoid A
            return checkSphereVsEllipsoid(pb, half_b.x(), pa, Ra, half_a);
        }

        // Case 3: both are ellipsoids
        return checkEllipsoidVsEllipsoid(pa, Ra, half_a, pb, Rb, half_b);
    }

    template<typename Vec>
    bool isUniformScale(const Vec& s, typename Vec::Scalar tol = typename Vec::Scalar(1e-6)) const {
        return (std::abs(s.x() - s.y()) < tol) && (std::abs(s.x() - s.z()) < tol);
    }

    static Scalar directionalRadius(const Point& unit_dir, const Mat3& R, const Point& semi_axes) {
        const Point local_dir = R.transpose() * unit_dir;
        const Point scaled = local_dir.cwiseQuotient(semi_axes);
        const Scalar denom = scaled.norm();
        if (denom <= Scalar(1e-12)) {
            return Scalar(0);
        }
        return Scalar(1) / denom;
    }

    bool checkSphereVsEllipsoid(
        const Point& center_sphere, Scalar radius,
        const Point& center_ellipsoid, const Mat3& R, const Point& semi_axes) const
    {
        const Point d = center_sphere - center_ellipsoid;
        const Scalar dist = d.norm();
        if (dist < Scalar(1e-8)) return true;

        const Point dir = d / dist;
        const Scalar r_e = directionalRadius(dir, R, semi_axes);

        return dist <= (radius + r_e);
    }

    bool checkEllipsoidVsEllipsoid(
        const Point& c1, const Mat3& R1, const Point& semi_axes1,
        const Point& c2, const Mat3& R2, const Point& semi_axes2) const
    {
        const Point d = c2 - c1;
        const Scalar dist = d.norm();
        if (dist < Scalar(1e-8)) return true;

        const Point dir = d / dist;
        const Scalar r1 = directionalRadius(dir, R1, semi_axes1);
        const Scalar r2 = directionalRadius(dir, R2, semi_axes2);

        return dist <= (r1 + r2);
    }
};

} // namespace arena

#endif // ARENA_SPHERE_HPP
