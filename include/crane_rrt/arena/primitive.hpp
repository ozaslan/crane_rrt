#ifndef _ARENA_PRIMITIVE_HPP
#define _ARENA_PRIMITIVE_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include "aabb.hpp"
#include "color.hpp"

namespace arena {

template<typename Scalar>
class Primitive {
public:
    using Self    = Primitive<Scalar>;
    using Ptr     = std::shared_ptr<Self>;
    using Point   = Eigen::Matrix<Scalar, 3, 1>;
    using Mat3    = Eigen::Matrix<Scalar, 3, 3>;
    using Pose    = Eigen::Transform<Scalar, 3, Eigen::Affine>;
    using AABB    = arena::AABB<Scalar>;
    using Color   = arena::Color<Scalar>;

    virtual ~Primitive() = default;

    Primitive& withTag(const std::string& tag) {
        mTag = tag;
        return *this;
    }

    // ----- Pose -----
    const std::string& getTag() const { return mTag; }
    void setTag(const std::string& tag) { mTag = tag; }

    void setPose(const Pose& pose) { mPose = pose; }
    void setPose(const Point& position, const Mat3& orientation) {
        mPose.linear() = orientation;
        mPose.translation() = position;
    }

    void setPosition(const Point& position) {
        mPose.translation() = position;
    }

    Pose  getPose() const { return mPose; }
    Point getPosition() const { return mPose.translation(); }
    Mat3  getOrientation() const { return mPose.linear(); }

    // ----- Scale -----
    void setScale(const Point& scale) { mScale = scale; }
    Point getScale() const { return mScale; }

    // ----- Color -----
    void setColor(const Color& color) { mColor = color; }
    virtual Color getColor() const { return mColor; }

    // ----- AABB -----
    virtual AABB getAABB() const = 0;
    virtual Ptr  clone() const = 0;

    virtual bool checkRoughCollision(const Self& other) const {
        return getAABB().checkCollision(other.getAABB());
    }

    virtual bool checkCollision(const Self& other) const {
        (void)other;
        throw std::logic_error("Primitive::checkCollision base overload should never be called directly.");
    }

    virtual void clear() {
        ;
    }
    
protected:
    std::string mTag = "";
    Pose  mPose      = Pose::Identity();
    Point mScale     = Point::Ones();
    Color mColor     = Color::White();
};

} // namespace arena

#endif // ARENA_Primitive_HPP
